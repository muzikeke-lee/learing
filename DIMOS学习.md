# DIMOS学习

> 分册 **2 / 9** ｜ 上级 [学习文档](学习文档.md) ｜ 术语 [术语表](术语表.md)
> 前置：[平台与部署学习](平台与部署学习.md) ｜ 最近更新 2026-09-07

---

## 0. 这册学什么

**DIMOS 框架本体** —— 我所有自研机器人能力的宿主。学完应该能：拿到一个 blueprint 就画出 module 连接图，
并说清「这个功能为什么是 module、那个功能为什么是独立进程」。

上游：[dimensionalOS/dimos](https://github.com/dimensionalOS/dimos)，本地基线 **v0.0.11**（vendored，不是 pip 装的）。

---

## 1. 项目与文件

### 1.1 框架本体

| 路径（以 `dimosmain/` 为例） | 是什么 |
|---|---|
| `dimos/core/blueprints.py` | ★**我打了补丁的地方**：autoconnect 支持 `Module \| None` 可选 peer |
| `DIMOS_UPSTREAM_README.md` | 上游原版 README（和我们的 `README.md` 分开放，别混） |
| `AGENTS.md` / `AI_POLICY.md` / `CLA.md` / `CONTRIBUTING.md` | 上游治理文件 |
| `pyproject.toml` / `setup.py` / `uv.lock` / `flake.nix` | 依赖与环境（uv + nix 双轨） |
| `docker/` | 容器化 |
| `stubs/` `native/` | 类型桩 / 原生扩展 |

### 1.2 M20 自研包 `dimos/robot/deeprobotics/m20/`

| 文件 | 职责 |
|---|---|
| `connection.py` | UDP 运控 + 姿态/转向 + gait 切换 + **急停** |
| `velocity_controller.py` | 速度环 |
| `status_listener.py` | 状态上报 |
| `waypoint_registry.py` | 航点 |
| `camera.py` | RTSP 取图 |
| `path_planner.py` | A\* 占据栅格规划 → [分册 6](规划与导航学习.md) |
| `safety_monitor.py` | 雷达近障急停（UDP 收 sidecar）→ [分册 3](激光雷达学习.md) |
| `patrol_behaviors.py` | 导航/巡逻编排（闭环到点 + 站起再走）→ [分册 6](规划与导航学习.md) |
| `voice/{blueprint_b, qwen_agent, wake_word}` | 语音 → [分册 8](语音交互学习.md) |
| `traversal/`（**只在 `3DMap_NP-MFS/dimos-m20` 那棵树里**） | 多楼层可通行性图 → [分册 6](规划与导航学习.md) |

### 1.3 并跑的独立进程 `runtime/`（部署到 106 home，**不是 module**）

| 文件 | 职责 |
|---|---|
| `voice_io_runner.py` | 命令服务，TCP 端口由 `M20_VOICE_SERVER_PORT` 指定（9700） |
| `gait_daemon.py` | `/GAIT` 常驻 DDS 发布，姿态调节，**~0.5s 热切换** |
| `m20_lidar_sidecar.py` | rclpy `/lidar_relayed` → UDP 7777 |
| `voice_frontend_104.py` | 跑在 104 的豆包 ASR/TTS 前端 |

### 1.4 其他 overlay

| 路径 | 是什么 |
|---|---|
| `bridge/lidar_imu_bridge/` | C++ ROS 节点：`rt/LIDAR/POINTS` → `rt/lidar_relayed` |
| `drdds_overlay/` | 厂商 DDS msg/srv IDL（`gait_daemon` 依赖 `drdds.msg.Gait`） |
| `tools/` | 诊断工具：`relocalize` / `gait_tool` / `udp_test` / `state_monitor` / `planner_viz` / `waypoint_recorder` |
| `config/` | waypoints / 地图 / `m20_env.sh.example`（**脱敏版**） |
| `m20_docs/` | 项目文档 + 厂商手册 + **逆向摘要** + 地图 |
| `deploy/deploy.py` | 一键整栈部署 → [分册 1](平台与部署学习.md) |

---

## 2. 核心机制

### 2.1 blueprint = 装配声明

DIMOS 的核心抽象：blueprint 描述「哪些 module、怎么连」，autoconnect 负责按声明把 module 的端口接起来。
读法：**先读 blueprint 看拓扑，再点开单个 module 看实现** —— 反过来读会迷路。

### 2.2 ★ 可选 peer 补丁

上游 autoconnect 要求每个 peer 都存在。我们在 `dimos/core/blueprints.py` 打了补丁，让 `Module | None`
类型的 peer 可以缺省。

**为什么需要**：M20 有些能力是可选的（比如某台机器上没起雷达 sidecar，`safety_monitor` 就没有上游）。
没有这个补丁，缺一个可选组件整个 blueprint 装配就失败。

> 这是**唯一一处**改动上游 `core/` 的地方 —— 升级 dimos 版本时第一个要重新打的补丁。

### 2.3 module 还是独立进程？

`runtime/` 下四个东西都跑在 106，但**不在 dimos 进程里**。判断标准看它们的共性：

| 进程 | 为什么不做成 module |
|---|---|
| `gait_daemon.py` | 需要**常驻 DDS 发布**，生命周期和 blueprint 不一致 |
| `m20_lidar_sidecar.py` | 需要 **rclpy**（ROS 客户端库），和 dimos 主进程的运行时冲突 |
| `voice_io_runner.py` | 对外提供 **TCP 服务**，要独立于 blueprint 起停 |
| `voice_frontend_104.py` | **跑在另一台主机**（104，音频硬件在那） |

⇒ 归纳：**需要独立生命周期、异构运行时、或在别的主机上**的，做独立进程；纯数据流处理的，做 module。

### 2.4 姿态调节：DDS 直发绕过厂商限制

厂商把「姿态调节」只绑在遥控器上，接口层没有暴露。逆向发现它其实是 **`SixAxisStand` 步态**
（GaitParam `0xF001` = 61441），**DDS 直发 `/GAIT` 就能复现** ⇒ 做成 `gait_daemon.py` 常驻发布，~0.5s 热切换。

这是「厂商没给接口 ≠ 做不到」的典型案例，也是 `drdds_overlay/` 存在的原因（要拿到厂商的 msg 定义才能发）。

---

## 3. 🔴 五份副本：读代码前先确认在哪棵树

| 树 | 分支 / 用途 | 独有内容 |
|---|---|---|
| `dimosmain/` | M20 主线（语音 + 自主导航） | `bridge/` `config/` `m20_docs/` `drdds_overlay/` |
| `agibot-d1/dimos-m20/` | `feat/agibot-d1`，D1 穿门 nav 线 | `agibot/d1/*` 包 |
| `agibot-d1/D1sidefellow/` | `feature/d1-side-follow`，**狗上实际部署** | 侧跟随全栈；merge-base `84be476` |
| `agibot-d1/t01/` | `t01` 平台线（待确认） | 配套 `~/dimos-t01-venv/` |
| `3DMap_NP-MFS/dimos-m20/` | 多楼层规划 fork | `traversal/` 包 + `tools/trav_*` |
| `dimostest/` | 待确认（早期实验树？） | 少 `bridge/` `config/` `m20_docs/` |

**判断当前在哪棵树的快捷办法**：看有没有 `traversal/`（→ NP-MFS）、有没有 `bridge/`（→ dimosmain）、
有没有 `agibot/d1/`（→ D1 线）。

🔴 **D1 运行时用 `agibot/d1/*` 不是 `deeprobotics/m20/*`** —— 目录长得像，改错文件 = 部署了不生效，
而且**没有任何报错**，是最难自查的一类错。

---

## 4. 动手清单

- [ ] 读 `dimos/core/blueprints.py`，找到可选 peer 补丁那几行，写清它改了什么判断
- [ ] 挑一个已有 blueprint（如 `voice/blueprint_b`），画出 module 连接图 + 标出哪些 peer 是可选的
- [ ] 用 `git log` 对比五棵树的 HEAD 和分支，做成一张表贴进笔记
- [ ] `tools/` 六个诊断工具各跑一次（能离线跑的先离线跑），各写一句话用途
- [ ] 读 `drdds_overlay/` 里 `Gait` 的 IDL 定义，对上 `gait_daemon.py` 发的字段
- [ ] 在 D1 树里同时打开 `agibot/d1/` 和 `deeprobotics/m20/` 的同名文件，看它们差多少

---

## 5. 自测问题

- [ ] blueprint 的 autoconnect 怎么把 module 接起来？读一个 blueprint 的正确顺序是什么？
- [ ] `Module | None` 那个补丁解决的是什么问题？不打会怎样？升级 dimos 版本时为什么它最危险？
- [ ] `runtime/` 下的四个进程为什么是**独立进程**而不是 module？归纳出判断标准。
- [ ] `m20_lidar_sidecar.py` 具体是因为什么技术原因不能做成 module？
- [ ] 姿态调节为什么要靠 DDS 直发 `/GAIT`（`SixAxisStand`，GaitParam `0xF001`=61441）而不是走厂商接口？
- [ ] `drdds_overlay/` 为什么必须存在？删掉它 `gait_daemon` 会怎样？
- [ ] 只看目录内容，怎么快速判断自己在哪棵 dimos 树里？
- [ ] D1 上改了 `deeprobotics/m20/patrol_behaviors.py` 然后部署，会发生什么？**为什么难以自查**？

---

## 6. 笔记（日期倒序）

### 模板（复制这段往下写）

```markdown
### 2026-XX-XX ｜ <主题>
- **在学**：<项目/文件路径>
- **搞懂了**：<一句话结论，重点写「从代码里看不出来的」>
- **踩的坑**：<现象 → 真实原因 → 判据>
- **代码位置**：`path/to/file.py:123`
- **已回写到**：<哪份权威文档；没回写就写「待回写」>
- **仍不懂**：<搬到第 7 节>
```

<!-- 从这里往下写笔记 -->

---

## 7. 本册疑问

- [ ] 上游 dimos 现在什么版本？v0.0.11 落后多少？升级成本主要在哪（可选 peer 补丁 + 各 robot 包）？
- [ ] 五棵树能否收敛成一份 + 分支？先用 md5 对拍找出各自孤本
- [ ] `agibot/d1/*` 和 `deeprobotics/m20/*` 有多少是可以抽公共基类的？
- [ ] `dimos/robot/` 下除了 `deeprobotics` 和 `agibot`，上游还带哪些机器人？有可复用的吗？
- [ ] `stubs/` `native/` 是上游的还是我们加的？
- [ ] uv 和 nix 两套环境定义在实际部署里用的是哪套？
