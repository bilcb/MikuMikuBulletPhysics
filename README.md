# MikuMikuBulletPhysics

在 Blender 中运行 MMD（MikuMikuDance）物理模拟的插件。通过加载原生 C++ 共享库（Bullet3 物理引擎），为 mmd_tools 导入的 PMX 模型提供与 MMD 一致的刚体物理模拟。

- 项目地址：<https://github.com/bilcb/MikuMikuBulletPhysics>
- 许可证：MIT License

## 功能

- **实时物理模拟** — 基于 Bullet3，以 60Hz 固定步长运行（面板可选 30/60/90/120/240Hz）
- **PMX/VMD/VPD 解析** — 原生 C++ 解析 PMX 模型（骨骼、刚体、关节、Morph）、VMD 动画（骨骼关键帧、表情、IK 开关、自阴影距离）、VPD 姿态
- **6 级骨骼名匹配** — 从 rigid bone_id 到 regex 后缀剥离的 fallback，覆盖 mmd_tools 导入 + 手动 rig
- **Blender 集成** — 侧边栏面板 (View3D → Sidebar → MMBP)，一键启停，参数可视化调节
- **IK 求解** — CCD IK 求解器，支持单轴/多轴约束、万向锁处理、180° 退化保护
- **VMD 贝塞尔插值** — 按 VMD 曲线插值骨骼动画，支持牛顿法+Bisection 混合反函数求解
- **物理渐进混合** — 防止剧烈动画变化导致物理爆炸（参考 saba SyncPhysics）
- **增量 VMD 合并** — 支持多次加载 VMD 文件叠加动画
- **后物理骨骼** — 支持 `transAfterPhys` 标记的骨骼在物理模拟后更新
- **Morph 动画** — 支持 Group/Vertex/Bone/UV/Material/Flip/Impulse 全类型 Morph 解析
- **弹簧阻尼可调** — 关节弹簧阻尼系数可在 UI 中调节
- **PMX/VMD 数据校验** — 解析时检测 NaN/Inf、shapeType/mode/group 范围、质量/阻尼负数、IK 循环上限、joint 索引越界等

## 安装

### 前置条件

- Blender 4.2+
- [mmd_tools](https://github.com/MMD-Blender/blender_mmd_tools) 插件（用于导入 PMX 模型）

### 安装步骤

1. 克隆仓库（含子模块）：
   ```bash
   git clone --recurse-submodules https://github.com/bilcb/MikuMikuBulletPhysics.git
   ```
2. 在 Blender 中：`Edit → Preferences → Add-ons → Install` → 选择项目目录
3. 启用 `MikuMikuBulletPhysics`

也可将项目文件夹复制到 Blender 插件目录：

- Windows：`%APPDATA%\Blender Foundation\Blender\4.2\scripts\addons\`
- Linux：`~/.config/blender/4.2/scripts/addons/`
- macOS：`~/Library/Application Support/Blender/4.2/scripts/addons/`

### 编译 C++ 库（可选）

仓库根目录已包含预编译的 `mmbp_physics.dll`（Windows x64），可直接使用。如需自行编译：

#### 依赖

- CMake ≥ 3.20
- C++20 编译器（GCC ≥ 11, MSVC ≥ 16.10, Clang ≥ 14）
- Bullet3（已作为 git submodule 包含：`src/bullet3/`）

#### 编译

```bash
cd src
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

产出物 `mmbp_physics.dll`（Windows）/ `.so`（Linux）/ `.dylib`（macOS）复制到项目根目录（与 `__init__.py` 同级）。

## 使用

### 快速开始

1. 使用 mmd_tools 导入 PMX 模型
2. 在 3D View 侧边栏打开 **MMBP** 标签页
3. 选择模型根对象，点击 **Load PMX** 加载同一 PMX 文件
4. （可选）点击 **Load VMD** 加载动画
5. 点击 **Start Physics** 开始实时物理模拟
6. 点击 **Stop** 停止

### 物理参数

所有参数均可在侧边栏 UI 中调节（仅在物理停止时可修改）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| Gravity | `(0, -98, 0)` | MMD 单位（dm/s²），约地球重力的 10 倍 |
| Solver Iterations | 10 | 约束求解迭代次数（1-100） |
| Physics Rate | 60 Hz | 物理固定频率，面板可选 30/60/90/120/240Hz |
| Max Substeps | 6 | 防止帧率过低时物理失控（spiral-of-death 防护） |
| Scale (0=auto) | 0 | PMX→Blender 缩放系数，0 从 mmd_tools 自动检测 |
| Spring Damping | 0.3 | 关节弹簧阻尼（0=无阻尼，1=临界阻尼） |
| Write Interval | 1 | 每 N 个 substep 写回一次骨骼（越大越省 CPU） |
| Log Level | WARN | C++ 引擎日志级别（DEBUG/INFO/WARN/ERROR） |

### 行为说明

- 开始模拟后会临时禁用 Blender 内置刚体世界，并静音 mmd_tools 的物理跟随约束
- 模拟停止后自动恢复所有 Blender 状态（Action、NLA、RigidBody World、约束）
- 物理运行时自动关闭 `use_global_undo`，防止撤销历史膨胀
- Blender 加载新文件或保存时自动停止物理
- 使用引用计数管理 `disable_all/restore_all`，支持嵌套调用

## 架构概览

### 5 阶段物理流水线

```
PhysicsPipeline::step()
  ├─ Stage 1: animate()       — VMD 关键帧求值（贝塞尔插值）+ IK 开关 + Morph
  ├─ Stage 2: prePhysics()    — 前物理骨骼变换更新 + IK 求解
  ├─ Stage 3: simulate()      — 激活刚体 → Bullet 步进 → syncBoneTransforms → cascadeAll
  ├─ Stage 4: postPhysics()   — 后物理骨骼变换更新 + IK 求解
  └─ Stage 5: output()        — 全局变换 → 父相对局部 → rest matrix 转换 → 输出
```

- **cascadeAll**: 物理写回后从根骨骼单次级联，对有刚体的子骨骼同步位置不覆盖旋转
- **MMD 碰撞组**: 16 组碰撞掩码通过 `btOverlapFilterCallback` 过滤，地面始终可碰撞
- **软约束兼容**: 弹簧关节 min==max 限位时自动添加微小 epsilon 模拟 MMD Bullet 2.75 软约束

### 模块架构

```
Blender Python 层
  __init__.py     — 插件注册、Scene Properties
  mmbp/core/
    state.py      — State + StateManager（每 Armature 物理状态）
    lib.py        — ctypes DLL 加载 + 函数签名注册
    bonemap.py    — PMX↔Blender 骨骼映射（含批量名获取）
    loop.py       — Timer 回调驱动物理循环
  mmbp/ops/
    load.py       — Load PMX / Load VMD 操作器
    start.py      — Start / Stop 操作器
  mmbp/bridge/
    ui.py         — 侧边栏 UI 面板
        │ ctypes FFI (30+ C API 函数)
        ▼
C++ 共享库 (mmbp_physics.dll / .so / .dylib)
  bridge/capi/    — C API 入口（异常安全、句柄验证）
  bridge/engine/  — Engine 协调器 + PhysicsPipeline 流水线
  bridge/adapter/ — BlenderAdapter 坐标系转换
  pmx/            — PMX 2.0 解析器（UTF-16/UTF-8 自动检测）
  vmd/            — VMD 解析器（Shift-JIS 编码、关键帧排序、警告收集）
  model/          — BoneNode 树 + Morph 权重管理
  anim/           — VMD 动画求值（贝塞尔插值、IK 关键帧、Morph）
  ik/             — CCD IK 求解器（平面模式、万向锁处理）
  node/           — 骨骼变换树（拓扑排序优化、附加变换、递归深度保护）
  physics/        — Bullet3 物理世界封装（刚体、关节、弹簧、碰撞过滤）
  math/           — 欧拉角↔四元数转换（YXZ 序列、万向锁 Candidates）
  util/           — 编码工具（UTF-16LE/UTF-8）+ 日志系统（Meyers Singleton）
```

### 线程模型

- Blender 主线程执行所有操作（UI + Timer 回调 + Operator）
- C++ 引擎单线程执行，无并发问题
- Timer 回调以物理固定步长为间隔驱动模拟循环

## 安全特性

- **C API 异常安全** — 所有 `extern "C"` 函数用 try-catch 包裹
- **句柄验证** — 使用 magic number (`0x4D4D5048`) 验证 Engine 指针有效性
- **NaN/Inf 防护** — 输出到 Blender 前检查 `isfinite()`，异常值替换为安全默认值
- **参数校验** — 所有 C API 入口检查空指针和无效参数
- **循环引用检测** — PMX 加载时 DFS 检测骨骼层级循环，自动断链
- **递归深度保护** — 骨骼变换递归最大 256 层，防止栈溢出
- **Undo 膨胀防护** — 物理运行时自动禁用 `use_global_undo`
- **文件操作保护** — Blender 加载/保存时自动停止物理
- **PMX 计数上限** — 骨骼/刚体/关节/IK 链数量均有合理上限保护
- **编码错误收集** — VMD 解析中的编码错误收集到 warnings 列表，不静默丢弃

## 当前状态与限制

- 支持单模型实时物理预览
- 暂不支持物理烘焙到关键帧
- 暂不支持 SDEF 蒙皮（BDEF1/2/4/QDEF 无渲染相关逻辑）
- 依赖 mmd_tools 导入 PMX 模型，本插件不负责模型导入和渲染
- Windows 平台有预编译 DLL，macOS / Linux 需自行编译

## 项目结构

```
MikuMikuBulletPhysics/
├── __init__.py                 # Blender 插件入口 + Scene Properties
├── blender_manifest.toml       # Blender 4.2+ 扩展清单
├── mmbp_physics.dll             # 预编译 C++ 共享库
├── mmbp/                        # Python 插件包
│   ├── core/
│   │   ├── lib.py              # ctypes DLL 加载 + 函数签名
│   │   ├── state.py            # State + StateManager 状态管理
│   │   ├── bonemap.py          # PMX↔Blender 骨骼映射
│   │   └── loop.py             # Timer 回调物理循环
│   ├── ops/
│   │   ├── load.py             # Load PMX / VMD 操作器
│   │   └── start.py            # Start / Stop 操作器
│   └── bridge/
│       └── ui.py               # 侧边栏 UI 面板
├── src/                        # C++ 源码
│   ├── CMakeLists.txt
│   ├── bullet3/                # Bullet3 子模块
│   └── core/
│       ├── pmx/                # PMX 2.0 解析器
│       ├── vmd/                # VMD 解析器
│       ├── model/              # BoneNode 树 + Morph
│       ├── anim/               # VMD 动画求值（贝塞尔插值）
│       ├── ik/                 # CCD IK 求解器
│       ├── node/               # 骨骼变换树
│       ├── physics/            # Bullet3 物理世界封装
│       ├── math/               # Euler↔Quaternion 转换
│       ├── util/               # 编码 + 日志
│       └── bridge/
│           ├── capi/           # C API (extern "C")
│           ├── engine/         # Engine + PhysicsPipeline
│           └── adapter/        # BlenderAdapter 坐标转换
└── temp/                       # 参考材料（不参与构建）
    ├── reference/              # saba, blender_mmd_tools 参考实现
    └── tests/                  # 测试模型和动作文件
```

## 致谢

- [Bullet Physics](https://github.com/bulletphysics/bullet3) — 刚体物理引擎
- [blender_mmd_tools](https://github.com/MMD-Blender/blender_mmd_tools) — Blender MMD 导入工具，Python 层设计参考
- [saba](https://github.com/benikabocha/saba) — MMD C++ 参考实现，物理渐进混合和 IK 算法参考
