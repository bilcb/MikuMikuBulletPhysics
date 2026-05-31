# MikuMikuBulletPhysics

在 Blender 中运行 MMD（MikuMikuDance）物理模拟的插件。通过加载原生 C++ 共享库（Bullet3 物理引擎），为 mmd_tools 导入的 PMX 模型提供与 MMD 一致的刚体物理模拟。

当前为早期开发版本，实现了基本的单模型实时物理预览。

- 项目地址：<https://github.com/bilcb/MikuMikuBulletPhysics>
- 许可证：MIT License

## 功能

- **实时物理模拟** — 基于 Bullet3，以 120Hz 固定步长运行
- **PMX/VMD 解析** — 原生 C++ 解析 PMX 模型（骨骼、刚体、关节）和 VMD 动画（骨骼关键帧）
- **坐标自动转换** — MMD Y-up 与 Blender Z-up 之间的转换
- **Blender 集成** — 侧边栏面板 (View3D → Sidebar → MMP)，一键启停
- **IK 求解** — CCD IK 求解器，支持单轴和多轴约束
- **VMD 贝塞尔插值** — 按 VMD 曲线插值骨骼动画

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

仓库根目录已包含预编译的 `mmp_physics.dll`（Windows x64），可直接使用。如需自行编译：

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

产出物 `mmp_physics.dll`（Windows）/ `.so`（Linux）/ `.dylib`（macOS）复制到项目根目录（与 `__init__.py` 同级）。

## 使用

### 快速开始

1. 使用 mmd_tools 导入 PMX 模型
2. 在 3D View 侧边栏打开 **MMP** 标签页
3. 选择模型根对象，点击 **Load PMX** 加载同一 PMX 文件
4. （可选）点击 **Load VMD** 加载动画
5. 点击 **Start** 开始实时物理模拟
6. 点击 **Stop** 停止

### 说明

- 开始模拟后会临时禁用 Blender 内置刚体世界，并静音 mmd_tools 的物理跟随约束
- 模拟停止后自动恢复
- 物理运行时会自动关闭 `use_global_undo`，防止撤销历史膨胀

### 物理参数

当前版本参数为硬编码默认值，暂无可视化调节面板：

| 参数 | 值 | 说明 |
|------|----|------|
| 重力 | `(0, -9.8, 0)` | MMD 惯例（单位 cm） |
| 求解器迭代 | 20 | 约束求解精度 |
| 固定步长 | 1/120 s | 与 MMD 物理惯例一致 |
| 最大子步数 | 10 | 防止帧率过低时物理失控 |
| 刚体缩放 | 0.08 | PMX cm → Blender m |

### 坐标系

- PMX/VMD 文件：Y-up，右手系
- Blender 场景：Z-up，右手系
- DLL 内部：Y-up（与 MMD 一致）
- 输出时通过每根骨骼的 rest matrix 转换为 Blender 骨骼局部空间

## 架构概览

```
Blender Python 层 ──ctypes→ C++ 共享库 (mmp_physics.dll)
                               ├── PMX 解析器
                               ├── VMD 解析器
                               ├── 骨骼模型树
                               ├── VMD 动画求值（贝塞尔插值）
                               ├── CCD IK 求解器
                               ├── Bullet3 物理世界
                               └── BlenderAdapter 坐标转换
```

Python 侧负责 UI、DLL 加载、120Hz 定时器循环、以及将物理结果写回 Blender 骨骼。

## 当前状态与限制

- 支持单模型实时物理预览，暂不支持多模型、烘焙、时间线模式
- 物理参数（重力、迭代次数等）暂不可从 UI 调节
- 依赖 mmd_tools 导入 PMX 模型，本插件不负责模型导入
- Windows 平台有预编译 DLL，macOS / Linux 需自行编译
- Blender Python 主线程开销限制，手感无法完全等同 MMD 本体

## 项目结构

```
MikuMikuBulletPhysics/
├── __init__.py                 # Blender 插件入口
├── blender_manifest.toml       # Blender 4.2+ 扩展清单
├── mmp_physics.dll             # 预编译 C++ 共享库
├── mmp/                        # Python 插件包
│   ├── core/
│   │   ├── dll.py              # ctypes DLL 加载
│   │   ├── state.py            # 状态管理 & 物理调度
│   │   ├── loop.py             # 120Hz 定时器循环
│   │   └── coord.py            # 坐标转换工具
│   ├── ops/
│   │   ├── load.py             # Load PMX / VMD 操作器
│   │   └── start.py            # Start / Stop 操作器
│   └── bridge/
│       └── ui.py               # 侧边栏 UI 面板
├── src/                        # C++ 源码
│   ├── CMakeLists.txt
│   ├── bullet3/                # Bullet3 子模块
│   └── core/
│       ├── pmx/                # PMX 解析器
│       ├── vmd/                # VMD 解析器
│       ├── model/              # 骨骼模型数据结构
│       ├── anim/               # VMD 动画求值
│       ├── ik/                 # CCD IK 求解器
│       ├── node/               # 骨骼变换树
│       ├── physics/            # Bullet3 物理世界封装
│       ├── math/               # 数学工具（Euler↔Quaternion）
│       ├── util/               # 编码工具（UTF-16 / Shift-JIS）
│       └── bridge/             # Blender 桥接层
│           ├── capi/           # C API (extern "C")
│           ├── engine/         # Engine + PhysicsPipeline
│           └── adapter/        # BlenderAdapter 坐标转换
└── temp/                       # 参考材料（不参与构建）
    ├── reference/              # saba, blender_mmd_tools 子模块
    └── tests/                  # 测试模型和动作文件
```

## 安全特性

- **C API 异常安全** — 所有 `extern "C"` 函数用 try-catch 包裹
- **NaN/Inf 防护** — 输出到 Blender 前检查 `isfinite()`
- **参数校验** — 所有 C API 入口检查空指针
- **Undo 膨胀防护** — 物理运行时自动禁用 `use_global_undo`
- **文件操作保护** — Blender 加载/保存时自动停止物理

## 致谢

- [Bullet Physics](https://github.com/bulletphysics/bullet3) — 刚体物理引擎
- [blender_mmd_tools](https://github.com/MMD-Blender/blender_mmd_tools) — Blender MMD 导入工具
- [saba](https://github.com/benikabocha/saba) — MMD C++ 参考实现
