# G2 3box 场景简化与笛卡尔目标交互计划

状态：仅完成调研与实施计划，尚未运行场景简化，尚未修改 GUI/IK/规划器，也未启动新数据规划
计划日期：2026-07-24
目标数据：`/home/ck/MapAnythingTest/G2/3box`
首帧：`snapshot_20260724_040712_0001`

## 1. 结论

可以把现有的 7 关节目标滑杆改成更直观的流程：

1. 在 3D 场景基本体的边缘上点选一个锚点；
2. 生成一个可见的目标点；
3. 用 X/Y/Z 控件连续移动该点；
4. 后台实时求逆运动学并显示可达性；
5. 用户确认后，才运行关节空间 RRT-Connect。

但“一个 XYZ 点”不能唯一确定机械臂末端姿态，因为它只约束 3 个平移自由度，仍缺 3 个旋转
自由度。首版应采用以下明确语义：

- 目标 frame 是活动臂的 `arm_l_end_link` 或 `arm_r_end_link`；
- 目标旋转默认锁定为该 snapshot 起点的法兰旋转；
- XYZ 是米制 `base_link` 坐标；
- 这是 `arm_body_demo`，不是实机夹爪 TCP 目标；
- 当前错误 omnipicker 几何仍被排除，输出继续固定
  `execution_authorized=false`。

这个功能不是一次简单的控件替换。它涉及可靠的 3D picking、目标姿态语义、连续 IK、碰撞预览、
异步任务取消和 GUI 状态管理。不过可以按下面的阶段逐步完成，每阶段都有独立验收点。

## 2. 本轮已核实的数据

`G2/3box` 包含 5 个可用 snapshot：

| Snapshot | 占据体素 | 体素尺寸 | `base_link` 范围 `[min, max]` m |
|---|---:|---:|---|
| `040712_0001` | 43,154 | 0.01 m | `[-0.329,-1.266,-0.298]` → `[1.691,1.334,1.152]` |
| `040725_0002` | 46,130 | 0.01 m | `[-0.373,-1.252,-0.288]` → `[1.697,1.328,1.112]` |
| `040817_0003` | 46,549 | 0.01 m | `[-0.585,-1.240,-0.305]` → `[1.995,1.320,1.105]` |
| `040844_0004` | 48,762 | 0.01 m | `[-0.971,-1.289,-0.138]` → `[1.959,1.311,1.042]` |
| `040911_0005` | 48,357 | 0.01 m | `[-0.956,-1.239,-0.192]` → `[1.974,1.321,1.078]` |

每帧目前都有：

- `voxels.npz/.glb`；
- `scene.glb/.ply` 和 filtered 版本；
- `capture_state.json`；
- 相机 pose、summary 和 preprocess manifest；
- 对应原始 `camera_extrinsics.json` 的绝对来源路径。

每帧目前都没有：

- `cleaned_voxels.npz/.glb`；
- `obstacles.json/.glb`；
- `simplify_report.json`。

因此现有 `Avoid/scripts/avoidance_gui.py` 不能直接加载 `G2/3box`。必须先完成并审计场景简化。

第一帧输入哈希：

- `voxels.npz`：
  `5413af68858285b1a944a7c9a3b07d68c2accf18f757053c860821056960a8a3`
- `capture_state.json`：
  `0c1cc57c35a8afbc0eefd832122f1a5c6c093b7c0259148af4d85954e27952c8`

## 3. 简化器兼容性结论

计划复用 `MapAnythingPipeline/G2_FINDINGS_20260722.md` 第 12 节和
`MapAnythingPipeline/scene_simplify.py`：

- 腕相机中心 0.30 m 空间切除；
- 保守 DBSCAN 去噪；
- Z 直方图确定桌面；
- 桌面输出 support box；
- 桌面上物件按 XY 聚类并拟合 box/cylinder；
- 保留桌子、蓝盒子和低矮主体。

当前存在一个输入 schema 差异：

- `scene_simplify.py --extrinsics` 读取旧版 `camera_extrinsics.json`，需要
  `extrinsics.head_rgb/hand_left_rgb/hand_right_rgb` 和 `fk_base_T_link`；
- `G2/3box/snapshot_*` 本地的 pose 文件使用新版 `poses.head/hand_left/hand_right`；
- `capture_state.json.source` 指向的原始旧版 extrinsics 文件仍存在，5/5 均可读取。

首轮可以通过 `capture_state.json.source` 使用原始文件，不需要猜变换。随后应在 `Avoid` 增加
一个输入解析/编排层，使每个 snapshot 自包含，并按优先级解析：

1. snapshot-local 规范化 extrinsics；
2. `capture_state.json.source` 且文件和 snapshot 身份匹配；
3. 否则拒绝，不从文件名或 URDF 猜相机 pose。

另一个语义问题是简化器当前把 `arm_l/r_end_link` 标成 `left/right_gripper`。后续应改为
`left/right_arm_end` 或明确标注 `flange`；不能继续把它显示成已确认夹爪中心。

## 4. URDF 审阅记录

审阅文件：

`G2/G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf`

SHA-256：

`32daa84c58023cd8fa47f01c0518f53d73ba9b069155005d22e5d0e7d100fafb`

### 4.1 可用于演示的可靠 frame

- `arm_l_end_link`：
  `arm_l_link7` 后固定变换，`xyz=[0.095,0,0]`，
  `rpy=[pi,-pi/2,0]`。
- `arm_r_end_link`：
  与左臂同样是 link7 后固定 95 mm 的机械臂法兰 frame。
- 这两个 frame 属于机械臂本体，可作为当前 Cartesian demo 的跟踪目标。

第一帧起点法兰位置：

- 左：`[0.662320, 0.439478, 1.050716] m`
- 右：`[0.563334, -0.414978, 1.100452] m`

### 4.2 不可作为实机目标的 frame

- `gripper_l_center_link` 和 `gripper_r_center_link`：
  从对应 arm end 沿局部 Z 固定偏移 `0.14308 m`。
- `gripper_l/r_base_link`、`gripper_l/r_camera_link`：
  mesh 路径明确位于 `gripper/omnipicker/`。
- 这些 frame 和碰撞 mesh 描述的是 omnipicker，不是当前安装夹爪。

第一帧按错误 URDF 算出的名义中心位置虽然可以计算，但只能用于来源审计：

- 左：`[0.731092, 0.329456, 0.990405] m`
- 右：`[0.630164, -0.301658, 1.044202] m`

不能把这两个位置作为当前实机 TCP。

### 4.3 相机 frame 也不能替代 TCP

第一帧 preprocess manifest 已记录，标定的手部传感器 frame 与 URDF omnipicker camera link
之间约有：

- 左：`0.1186 m / 98.5°`
- 右：`0.1212 m / 97.1°`

这说明相机 pose 可用于场景重建和空间去夹爪，但不能把 URDF camera link 当成当前夹爪 TCP。

## 5. 目标点交互设计

### 5.1 边缘点选

首版只对规划基本体开放选择，不直接在稠密点云中 picking：

- box：生成 12 条边；
- cylinder：生成上下圆周采样边和竖直母线；
- 将 3D 候选边通过 Matplotlib 当前投影视图映射到屏幕；
- 计算鼠标位置到每条投影线段的最近点；
- 在像素阈值内选择最近线段，并由线段参数反算精确 3D 坐标；
- 保存 primitive id、边 id、线段参数和 `base_link` 坐标。

这种方法比从 Matplotlib 鼠标事件直接“反投影到任意 3D 深度”更稳定，因为普通 2D 点击本身
没有唯一深度。

选中后显示：

- 高对比目标球；
- X/Y/Z 三条短轴；
- 被选 primitive 和边高亮；
- 原始锚点与当前拖动目标之间的细线。

### 5.2 XYZ 编辑

首个可用版本不实现完全自由的 3D 鼠标拖拽，而提供稳定、可精确复现的：

- X/Y/Z 三个滑杆；
- 对应数值输入框；
- 1 mm / 1 cm stepper；
- “恢复锚点”和“恢复起点法兰”；
- 可选的单轴拖动模式。

第二阶段再实现三轴 gizmo。自由屏幕拖动必须绑定某个轴或拖动平面，否则同样存在深度歧义。

### 5.3 边缘点不是直接运动终点

障碍表面的边缘点本身处于碰撞边界，不能直接作为法兰目标。选择锚点后应生成：

`goal_position = anchor_position + outward_normal * approach_offset`

规则：

- box 的法向根据所选边邻接面和相机视角选择，并允许用户翻转；
- cylinder 使用径向法向；
- 默认 offset 在 GUI 中明确显示；
- offset 不能被混同为真实夹爪长度；
- 没有确认夹爪模型前，任何 offset 都只服务于 arm-body 算法演示。

如果边属于两个面的交线，默认法向使用两个外法向的归一化和；用户可以切换到任一邻接面法向。

### 5.4 旋转策略

首版提供一个只读模式：

`保持 snapshot 起点法兰旋转`

由起点 `base_T_arm_end` 的 3×3 rotation 和编辑后的 XYZ 合成完整 4×4 IK 目标。

之后可增加：

- 保持上一可行解旋转；
- 法兰某轴对准 approach normal；
- roll 单自由度控制；
- 完整旋转 gizmo。

在真实 TCP 和夹爪外形确认前，不应增加“夹爪朝向物体”“抓取姿态”等措辞。

## 6. Cartesian 目标数据契约

建议新增可保存、可复现的 JSON：

```json
{
  "schema_version": 1,
  "robot_profile": "g2",
  "world_frame": "base_link",
  "translation_unit": "meter",
  "planning_scope": "arm_body_demo_excludes_unconfirmed_end_effector",
  "active_arm": "left",
  "tracked_frame": "arm_l_end_link",
  "position_m": [0.70, 0.20, 0.90],
  "orientation_policy": "hold_snapshot_start_flange",
  "base_R_goal": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  "anchor": {
    "primitive_id": 2,
    "feature": "box_edge",
    "edge_id": 7,
    "position_m": [0.70, 0.12, 0.80],
    "outward_normal": [0, 1, 0],
    "approach_offset_m": 0.08
  },
  "execution_authorized": false
}
```

保存时绑定：

- `obstacles.json` SHA-256；
- `capture_state.json` SHA-256；
- URDF 和 collision mesh bundle SHA-256；
- end-effector contract SHA-256；
- GUI/规划参数。

## 7. IK 与 worker 改造

当前状态：

- `avoidance/ik_solver.py` 已能求完整 6D pose；
- `planner.py` 已接受 `goal_pose`；
- GUI worker 只接受 `goal_arm`；
- GUI 只显示 7 关节目标滑杆。

计划增加两个 worker action：

### `preview_cartesian_goal`

输入：scene、capture、side、position、orientation policy、request id。
输出：

- 完整 `base_T_goal`；
- IK 成功/失败；
- 位置和旋转误差；
- 关节目标；
- 目标姿态碰撞报告；
- 法兰位置和整机骨架预览；
- failure reason。

拖动时采用 150–250 ms debounce。每个请求带单调递增 request id，GUI 丢弃过期响应，避免快速
拖动后旧 IK 结果覆盖新目标。

IK seed 顺序：

1. 上一个相邻可行预览解；
2. 当前 snapshot 起点；
3. 确定性随机 seed。

目标函数增加对上一个解的弱正则，减少拖动时肘部姿态突然翻转。

### `plan_cartesian_goal`

只在最近一次 preview 仍然有效且目标哈希一致时运行。worker 内部重新执行 IK 和目标碰撞检查，
不能信任 GUI 缓存，然后调用现有 RRT-Connect。

## 8. GUI 状态与反馈

目标点颜色：

- 灰：尚未求解；
- 绿：IK 成功且目标无碰撞；
- 橙：IK 成功但目标碰撞或安全间距不足；
- 红：不可达、超限或输入无效。

必须显示：

- 当前 XYZ；
- tracked frame；
- orientation policy；
- approach offset；
- IK 误差；
- 目标最小环境净距；
- 当前是否允许点击“开始规划”；
- 固定的“实机执行禁止”。

规划按钮只有在以下条件同时满足时启用：

- scene/capture 哈希未变化；
- 最新 request id 已返回；
- IK 成功；
- 目标碰撞检查通过；
- 目标未在 preview 后继续移动。

原 7 关节滑杆保留在“高级/关节目标”tab，用于回归与诊断，不再作为默认操作方式。

## 9. `G2/3box` 场景简化实施步骤

### 阶段 A：输入预检

对 5 帧逐一检查：

- `base_link/meter`；
- 0.01 m voxel；
- `capture_state` 22 个规划关节完整；
- source extrinsics 存在且 snapshot id 一致；
- 相机矩阵刚体有效；
- 输出目录中不存在需要人工保留的同名派生文件。

### 阶段 B：按第 12 节参数简化

第一帧计划命令，当前先不执行：

```bash
cd /home/ck/MapAnythingTest
conda run -n MAP python MapAnythingPipeline/scene_simplify.py \
  G2/3box/snapshot_20260724_040712_0001/voxels.npz \
  --extrinsics \
  G2/session_20260724_040334/snapshot_20260724_040712_0001/camera_extrinsics.json \
  --out-dir G2/3box/snapshot_20260724_040712_0001 \
  --gripper-radius 0.30 \
  --denoise-min-cluster 4 \
  --min-cluster 24 \
  --cluster-eps 0.03 \
  --obstacle-height 0.03 \
  --table-thickness 0.06 \
  --primitive-mode auto \
  --cylinder-max-diameter 0.20 \
  --max-footprint-aspect 3.0 \
  --box-inflate 0.08
```

确认第一帧后，再用完全相同参数处理其余 4 帧。不得为了得到“正好三个盒子”而静默逐帧调参。

### 阶段 C：简化结果验收

每帧必须人工和机器双重审核：

- 桌面保留为一个 support；
- 三个主要箱体/物件主体没有消失；
- 相邻箱体是否被错误合并；
- 腕部/夹爪体素是否残留成障碍；
- 没有巨大墙体或地面被误拟合为工作台物件；
- box/cylinder 尺寸和颜色合理；
- 五帧主体中心和尺寸漂移有统计报告；
- `obstacles.json` 与 `obstacles.glb` 一致；
- marker 使用 flange 语义，不冒充已确认 TCP。

规划使用 JSON 中未膨胀的 primitive，并在 `Avoid` 中独立应用 8 cm 环境 inflation 和 2 cm
clearance。`box_inflation_m=0.08` 只用于简化 GLB 可视化，不能在碰撞查询中重复计算。

审计命令，当前先不执行：

```bash
conda run -n MAP env PYTHONPATH=Avoid \
  python Avoid/scripts/audit_planning_scene.py \
  --input G2/3box \
  --planning-inflation-m 0.08 \
  --out Avoid/reports/g2_3box_planning_audit.json
```

## 10. 第一帧启动顺序

Cartesian 交互完成并通过测试后，以第一帧启动：

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP \
  env PYTHONPATH=Avoid MPLCONFIGDIR=/tmp/mapanything-matplotlib \
  python Avoid/scripts/avoidance_gui.py \
  --scene-root G2/3box
```

GUI 按名称排序，默认选择
`snapshot_20260724_040712_0001`。启动后仍只加载与检查起点，不自动发送规划请求；用户选点、
调整 XYZ 并得到绿色 preview 后，才可点击规划。

## 11. 代码实施分阶段

### M1：3box 简化编排与契约

目标文件：

- 新增 `Avoid/scripts/simplify_g2_snapshots.py`；
- 扩展 `Avoid/avoidance/planning_scene.py` 的 marker 语义；
- 增加 3box 输入/输出测试。

验收：5 帧可重复生成并审计，三主体不消失，输入来源和哈希完整。

### M2：Cartesian goal contract 与 position IK

目标文件：

- 新增 `Avoid/avoidance/cartesian_goal.py`；
- 扩展 `Avoid/avoidance/ik_solver.py`；
- 扩展 `Avoid/avoidance/planner.py`。

验收：固定旋转 XYZ 目标可求解；不可达、超限和碰撞目标均有明确 reason；相邻目标解连续。

### M3：worker preview 协议

目标文件：

- 扩展 `Avoid/scripts/g2_gui_worker.py`；
- 加 request id、preview 和 Cartesian plan action；
- 增加 worker 集成测试。

验收：过期响应不会覆盖新目标；GUI 线程不阻塞；规划前 worker 强制重验。

### M4：GUI picking 与 XYZ 控件

目标文件：

- 扩展 `Avoid/scripts/avoidance_gui.py`；
- 默认切到 Cartesian target tab；
- 关节目标移到高级 tab。

验收：box/cylinder edge 可选择；XYZ 可按 1 mm/1 cm 编辑；目标颜色与 IK/碰撞状态一致。

### M5：第一帧端到端规划

输入：

- 第一帧 `obstacles.json`；
- 同帧 `capture_state.json`；
- 用户选定 Cartesian 法兰目标。

验收：目标 JSON、plan manifest、路径截图和诊断报告全部哈希绑定；路径稠密复检通过；结果仍
不可执行。

## 12. 测试矩阵

单元测试：

- box 12 边生成；
- cylinder 边采样；
- 3D 投影线段最近点与世界坐标反算；
- 像素阈值外拒绝；
- anchor normal/offset；
- Cartesian JSON schema 和哈希；
- 固定旋转 pose 合成；
- position IK 容差、限位和 seed 连续性；
- stale request id 丢弃。

集成测试：

- 第一帧左/右法兰当前位姿 round-trip；
- 起点附近 1–3 cm XYZ 目标；
- 桌面内部目标必须碰撞拒绝；
- 膨胀 box 内目标必须拒绝；
- 超出工作区目标必须不可达；
- “法兰路径自由但肘部碰撞”必须被完整机器人检查拒绝；
- 快速连续拖动 20 次后只显示最后请求结果；
- GUI resize/高 DPI/滚动栏回归。

场景回归：

- 5 帧都保留桌面和三个主要物件；
- 5 帧第一起点审计；
- 相同目标在不同 snapshot 下明确显示各自可达/碰撞结论，不复用旧缓存。

## 13. 安全边界与开放条件

当前阶段可以验证：

- 场景简化是否保留主要障碍；
- 点选和 XYZ 目标是否符合直觉；
- 法兰位置 IK 和关节空间绕障是否工作；
- 机械臂本体、头部和另一只手臂是否避障。

当前阶段不能验证：

- 真实夹爪 TCP 是否到达目标点；
- 真实夹爪是否与箱体、桌面或物件碰撞；
- 抓取方向、开合空间和接触；
- 动态障碍、轨迹时间化或实机跟踪。

从“法兰点演示”升级为“实机夹爪终点”之前，必须提供：

1. 当前安装夹爪型号和保守碰撞几何；
2. 左右 `arm_end_T_tcp` 实测变换；
3. 更新后的 URDF/mesh/config 哈希；
4. 完整机器人起点与目标重审计；
5. 轨迹时间参数化、实时状态和执行门禁。

## 14. 本轮明确未执行

- 未修改 `MapAnythingPipeline/scene_simplify.py`；
- 未在 `G2/3box` 写入任何简化产物；
- 未修改 Cartesian GUI、worker、IK 或 planner；
- 未启动新数据 GUI；
- 未运行第一帧路径规划；
- 未发送任何机器人命令。

## 15. 实施状态（2026-07-24 更新，本机 numpy/scipy/sklearn/trimesh，无 pinocchio/显示）

已按本计划落地并**在本机可测的部分全部通过测试**（pinocchio/显示相关只能语法+模式核对）：

| 里程碑 | 交付 | 状态 |
|---|---|---|
| M1 简化编排 | `scripts/simplify_g2_snapshots.py`：复用 `scene_simplify.py`，加 support/object 分类 + Avoid schema，fail-closed 过 `load_planning_scene` | **已跑** `interaction/` 两帧 → 0001=1 support+3 object，0002=1+7；obstacles.json 已写 |
| M2 契约+几何 | `avoidance/cartesian_goal.py`：12 边、投影线段最近点、anchor+法向+offset、旋转策略、`CartesianGoal` schema+哈希，`execution_authorized` 恒 False | **13 单测通过** |
| M2 planner | `avoidance/planner.py`：放开 arm-body-demo 的笛卡尔目标（**仅跟踪法兰**，非 TCP），goal 记录标为 `flange_pose` | 代码就位，需 pinocchio 跑 |
| M3 worker | `scripts/g2_gui_worker.py`：新增 `preview_cartesian_goal`（快速 IK+碰撞，返回 request_id/target_status/skeleton）与 `plan_cartesian_goal`；worker 内**自算旋转**（不信 GUI 缓存） | 代码就位，需 pinocchio 跑 |
| M3/M4 状态机 | `avoidance/cartesian_target_controller.py`：点选→目标、XYZ 微调、翻转法向、offset、request_id 去抖丢弃过期、warm-start seed、`can_plan` 门禁、matplotlib 投影桥 | **10 单测通过**（含无头投影测试） |
| M4 GUI | `scripts/avoidance_gui.py`：笛卡尔面板（点选提示、XYZ ±1mm/±1cm、offset、翻转、target 状态色、笛卡尔规划按钮）、画布点选、200ms 去抖预览、target 叠加渲染、poll 路由；原 7 关节滑杆保留为“高级/回归” | 语法通过，需**显示+worker(pinocchio)** 跑 |

其余未做：cylinder 拟合与其边采样（首版 box-only）、goal JSON 落盘按钮、5 帧统计漂移报告、§12 集成测试中需 pinocchio 的那部分。

## 16. 在有环境的机器上的接续步骤

本机（numpy/scipy/sklearn/trimesh，无 pinocchio、无显示）无法运行 IK/碰撞/GUI，
以下几步必须在装好机器人后端与图形环境的机器上完成，才能让这套脚本按预期跑通：

1. **两个 conda 环境（GUI 与 worker 分离，见 `scripts/avoidance_gui.py` 的 `RobotWorker`）**：
   - GUI 进程：matplotlib/tkinter（§10 用 `-n MAP`）。
   - worker 子进程固定用 `conda run -n robot`，该 `robot` 环境**必须能 `import pinocchio, hppfcl`**（IK 与碰撞全靠它）。
     `planner.py`/`g2_gui_worker.py`/GUI 因此在无 pinocchio 机器上**未经运行验证，仅语法+模式核对**，需在此环境实测。

2. **装 pytest 跑全套**：`PYTHONPATH=. python3 -m pytest tests/ -q`。
   有 pinocchio 时，`test_g2_planning` 里 3 个 `skipped`（"robot backend required"）与 planner 集成路径才会真正执行——
   这是验证 worker 新增 `preview_cartesian_goal` / `plan_cartesian_goal` 与 IK 的关键一步。
   （无 pinocchio 机器上 `test_g2_planning::test_capture` 会因引用不存在的 `G2/expoutput3/.../capture_state.json` 失败，与本次改动无关。）

3. **生成全部 5 帧 3box 的 `obstacles.json`**（本机只处理了 `interaction/` 的 2 帧）：
   ```
   PYTHONPATH=. python3 scripts/simplify_g2_snapshots.py --scene-root <G2/3box> \
     --pipeline <MapAnythingPipeline 路径>     # 或设环境变量 MAPANYTHING_PIPELINE
   ```
   需每帧 `<snap>_input/camera_extrinsics.json`（旧格式）。跑完按 §9 阶段 C 人工审核：
   桌面为一个 support、三主体不消失/不误并、无夹爪残留。注意本地 0001 的 object `id=1` 是 0.4×0.51 m 大盒，
   **可能并了相邻箱体**，需确认；必要时调 `--cluster-eps` / `--min-cluster`。

4. **启动 GUI 端到端验证交互**（本机测不了，须实机确认）：
   ```
   PYTHONPATH=. python3 scripts/avoidance_gui.py --scene-root <G2/3box>
   ```
   - 点选一条**物体边** → 生成目标球；
   - XYZ ±1mm/1cm、offset、翻转法向 → 目标球实时变色（灰=未解 / 绿=可达无碰 / 橙=可达但碰撞 / 红=不可达）；
   - 仅当绿色时"笛卡尔目标规划"按钮亮 → 点击跑 RRT-Connect；
   - 重点核对三处本机未跑的逻辑：**画布点选的像素投影是否对准边**、**worker preview 的 IK/碰撞是否符合预期**、**快速拖动去抖是否只保留最后结果**。

5. **每帧目录须有 `capture_state.json`**（worker 的 `load_g2_capture_state` 依赖；`interaction/` 两帧已具备）。

6. **升级到实机夹爪终点前的红线不变**（§13）：当前仅"法兰点演示"，`execution_authorized` 恒 False；
   要跟踪真实夹爪 TCP，仍需实机夹爪保守碰撞几何 + 实测 `arm_end_T_tcp` + 重新绑定 URDF/mesh/config 哈希。
