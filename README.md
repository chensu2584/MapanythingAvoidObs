# G1/G2 MapAnything 避障与路径规划准备

G1 已实现早期实验的前两阶段：夹爪 TCP 位姿/标定报告，以及从 MapAnything 占据体素生成
操作地图。G2 已实现聚类场景输入、机械臂规划核心、多初值 IK、双向 RRT-Connect 和稠密路径
复核，并新增可交互 GUI。参数包 URDF 的 omnipicker 末端与实机夹爪不一致，因此完整 G2
碰撞审计和规划仍强制闭锁，直到补充实机夹爪碰撞模型与 TCP；GUI 仅开放明确隔离的机械臂
本体算法演示，没有轨迹时间参数化或运动执行代码。

当前恢复点：`142521` 已建立全场景人工审核 schema v2 空白草稿，`39,552` 个源占据体素均可
选择；当前 `selected=0`、`review_complete=false`，等待操作者进入 GUI 标绿。自动夹爪边界判断
明确延后，不能把紫色/黄色提示当作已批准删除。完整实现与验证记录见
[`WORK_LOG.md`](WORK_LOG.md)。

## G2 数据采集与直接深度重建

G2 采集、直接深度体素化和 GLB 机器人标记代码统一由 Avoid 仓库维护，采集数据仍写入工作区
兄弟目录 `G2`：

- `scripts/g2_capture_gui.py`：头部/左右手相机预览与 snapshot GUI；
- `scripts/g2_capture_session.py`：G2 采集 CLI、外参与 FK 校验后端；
- `scripts/reconstruct_depth_voxels.py`：原始或注册深度反投影与体素 GLB；
- `scripts/g2_glb_markers.py`：共享相机、法兰、简约左右手与 `base_link` 原点标记。

采集 GUI：

```bash
cd /home/ck/MapAnythingTest
python Avoid/scripts/g2_capture_gui.py
```

直接深度体素重建：

```bash
conda run --no-capture-output -n MAP \
  python Avoid/scripts/reconstruct_depth_voxels.py \
  --input G2/3box/undistorted
```

完整输入格式、输出、桌面裁剪、直接深度场景简化与逐帧统计见
[`DEPTH_VOXEL_RECONSTRUCTION.md`](DEPTH_VOXEL_RECONSTRUCTION.md)。

## G2 聚类场景：规划输入准备

`G2/expoutput3/snapshot_*/obstacles.json` 已能作为保守环境几何层使用。规划侧入口位于
`avoidance/planning_scene.py`，支持严格校验 `box/cylinder`、独立规划膨胀和解析有符号距离；
`markers` 仅作位姿参考，不参与碰撞。其中左右 `gripper_center` 来自不匹配的 omnipicker URDF，
不能当作实机夹爪 TCP；头部和手部相机 marker 不受这项末端中心语义影响。

批量审计：

```bash
cd /home/ck/MapAnythingTest
conda run -n MAP env PYTHONPATH=Avoid \
  python Avoid/scripts/audit_planning_scene.py \
  --input G2/expoutput3 \
  --planning-inflation-m 0.08 \
  --out Avoid/reports/g2_expoutput3_planning_audit.json
```

当前 6 帧环境几何契约通过，但实机末端工具未确认，因此 6/6 起点碰撞审计均为 `not_run`。
先前基于 omnipicker 得出的“4/6 可规划”和两帧夹爪碰撞结论已经撤回。详细结论见
 [`G2_CLUSTER_PLANNING_READINESS.md`](G2_CLUSTER_PLANNING_READINESS.md)。

## G2 避障 GUI

GUI 已实现，默认读取 `G2/expoutput3` 的 6 个 snapshot。它在 `MAP` 环境显示聚类桌面、蓝盒子、
其他盒/圆柱、相机与原点、G2 机身/头部/双臂、10 cm 规划安全边界和活动臂法兰路径；7 个目标
关节可用滑杆调整，并已增加物体边点选、XYZ/approach offset 微调、Cartesian IK/碰撞 preview
和法兰目标规划。规划后可拖动时间轴或播放路径。右侧诊断显示起点状态、RRT 结果、碰撞查询
次数和最小环境净距。

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP \
  env PYTHONPATH=Avoid MPLCONFIGDIR=/tmp/mapanything-matplotlib \
  python Avoid/scripts/avoidance_gui.py
```

GUI 默认按屏幕 92%×88% 居中打开，并在高 DPI 桌面额外使用 `1.25` 的控件缩放。需要调整时可
附加 `--ui-scale 1.0`（允许范围 `0.8`–`2.0`）；左侧控制栏固定最小宽度并可垂直滚动。

GUI 通过一次性 JSON worker 调用 `robot` 环境的 Pinocchio/HPP-FCL/RRT，界面本身不需要混装
机器人依赖。红色顶部栏固定声明当前是 `arm_body_demo`：只排除名称明确为 `gripper_*` 的 4 个
错误 omnipicker 碰撞几何，使用 `arm_l/r_end_link` 法兰而不是未知 TCP。
机身、头部、左右臂和另一条固定手臂仍参与碰撞。此模式只能观察场景重建能否驱动绕障搜索，
所有结果固定 `execution_authorized=false`，不能用于实机。

`G2/3box` 第三帧已验证点选和 preview 链路，但简化 primitive 与左臂起点接触；当前 planner
仍会以 `start_configuration_in_collision` 拒绝。下一步采用受约束脱离前缀，再从首个正常
无碰状态运行 RRT；不会全局忽略该障碍。关节可行性、未知夹爪处理、聚类精度门禁和恢复方案
详见 [`G2_DANGER_START_AND_VALIDATION_PLAN.md`](G2_DANGER_START_AND_VALIDATION_PLAN.md)。

无显示环境可验证完整 GUI 数据与绘图链路：

```bash
conda run -n MAP env PYTHONPATH=Avoid MPLCONFIGDIR=/tmp/mapanything-matplotlib \
  python Avoid/scripts/avoidance_gui.py --smoke-test
```

输出为 [`reports/avoidance_gui_smoke.json`](reports/avoidance_gui_smoke.json) 与
[`reports/avoidance_gui_smoke.png`](reports/avoidance_gui_smoke.png)。

## G2 阶段三：离线单臂规划

规划核心在 `robot` 环境运行。输入是同一 snapshot 的 `obstacles.json`、`capture_state.json`
以及目标关节或目标夹爪中心位姿。当前默认
[`configs/g2_end_effector_model.json`](configs/g2_end_effector_model.json) 明确记录
`urdf_matches_installed=false`，所以命令只输出 `status=blocked` 清单，不调用碰撞、IK 或 RRT。

目标关节 JSON 可用紧凑数组：

```json
{
  "arm_joint_positions_rad": [0, 0, 0, -1.2, 0, -0.5, 0]
}
```

门禁验证命令：

```bash
cd /home/ck/MapAnythingTest
conda run -n robot env PYTHONPATH=Avoid \
  python Avoid/scripts/plan_g2_avoidance.py \
  --scene G2/expoutput3/snapshot_20260723_034729_0001 \
  --capture-state G2/expoutput3/snapshot_20260723_034729_0001/capture_state.json \
  --arm left \
  --goal-joints Avoid/examples/g2_left_goal_034729_smoke.json \
  --out Avoid/reports/g2_plan.json
```

当前命令应以退出码 2 和 `reason=installed_end_effector_model_unconfirmed` 结束。要开放离线规划，
必须先提供左右实机夹爪的型号或尺寸、相对 `arm_l/r_end_link` 的保守碰撞几何，以及实测 TCP
变换；随后更新机器人 URDF/mesh 和末端配置的哈希、确认字段与 TCP frame。仅把
`confirmed` 改成 `true` 不会通过其余一致性检查。

模型确认后，目标位姿可用 `--goal-pose` 提供米制 `base_T_goal` 4x4 矩阵；成功输出才会包含
稠密关节路点、`base_T_tcp` 路径和最小环境间距。无论成功或失败，`execution_authorized`
始终为 `false`。
当前搜索把未被基本体占据的区域仅作为离线 `assumed_free`，并会在 manifest 中记录
`validated_workspace_bounds_available=false`；这正是禁止执行的硬阻断项之一。

命令行也可显式附加 `--arm-body-demo` 来复现 GUI 的关节目标算法演示。该开关拒绝
`--goal-pose`、排除错误 `gripper_*` 几何并输出 `status=demo_planned`；它不是绕过完整规划门禁
的方式，输出同样禁止执行。

批量检查六个已有起点：

```bash
conda run -n robot env PYTHONPATH=Avoid \
  python Avoid/scripts/audit_g2_planning_starts.py \
  --input G2/expoutput3 \
  --out Avoid/reports/g2_expoutput3_start_collision_audit.json
```

## 环境与坐标约定

- 地图处理使用 `conda` 的 `MAP` 环境。
- 实时机器人反馈使用完成 `robot_test/env.sh` 初始化的 `robot` 环境。
- 所有计算文件使用米制 `base_link`；这里导出的 GLB 不再做旧 `voxels.glb` 的绕 X 轴 180°
  查看器翻转。
- GLB 是可视化文件，NPZ 才是完整占据地图。单独 GLB 不具有足够的体素/坐标元数据；当传入
  `voxels.glb` 时，程序实际安全地读取同目录 `voxels.npz` 并在 manifest 中记录这一点。

## 阶段一：夹爪/TCP 位姿

离线复算最新 capture：

```bash
conda run -n MAP python Avoid/scripts/report_gripper_pose.py \
  --capture-dir outputs/g1_capture_20260721_115621
```

实时只读反馈（先在机器人 shell 中完成 SDK/Aorta 环境初始化）：

```bash
conda run --no-capture-output -n robot python Avoid/scripts/report_gripper_pose.py --live
```

默认 TCP 不是完成的实测标定，而是：

```text
hand_T_tcp = URDF reference ([0, 0, 0.14308] m) @ measured correction
```

在 `configs/tcp_calibration.json` 中填入你的实测修正后，只有人工核对并显式设置
`confirmed: true` 才能通过该项执行门禁。也可把 mode 改为 `absolute_measured_hand_T_tcp` 并提供
绝对实测位姿；两种模式不能混用。当前阶段永远不发送运动指令。

## 阶段二：操作地图

先生成不删除自机候选的审核版本：

```bash
conda run -n MAP python Avoid/scripts/build_operation_map.py \
  --input outputs/g1_capture_20260721_115621
```

输出在 reconstruction 的 `avoidance/` 下：

- `self_filter_candidates_base_link.glb`：紫红色为拟删除 core，黄色为始终保留的歧义 shell；
- `self_filter_report.json`：每个 link 的命中数、TCP 到地图距离和门禁状态；
- `operation_map.npz`：完整稀疏占据，`cell_kind=1` 是保留源体素，`2` 是膨胀体素；
- `operation_map_base_link.glb`：操作地图表面和左右 TCP 标记；
- `operation_map_manifest.json`：输入哈希、参数、量化、坐标和输出哈希。

审核预览确认紫红色区域确为机器人自身、没有把桌面或真实障碍包含进去后，才运行：

```bash
conda run -n MAP python Avoid/scripts/build_operation_map.py \
  --input outputs/g1_capture_20260721_115621 \
  --approve-self-filter
```

### 当前实验：142521 全场景人工夹爪标定

暂时不依赖自动 core/shell 对夹爪的判断。下面的模式允许从**全部已占据源体素**中人工选择：
紫色和黄色只作为位置提示，灰色同样可以选择；只有人工选成绿色的体素会被后续地图删除。

首次进入或继续增加选择：

```bash
conda run --no-capture-output -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_142521 \
  --selection-scope all-occupied \
  --mode add \
  --brush-radius-m 0.02
```

按住 Shift 加左键选择种子点，关闭窗口后才会应用 2 cm 笔刷并询问是否保存。紫色、黄色、
灰色都能作为种子。误选时重新运行上面的命令，把 `--mode add` 改为 `--mode remove`，然后
点击绿色区域。每次保存都会更新：

- `avoidance/manual_self_filter_review.json`：精确体素索引、输入哈希和完成状态；
- `avoidance/manual_self_filter_review_base_link.glb`：绿色为人工选择，其他颜色仅作上下文提示。

人工核对完毕后把测试 review 标记完成：

```bash
conda run -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_142521 \
  --selection-scope all-occupied \
  --no-gui --mark-complete --yes
```

再生成只移除绿色体素的测试操作地图（不要附加 `--approve-self-filter`，否则命令会拒绝）：

```bash
conda run -n MAP python Avoid/scripts/build_operation_map.py \
  --input outputs/g1_capture_20260721_142521 \
  --self-filter-review \
  outputs/g1_capture_20260721_142521/avoidance/manual_self_filter_review.json
```

该全场景 review 使用 schema v2，与旧黄色审核文件分开保存。未选择体素一律保留为场景；
自动识别夹爪边界的问题留待后续处理。

### 旧版黄色候选审核（兼容保留）

默认情况下黄色歧义点即使批准 core 也保留。黄色中混有机器人和真实场景时，可继续使用旧版
候选审核 GUI；不要整体批准黄色：

```bash
conda run --no-capture-output -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_131012 \
  --approve-core \
  --mode add \
  --brush-radius-m 0.02
```

窗口颜色：紫色是 core，黄色是尚未选中的歧义体素，绿色是已选为机器人的黄色体素，灰色是
场景上下文。按住 Shift 加左键点选黄色种子；关闭窗口后，程序按 `--brush-radius-m` 扩展笔刷。
误选时用 `--mode remove` 再点绿色区域。每次保存都会同时更新
`avoidance/self_filter_review_base_link.glb`，重新打开 GUI 或该 GLB 可检查绿色结果。

确认全部选择后，把 review 标为完成：

```bash
conda run -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_131012 \
  --no-gui --approve-core --mark-complete --yes
```

`--mark-complete` 在 `--no-gui` 模式下本身也会触发非交互保存；这里保留 `--yes` 让命令意图
更直观，并兼容旧版本脚本。

完成后只查看、不修改 review：

```bash
conda run --no-capture-output -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_131012 --view-only
```

然后用完成的、哈希绑定的 review 构建地图：

```bash
conda run -n MAP python Avoid/scripts/build_operation_map.py \
  --input outputs/g1_capture_20260721_131012 \
  --self-filter-review \
  outputs/g1_capture_20260721_131012/avoidance/self_filter_review.json
```

review 精确保存被选中的源体素索引，并绑定 `voxels.npz`、URDF、capture state 和 mask 参数的
哈希；任何输入或参数变化都会拒绝复用。未选择的黄色一律保留为场景/未批准。黄色审核 shell
默认 0.105 m，仅是候选审计区。构建程序随后扩展 grid，并按体素立方体之间的真实欧氏距离做
默认 0.05 m Minkowski 膨胀。

`planning_ready=true` 只表示对应离线规划门禁通过，不表示可以执行。网格外始终视为占据；
网格内空白目前只是 `assumed_free_not_raycast_verified`。真机前仍需轨迹时间参数化、实时起点
漂移、反馈新鲜度、跟踪误差、急停和人工二次确认门禁。

## 测试

```bash
conda run -n MAP env PYTHONPATH=Avoid python -m unittest discover -s Avoid/tests -v
conda run -n robot env PYTHONPATH=Avoid python -m unittest discover -s Avoid/tests -v
```
