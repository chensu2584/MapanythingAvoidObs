# G1 MapAnything 避障实验工作记录

本文件记录已经实际完成并验证的工作。设计意图与未来阶段见
`EARLY_EXPERIMENT_AVOIDANCE_PLAN.md`，操作命令见 `README.md`。除非另有明确记录，所有结果
均属于低速、有人监管的早期实验，不代表真机执行安全认证。

## 2026-07-24：纠正 G2 实机夹爪与 URDF 末端不匹配

用户确认参数包 URDF 中的 omnipicker/智慧手不是实机当前安装夹爪。仓库内没有当前夹爪的
替代 URDF、mesh、保守尺寸或 TCP 标定，因此：

- 新增 `configs/g2_end_effector_model.json` 和 `avoidance/end_effector_model.py`，明确记录
  `urdf_matches_installed=false`、碰撞模型未确认、TCP 未确认。
- `G2CollisionChecker` 在进入 HPP-FCL 前强制检查末端工具型号、URDF/mesh 哈希、碰撞几何和
  TCP；当前必定拒绝。
- `plan_g2_avoidance.py` 仍写出哈希绑定 manifest，但状态为 `blocked`，不运行碰撞、IK 或 RRT，
  不输出 TCP 与路径。
- 六帧审计更新为 6/6 `not_run`、`valid_start_count=0`；旧 4/6 结果明确标为无效。
- `reports/g2_smoke_plan.json` 已覆盖为 blocked 清单，旧 `+0.03 rad` 路径不再有效。
- `robot` 环境 24/24 测试通过，覆盖错误末端模型拒绝与规划器 fail-closed。

恢复条件：提供当前左右夹爪 collision mesh 或保守基本体、相对 arm end 的安装变换，以及
实测左右 TCP；重新绑定 URDF/mesh/config 哈希并审核后，才可恢复离线碰撞和规划。

## 2026-07-24：G2 阶段三离线单臂避障规划

> 本节记录规划核心的首次实现。其基于 omnipicker 末端得到的碰撞数值和路径结果，已被上节
> 撤回；机械臂本体、IK 和 RRT 代码保留，等待接入正确实机夹爪模型。

### 完成内容

- `avoidance/g2_robot_model.py`：读取 22 个 capture 关节，加载 G2 `nq=50/nv=46` 模型、
  35 个碰撞对象、双臂限位和夹爪中心 FK。
- `avoidance/collision_checker.py`：HPP-FCL 完整机器人自碰撞及 mesh 对 box/cylinder 环境
  碰撞；环境原始尺寸应用 8 cm 膨胀和 2 cm 额外间距。
- `configs/g2_allowed_self_collisions.json`：显式记录四组 URDF 凸包设计内接触并绑定 URDF
  和 26 个唯一碰撞 mesh 的 bundle SHA-256，不从当前姿态动态放行。
- `avoidance/ik_solver.py`：左右单臂多初值、有界、碰撞过滤的目标位姿 IK。
- `avoidance/rrt_connect.py` 与 `avoidance/planner.py`：双向 RRT-Connect、shortcut、最大
  0.04 rad 关节步长、最大 0.02 m 连杆/TCP 位移细分及最终稠密复核。
- `scripts/plan_g2_avoidance.py`：支持关节目标或 `base_T_goal`，输出哈希绑定计划清单；
  `execution_authorized=false` 固定关闭。
- `scripts/audit_g2_planning_starts.py`：批量输出六帧完整机器人起点审计。

### 实测结果

- 6/6 capture 均无未允许自碰撞；每个场景检查 507 个自碰撞对。
- `034729`、`034806`、`034838`、`035236` 起点有效，安全裕量之后最小环境间距分别约
  16.6、12.8、41.0、42.0 mm。
- `034923`、`035146` 因左夹爪扫掠体与物件相交而闭锁。
- `034729` 左臂关节 1 增加 0.03 rad 的非零目标完成离线直线路径和稠密复核。
- 当前 TCP 位姿作为 IK 目标时，3 个 seed 中 2 个得到无碰撞容差内解。

### 验证

- `MAP` 环境：22 项测试通过，2 项 robot 后端测试按环境跳过。
- `robot` 环境：22/22 测试通过。
- 批量报告：`reports/g2_expoutput3_start_collision_audit.json`。
- 烟测计划：`reports/g2_smoke_plan.json`，所有输入哈希齐全且执行授权关闭。
- 未实现轨迹时间参数化、实时反馈门禁、GUI 和 SDK 执行；没有发送任何运动命令。

## 2026-07-24：G2 聚类场景规划输入迁入 Avoid

### 完成内容

- 新增 `avoidance/planning_scene.py`：
  - 严格读取 `obstacles.json`，拒绝错误坐标系、单位、尺寸和非竖直圆柱；
  - 将 `support/object` 的 `box/cylinder` 与 `markers` 分开；
  - 提供盒/圆柱有符号距离和独立规划膨胀查询；
  - GLB 输入只路由到同目录 JSON，不从可视化网格反推碰撞几何。
- 新增 `scripts/audit_planning_scene.py` 和
  `reports/g2_expoutput3_planning_audit.json`，完成全部 6 帧批量审计。
- 新增 `tests/test_planning_scene.py`，覆盖基本体读取、圆柱方向、frame fail-closed、
  marker 排除、距离和规划膨胀语义。
- 新增 `G2_CLUSTER_PLANNING_READINESS.md`，固定后续 G2 robot/collision/IK/RRT 实现顺序。

### 审计结论

- 6/6 场景契约有效，6/6 marker 完整。
- 主蓝料箱中心跨帧标准差约 5–7 mm，尺寸标准差约 6–11 mm。
- 解析环境距离查询可用。
- 当时 G2 单臂规划、移动底盘规划和轨迹执行均保持 `ready=false`；同日后续阶段三记录见上节。
- 该输入迁移步骤本身没有创建规划路径、轨迹或机器人控制代码。

## 2026-07-21：142521 全场景人工夹爪体素审核

### 决策

- 当前自动 URDF core（紫色）与 ambiguity shell（黄色）不能可靠覆盖全部夹爪重建体素。
- 自动判断夹爪边界的问题暂时延后，不继续通过扩大自动 shell 来猜测。
- 当前实验改为全场景精确人工审核：全部源占据体素均可选择，紫色、黄色和灰色只是视觉提示。
- 后续构图只删除操作者明确选择为绿色的体素；未选择体素一律保留为场景。
- 全场景人工 review 不得与 `--approve-self-filter` 组合，防止自动紫色 core 被额外删除。

### 实现

- `Avoid/scripts/review_self_filter.py`
  - 新增 `--selection-scope all-occupied`；旧 `ambiguity-shell` 模式继续兼容。
  - add/remove 笔刷的候选域可扩展到全部占据体素。
  - 全场景模式不要求 `--approve-core`，并使用独立文件名，避免覆盖旧黄色审核。
  - 首次运行会自动创建 `avoidance/`，修复新 capture 在预览导出前目录不存在导致的保存失败。
- `Avoid/avoidance/shell_review.py`
  - 新增 schema v2：`exact_occupied_voxel_review`。
  - 保存 `selected_robot_indices` 和 `selected_robot_voxel_count`，并绑定源体素、capture state、
    G1 URDF、候选参数及文件哈希。
  - 校验选择必须来自真实源占据索引；越界、重复、非占据点、输入变化或草稿状态均 fail closed。
  - schema v1 黄色 review 的契约和读取保持兼容。
- `Avoid/avoidance/operation_map.py` 与 `Avoid/scripts/build_operation_map.py`
  - 能读取完成的全场景 schema v2 review。
  - 全场景模式仅使用精确人工 mask，不自动并入紫色 core。
  - manifest/report 新增 selection scope、人工删除总数、黄色交集数及自动 core 是否获批。
  - 未完成 review 会被拒绝；全场景 review 与 `--approve-self-filter` 同时出现也会被拒绝。
- `Avoid/tests/test_shell_review.py`
  - 增加全场景已占据点可选、非占据点拒绝、schema v2 round-trip 测试。

### 142521 输入与当前产物

输入目录：`outputs/g1_capture_20260721_142521`

- 规范输入：`voxels.npz`
- 体素大小：约 `0.01 m`
- 源占据体素：`39,552`
- grid dims：`[262, 434, 199]`
- matching state：`capture_state.json`
- G1 模型：`/home/ck/robot_test/G1.urdf`

已生成可继续编辑的空白草稿：

- `avoidance/manual_self_filter_review.json`
  - 大小：`1,871` bytes
  - 当前空白草稿 SHA-256：`b65a0b2c5f11454fe28fd98fee395b4c37eb5d9b5b424c1a0bb9a61efb12a208`
  - schema：`2`
  - selection scope：`all_occupied`
  - selected：`0`
  - `review_complete=false`
- `avoidance/manual_self_filter_review_base_link.glb`
  - 大小：`10,793,384` bytes
  - SHA-256：`3384c2187b3c81eb8c726f7f8489ea3a3f546014da2f2f3ace2f8c693bf6c05f`
  - Trimesh 复读结果：2 个 geometry，317,256 vertices，476,288 faces

草稿保持 incomplete 是有意的：必须先由操作者完成绿色体素选择，再显式 mark complete。已实测
将该草稿传给 `build_operation_map.py` 会以 `Self-filter review is still a draft` 拒绝构图。

### 操作者下一步

增加人工选择：

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_142521 \
  --selection-scope all-occupied \
  --mode add \
  --brush-radius-m 0.02
```

误选时把 `--mode add` 改成 `--mode remove`。最终人工确认后：

```bash
conda run -n MAP python Avoid/scripts/review_self_filter.py \
  --input outputs/g1_capture_20260721_142521 \
  --selection-scope all-occupied \
  --no-gui --mark-complete --yes
```

随后才能把完成的 `manual_self_filter_review.json` 交给操作地图构建；不要附加
`--approve-self-filter`。

### 验证

- `python -m py_compile`：修改的 4 个 Python 模块通过。
- `conda run -n MAP env PYTHONPATH=Avoid python -m unittest discover -s Avoid/tests -v`：12/12 通过。
- GUI 运行依赖已确认：Open3D 0.19.0，SciPy 1.15.3。
- 没有创建 IK、规划或执行功能，没有向机器人发送运动命令。

## 2026-07-21：浏览器测量器增加 GLB 支持

实际页面位置为 `TestData/reconstruction_outputs/measure_viewer.html`；不存在重复的
`TestData/recon/measure_viewer.html`。

完成内容：

- 文件选择和拖放同时接受 `.ply` 与 `.glb`。
- 内置 GLB 2.0 解析支持 embedded BIN、accessor/bufferView、节点 matrix/TRS、`POSITION`、
  `COLOR_0`、材质基础色、点 primitive 以及有/无索引的网格 primitive。
- PLY 继续采用 OpenCV Y-down 视角；GLB 按 glTF Y-up 渲染。
- 保留两点测距；测量点吸附到最近投影顶点。
- 多次加载时释放旧 WebGL buffer，避免持续占用显存。

验证结果：

- JavaScript 语法检查通过。
- `g_1_Test_1..4/scene.glb` 全部解析成功，顶点数分别为 547,292、535,070、545,257、
  519,294。
- 无头 Firefox 实际选择 `g_1_Test_1/scene.glb` 后，页面显示 547,292 顶点；画布截图包含
  13,806 种颜色，确认不是只完成解析而未绘制。
- 原 HTML 内嵌默认点云数据保持不变。

## 当前未完成与恢复边界

- 142521 尚未完成人工绿色体素选择，review 不能用于正式 operation map。
- 自动夹爪识别、夹爪边界估计和跨姿态泛化明确延后。
- TCP 实测配置仍未确认。
- G2 IK/RRT 核心与机械臂本体演示 GUI 已实现；实机夹爪碰撞与 TCP 未建模，完整 G2 规划
  仍强制闭锁，演示路径不可执行。
- 轨迹时间参数化、实时门禁和执行仍未实现。
- 当前任何 `planning_ready` 都不等于 `execution_ready`；现阶段不得发送真机运动。

## 2026-07-24：G2 聚类避障 GUI

- 新增 `scripts/avoidance_gui.py`：Tkinter + Matplotlib 3D，支持 snapshot/左右臂选择、7 轴
  目标滑杆、双向 RRT-Connect、时间轴和路径播放。
- 新增 `scripts/g2_gui_worker.py`：GUI 在 MAP 环境运行，FK、HPP-FCL 和规划在 robot 环境中
  通过一次性 JSON 请求执行。
- 碰撞检查器和规划器新增显式 `arm_body_demo` 作用域。它只排除 4 个名称为 `gripper_*` 的
  错误 omnipicker 几何，保留机身、头部、双臂与场景碰撞，跟踪腕部法兰且拒绝 TCP 目标。
- 正常模式继续要求 `g2_end_effector_model.json` 全部一致性门禁通过；GUI 演示输出固定
  `execution_authorized=false` 和 `execution_valid=false`。
- 无显示冒烟测试产物：
  `reports/avoidance_gui_smoke.json`、`reports/avoidance_gui_smoke.png`。
