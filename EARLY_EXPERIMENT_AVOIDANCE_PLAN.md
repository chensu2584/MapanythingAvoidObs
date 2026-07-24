# G1/G2 MapAnything 早期避障实验计划

状态：G1 阶段一、二已实现；G2 规划核心已实现，但实机夹爪模型不匹配而强制闭锁
复核日期：2026-07-24
目标代码目录：`/home/ck/MapAnythingTest/Avoid`

## 1. 实验边界

这是低速、有人监管的早期实验版本，不是经过安全认证的运动规划系统。重建地图是静态的，
5 cm 地图膨胀和 2 cm 路径间距只能缓解尺度、深度和离散误差，不能发现重建之后进入场景的
动态障碍物，也不能证明相机未观察区域是自由空间。

首版一次只规划左臂或右臂的 7 个关节。腰、头和另一条手臂固定在规划开始时的实时状态，
但它们的碰撞几何仍参与环境碰撞和自碰撞检查。双臂协同与腰部联合规划不在首版范围内。

## 2. 已确认的坐标与模型事实

- 规划坐标统一为 `base_link`，长度单位为米。
- `voxels.npz` 保存真实 `base_link` 坐标；现有 `voxels.glb` 为查看器绕 X 轴翻转了 180°。
- 规划优先读取 `voxels.npz`。只有在变换、体素大小和坐标系都可证明时才接受独立 GLB；
  缺少元数据时必须拒绝，不能猜测。
- `/home/ck/robot_test/G1.urdf` 可由 Pinocchio 加载为 18 DoF、61 个碰撞对象；碰撞几何是
  自包含的球体、圆柱等 primitive，不依赖缺失的视觉 STL。
- G2 参数包 URDF 可由 Pinocchio 3.9 加载为 `nq=50/nv=46`、35 个碰撞对象；27 个 STL
  package URI 在 `Avoid` 内规范化到参数包实际 `mesh/` 路径，不修改源 URDF。
- 该 G2 URDF 的 omnipicker 末端不是实机当前夹爪；URDF 夹爪碰撞体、中心 frame 和此前
  基于它的路径结论均不能用于实机。正确碰撞外形和 `arm_end_T_TCP` 未提供前规划必须闭锁。
- 当前夹爪中心近似为 `Link_hand_l/r` 局部 `+Z` 方向 0.14308 m。有效 TCP 定义为：

  `base_T_TCP = base_T_Link_hand @ hand_T_TCP_measured`

  实测可保存为绝对 `hand_T_TCP_measured`，也可显式保存为相对 URDF 0.14308 m 的修正；两种
  语义不得混用。实测配置未确认前，GUI 可以显示和 dry-run，但执行器必须拒绝发送运动。

## 3. Operation map

输入支持 MapAnything output 目录、`voxels.npz`、`views.npz`，以及带有可验证配套元数据的
`voxels.glb`。输出默认放在对应 reconstruction 的 `avoidance/` 派生目录，不覆盖原始结果：

- `operation_map.npz`
- `operation_map_base_link.glb`
- `operation_map_manifest.json`
- `self_filter_report.json`

处理顺序：

1. 校验 `base_link`、米制单位、体素大小、原点、dims 和输入哈希。
2. 检测当前 TCP/机器人碰撞体是否落在原始占据体素中。
3. 根据采集时关节状态生成机器人自体候选 mask。候选点不直接静默删除：输出删除前后预览、
   每个 link 的命中数以及与环境几何相邻的歧义点。审核支持两条互不混用的路径：旧 schema v1
   允许批准 core 并精确选择黄色 shell；当前 schema v2 允许从全部已占据源体素中人工选择，
   紫色/黄色仅作提示且只删除绿色精确 mask。两种 review 都绑定输入、URDF、状态和参数哈希，
   未选择部分始终保留。最终地图还必须通过当前完整机器人加 2 cm 间距检查，不能只检查 TCP 点。
4. 扩展网格边界，保证膨胀不会被原 tight bbox 裁切。
5. 对全部环境占据体素做三维欧氏 0.05 m 膨胀，不做语义类别过滤。
6. 记录量化后的实际最小/最大膨胀距离和所有参数。

未知空间策略：网格外始终视为占据；网格内没有占据点的位置在首版中只能标记为
`assumed_free`，GUI 必须显示这一限制。后续再基于相机射线增加 observed-free/unknown 三态地图。

## 4. TCP、实时状态与可达性

独立的 gripper pose worker 从 SDK 读取完整关节状态、WBC Link7 位姿和时间戳，用 G1 URDF FK
交叉验证，并输出左右 `base_T_Link_hand` 和 `base_T_TCP`。过期、缺失或 FK/WBC 超差的数据不能
作为规划起点。

GUI 选择的是目标 TCP 三维位置。首版默认锁定当前 TCP 方向；目标审核必须通过：

- 多初值 IK；
- URDF 关节限位及安全余量；
- 目标位置/方向误差阈值；
- 完整机器人自碰撞；
- 完整机器人到膨胀 operation map 的距离至少 0.02 m。

三维起点只用于显示；真正的规划起点必须是实时完整关节配置，因为相同 TCP 位置可能对应
多种肘部姿态。

## 5. 路径规划算法

主算法为活动手臂 7 维关节空间的双向 RRT-Connect。选择理由是本机 `robot` 环境已有
Pinocchio 3.9 和 hpp-fcl 3.0.2，但没有 OMPL；RRT-Connect 适合快速找到首条可行路径，且比
三维 TCP A* 能正确考虑肘部、腕部、机身和另一条手臂。

每个节点和树边都检查完整机器人。边采用自适应关节插值，限制相邻检查点的最大关节变化和
最大 TCP/连杆位移；不能只检查离散 waypoint。环境距离针对已经膨胀 5 cm 的地图再要求至少
2 cm，因此相对原始重建表面的名义安全距离约为 7 cm，另有体素量化余量。

成功路径已经过碰撞约束下的 shortcut smoothing 和稠密复核。Ruckig 速度、加速度和 jerk
受限的 20--30 Hz 轨迹仍是下一阶段；任何时间参数化之后都必须再次密集碰撞检查。

## 6. 模块划分

```text
Avoid/
├── configs/
│   ├── avoidance_defaults.json
│   ├── g2_end_effector_model.json
│   └── tcp_calibration.json
├── avoidance/
│   ├── contracts.py
│   ├── map_io.py
│   ├── operation_map.py
│   ├── shell_review.py
│   ├── tcp_model.py
│   ├── robot_model.py
│   ├── planning_scene.py
│   ├── end_effector_model.py
│   ├── g2_robot_model.py
│   ├── ik_solver.py
│   ├── collision_checker.py
│   ├── rrt_connect.py
│   ├── planner.py
│   ├── trajectory.py                 # 未实现
│   ├── sdk_bridge.py
│   └── worker_protocol.py
├── scripts/
│   ├── report_gripper_pose.py
│   ├── build_operation_map.py
│   ├── review_self_filter.py
│   ├── audit_planning_scene.py
│   ├── plan_g2_avoidance.py
│   ├── audit_g2_planning_starts.py
│   ├── execute_avoidance.py          # 未实现
│   └── avoidance_gui.py
└── tests/
```

GUI 只负责选择、显示、确认和调用 worker。地图处理、FK/IK、碰撞、规划、时间参数化与 SDK
执行不得堆入 GUI 文件。MAP 环境负责 Open3D/Trimesh 地图处理和显示；robot 环境负责
Pinocchio/hpp-fcl、实时状态和执行；当前 GUI 已通过一次性 JSON 文件 worker 跨环境调用，
后续实时反馈再扩展为长驻协议。

## 7. 执行门禁

执行器默认 dry-run。正式执行必须同时满足：TCP 实测已确认；地图、URDF、TCP 和 plan 哈希
一致；实时起点未漂移；SERVO 正常；反馈新鲜；无 SDK 错误/碰撞报警；整条轨迹复核通过；
用户二次确认。运行中监控跟踪误差和 SDK 状态，任一条件失效立即调用 motion stop，且不得
自动重规划后继续运行。

## 8. 验收顺序

1. 单体素、墙体和边界场景验证 5 cm 膨胀。
2. 验证 NPZ/GLB 坐标转换和 `base_link` 契约。
3. 离线回放最新 capture，验证 TCP、FK/WBC 与自体 mask 审核。
4. 合成场景验证 IK、关节限位、自碰撞和环境距离。
5. 构造“TCP 路径自由但肘部碰撞”的反例，必须拒绝。
6. 验证 RRT 路径全部连续插值点距离膨胀地图不小于 2 cm。
7. fake SDK 验证起点漂移、过期反馈、跟踪误差和急停。
8. 真机只读显示、原位 SERVO、无障碍 1--2 cm 小运动、软障碍低速测试，逐级开放。

只有以上离线门禁完成后，才开始真机路径执行验证。

## 9. 当前实现进度（2026-07-24）

- 阶段一已实现：TCP 标定配置、离线 capture 位姿报告、只读实时 SDK 位姿报告、URDF FK 与
  WBC Link7 交叉校验。TCP 配置仍为未确认，执行门禁保持关闭。
- 阶段二已实现：output 目录/`voxels.npz`/`views.npz` 输入、GLB 到同目录规范 NPZ 的安全
  路由、自机候选 mask、审核预览、显式批准门禁、欧氏 5 cm 膨胀、边界扩展、NPZ/GLB/
  manifest/report 输出。
- `142521` 已增加 schema v2 全场景人工审核：39,552 个源占据体素均可选择，只删除人工绿色
  mask，不自动批准紫色 core。当前草稿 selected=0、`review_complete=false`，自动夹爪边界判断
  延后；在操作者完成选择之前不能生成可用的 self-filtered operation map。
- 旧 schema v1 黄色审核继续兼容；两种模式使用不同文件名和契约，禁止混用。
- G2 阶段三已实现：22 关节 capture 起点校验、URDF/STL FK、35 个完整机器人碰撞体、
  URDF 与 mesh bundle 哈希绑定的允许碰撞矩阵、box/cylinder 环境 HPP-FCL、多初值 IK、
  7 维 RRT-Connect、
  shortcut 和按关节/连杆/TCP 位移自适应的稠密复核。
- 上述是规划核心能力，不代表当前实机模型完整。新增 `g2_end_effector_model.json` 门禁后，
  `expoutput3` 六个起点全部 `not_run`；旧 4/6 结论和 omnipicker 烟测已撤回。
- G2 避障 GUI 已实现：可选择 6 个聚类 snapshot 和左右臂、调整 7 轴目标、规划、拖动时间轴与
  播放路径；显示原始基本体、10 cm 安全边界、整机骨架和诊断。由于实机夹爪未知，只开放
  `arm_body_demo`，排除 4 个 `gripper_*` omnipicker 几何并使用腕部法兰，输出不能执行。
- 阶段四轨迹时间参数化、实时状态漂移门禁和 SDK 执行尚未实现；全部计划清单固定
  `execution_authorized=false`，没有发送任何运动命令。

逐项完成记录、产物哈希、测试结果和恢复命令见 `WORK_LOG.md`。
