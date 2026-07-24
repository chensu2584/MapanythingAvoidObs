# G2 危险起点恢复、关节可行性与场景精度计划

状态：设计与进度记录，尚未实现危险起点恢复规划
更新日期：2026-07-24
适用范围：`arm_body_demo_excludes_unconfirmed_end_effector`，所有结果禁止用于实机执行

## 1. 当前问题与结论

`G2/3box/snapshot_20260724_040817_0003` 已能在新版 GUI 中完成物体边点选、
XYZ/offset 调整和 Cartesian preview，但当前规划器无法从该帧起点开始：

- 简化场景包含 1 个 support 和 4 个 object box；
- primitive `2` 的尺寸约为 `[0.52, 0.76, 0.32] m`；
- HPP-FCL 报告 `arm_l_link5_0`、`arm_l_link6_0`、`arm_l_link7_0` 与 primitive `2`
  接触；
- 没有起点自碰撞；
- 环境几何使用 8 cm inflation 和 2 cm clearance，总安全裕量为 10 cm；
- `planner.py` 和 `rrt_connect.py` 当前都在起点无效时立即拒绝。

目标不是全局关闭碰撞检查，而是：

> 允许规划从一个已知的环境危险状态开始，生成一段严格受约束的脱离前缀；首次进入正常
> 无碰区域后，后续路径必须一直满足完整碰撞约束，不得重新进入危险区。

该能力只能先用于离线算法演示。未知实机夹爪仍使 `execution_authorized=false`。

## 2. 危险起点恢复设计

### 2.1 不采用的做法

以下做法会掩盖真实风险，不允许使用：

- 把发生起点碰撞的整个 primitive 从场景中删除；
- 对整条路径关闭该 primitive 的碰撞检查；
- 仅忽略起点采样，然后让普通 RRT 从无效节点扩展；
- 只检查离散路点，不检查路点之间的连续边；
- 允许初始接触集合在恢复过程中扩大；
- 把未知夹爪或自碰撞也归入“允许起步”的例外。

### 2.2 两阶段规划

规划拆为两个明确阶段。

阶段 A：受约束脱离前缀。

1. 在 `q0` 记录初始环境接触集合 `C0={(robot_geometry, primitive_id)}`；
2. 自碰撞必须为空，否则仍立即拒绝；
3. 搜索短距离关节路径 `q0...q_escape`；
4. 每条边保持关节限位和连续细分检查；
5. 中间状态只能保留 `C0` 的子集，不得产生新接触；
6. 危险度必须总体下降，不能先明显加重再穿出障碍；
7. `q_escape` 必须通过正常的完整碰撞与 clearance 检查。

阶段 B：正常避障规划。

1. 从 `q_escape` 启动现有双向 RRT-Connect；
2. 使用现有全部机器人自碰撞、环境碰撞和 10 cm 总裕量；
3. 平滑与稠密复检不得使用危险起点例外；
4. 从首次 clear 的路点起，任何重新进入 `C0` 或其他危险区都使路径失败。

最终路径为 `recovery_prefix + normal_rrt_path`。manifest 必须分别记录两阶段结果。

### 2.3 危险度定义

当前 `CollisionReport` 只报告接触对和最小非碰距离，发生碰撞时没有穿透深度。实施前需要让
HPP-FCL 返回每个接触对的 signed distance 或 penetration depth，并定义可审计的危险度：

```text
risk(q) = (
  new_contact_count,
  self_collision_count,
  initial_contact_count,
  total_penetration_depth,
  maximum_penetration_depth,
  clearance_deficit
)
```

比较采用字典序，优先禁止新接触和自碰撞，再要求初始接触数量及穿透程度下降。考虑数值噪声时
可允许很小的平台区间，但连续若干步不能改善就终止该恢复分支。

### 2.4 恢复搜索

首版建议采用短程、单向、多初值的 joint-space 恢复搜索，不直接修改通用 RRT：

- 只改变当前活动的 7 个手臂关节；
- 候选步长不大于正常 `extension_step_rad`；
- 使用有限差分估计 `risk(q)` 对关节的下降方向；
- 同时尝试远离接触 primitive 中心的法兰/肘部 Jacobian 方向；
- 每条候选边沿现有 `edge_step_rad` 和 `max_link_step_m` 双重细分；
- 设置最大恢复关节弧长、路点数和超时；
- 找不到 clear 状态时返回 `danger_start_escape_failed`，不进入 RRT。

后续若局部恢复容易卡住，再实现带 risk cost 的单向树搜索。不要让标准双向 RRT 的 goal 树
继承危险状态语义。

### 2.5 路径验收与 GUI

manifest 至少新增：

- `start_policy=constrained_environment_escape`；
- 初始接触对、穿透深度和场景 primitive；
- `recovery_waypoints`、`first_clear_index`；
- 恢复过程最大/最终危险度；
- 是否出现新接触；
- clear 后是否重新进入危险区；
- 恢复与正常路径各自的稠密复检结果。

GUI 状态建议：

- 红色：自碰撞、恢复失败或出现新接触；
- 黄色：起点危险，但已找到受约束脱离前缀；
- 绿色：起点正常，或脱离后完整路径通过；
- 路径中恢复前缀用橙色，正常路径继续用黄色；
- 诊断区明确列出初始接触、首次 clear 路点和“实机执行：禁止”。

## 3. 当前如何确保关节可行性

当前实现已经覆盖以下运动学可行性：

1. `capture_state.json` 必须包含 G2 body、head、左右臂所需关节，数值必须有限；
2. Pinocchio 从绑定哈希的 G2 URDF 构建模型；
3. 活动臂 7 个关节使用 URDF 上下限，并在两端各保留 `0.02 rad` margin；
4. 手工关节目标和每个 RRT 节点都必须在该收紧限位内；
5. Cartesian 目标通过 12 个初值的 IK，位置误差要求不超过 5 mm，旋转误差不超过 2°；
6. IK 迭代每步裁剪到关节限位，候选解还必须通过碰撞检查；
7. RRT 采样范围受同一关节限位约束；
8. 路径边按最大关节变化 0.04 rad 细分；
9. 另按任一机器人碰撞几何中心最大位移 0.02 m 增加细分；
10. RRT 扩展边的每个细分状态都运行完整机器人和环境碰撞检查；
11. 输出阶段会再次查询稠密路点并记录 clearance。

当前尚未覆盖：

- 速度、加速度、jerk 和力矩限制；
- 时间参数化及控制周期；
- 奇异点裕量、Jacobian condition number；
- 电机、线缆、软限位和控制器内部约束；
- 真实夹爪负载、惯量和扫掠体；
- 传感器时延、跟踪误差和动态障碍；
- 全身联动规划。目前 body、head 和非活动臂固定在 snapshot 起点。
- `smoothing_attempts` 虽有配置，但当前 RRT 尚未实际执行路径平滑；
- 输出阶段当前直接写 `dense_recheck_passed=true`，没有在复查发现异常时再次 fail closed；
  实现危险起点恢复前必须补上该断言和回归测试。

因此当前“关节可行”只代表：

> 在固定其余关节的前提下，活动臂的离线几何/运动学路径满足收紧关节限位和稠密碰撞检查。

它不等于动力学可行，更不等于可以发送给机器人。

## 4. 当前如何处理手部夹爪

G2 参数包 URDF 描述的是 omnipicker，但机器人当前安装的夹爪不是该设备。当前处理是
fail-closed：

- `configs/g2_end_effector_model.json` 声明 installed model 为
  `unknown_non_omnipicker_gripper`；
- 实机夹爪 collision geometry 未确认；
- 左右 `arm_end_T_tcp` 未测量；
- 正常完整规划因末端模型门禁不通过而 blocked；
- GUI 只开放 `arm_body_demo`；
- demo 排除 4 个名称为 `gripper_*` 的错误 omnipicker 碰撞几何：
  `gripper_l_camera_link_0`、`gripper_l_base_link_0`、
  `gripper_r_camera_link_0`、`gripper_r_base_link_0`；
- 机身、头部、左右机械臂和非活动臂仍参加碰撞检查；
- Cartesian 目标跟踪 `arm_l_end_link`/`arm_r_end_link` 法兰，不跟踪
  `gripper_l/r_center_link`，也不声称是真实 TCP；
- 所有结果固定 `execution_authorized=false`。

这是一种“排除已知错误模型、保留机械臂本体研究能力”的临时办法。它不能验证夹爪是否撞到
蓝盒子、桌面或物件，也不能验证抓取点是否可达。

恢复完整夹爪规划前必须提供：

1. 当前左右夹爪型号或测量尺寸；
2. 保守 collision mesh 或 box/cylinder 组合；
3. 相对左右 `arm_*_end_link` 的安装变换；
4. 实测 `arm_end_T_tcp`；
5. 更新后的 URDF/mesh/config 哈希和人工确认；
6. 加入夹爪后的起点、IK、路径与扫掠碰撞回归。

## 5. 当前聚类如何保证抽象精度

### 5.1 已有保障

当前 `scripts/simplify_g2_snapshots.py` 使用：

- 输入 `voxels.npz` 的 `base_link`/meter 契约；
- snapshot 匹配的旧格式相机外参和输入 SHA-256；
- 相机中心附近的空间夹爪体素移除；
- count filter；
- 3D DBSCAN 小簇去噪；
- Z 直方图支撑面检测；
- 桌面高度附近最大 XY cluster 作为 support；
- 高于桌面的 XY cluster 拟合 axis-aligned box；
- `min_cluster`、`cluster_eps` 和最小障碍高度阈值；
- 体素尺寸参与 AABB padding，避免退化成 0 m 厚度；
- 输出经过 `load_planning_scene` 严格 schema/正尺寸校验；
- 规划侧独立应用 inflation，不重复使用可视化 inflation。

这些措施能保证输入输出一致、基本体合法和较保守的几何包围，但不能保证语义抽象准确。

### 5.2 当前没有的保证

目前没有自动证明：

- 三个主体都被保留；
- 相邻物体没有被合并；
- 一个物体没有被切成多个碎片；
- AABB 没有包含大面积自由空间；
- 机器人残留没有被当成障碍；
- 相机附近删除没有误删真实物件；
- 不同 snapshot 的同一物体保持尺寸和位置一致。

第三帧 primitive `2` 的大 AABB 与左臂发生接触，是“契约合法但抽象未必准确”的具体案例。
调小 `cluster_eps` 到 `0.02 m` 后仍保留该大 box，说明仅调 DBSCAN 阈值不足以解决相连表面
或相邻物体误合并。

### 5.3 必须新增的量化验收

每个 snapshot 应生成独立 accuracy report，至少包含：

1. **保留率**：清理后、桌面以上源体素被任一 object primitive 覆盖的比例；
2. **过包围率**：primitive 内采样点到最近源占据体素超过阈值的比例；
3. **组件一致性**：源连通组件与 primitive 的一对一/一对多/多对一关系；
4. **跨帧稳定性**：匹配主体的中心、尺寸、颜色和体积漂移；
5. **主体门禁**：桌面和预期三个主体缺失时失败；
6. **机器人残留审计**：primitive 与 snapshot 机器人 collision geometry 的 raw 和 inflated
   重叠分别报告；
7. **近场可视审核**：保存原体素、基本体和机器人叠加截图/GLB；
8. **参数与哈希**：记录 voxel size、DBSCAN、去噪、桌面、gripper removal 和 primitive
   拟合参数。

建议第一版门禁：

- support 数量必须为 1；
- 三个已知主体必须能人工对应到独立 primitive；
- 主体源体素覆盖率不低于 98%；
- 出现多组件合并时标记 `manual_review_required`；
- raw primitive 与机械臂本体重叠时禁止直接用于规划；
- 只有 inflation 引起的起点接触单独标为 `near_clearance_start`；
- 未完成人工审核时只能进入 GUI preview，不能作为“场景精度已通过”的数据。

后续拟合应优先考虑：

- 在 XY 连通之外加入颜色、法向、高度层和可见性边界；
- 对高过包围率 cluster 做递归切分；
- box/cylinder 多模型拟合并以残差选择；
- 跨 snapshot 数据关联，利用主体持续性抑制单帧误合并；
- 同时保存 raw occupancy 与 primitive，规划诊断可回查原始证据。

## 6. 2026-07-24 工作进度记录

已完成：

- 阅读 Cartesian GUI 最新实施文档和代码；
- 用 `G2/3box` 第三组数据生成 Avoid 场景；
- 修复 object AABB 未传 `voxel_size` 导致单轴 `0 m` 的问题；
- 显式保持首版 `primitive_mode=box`，与当前 box-edge picking 一致；
- 最终场景为 1 support + 4 objects；
- 在 `MAP` GUI + `robot` worker 环境启动新版 GUI；
- 实际验证画布点选、XYZ/offset 面板、preview 请求和 Pinocchio/HPP-FCL 响应；
- 记录第三帧危险起点接触和 10 cm 总环境裕量；
- 形成危险起点两阶段恢复方案。

尚未完成：

- 危险起点 recovery prefix 实现；
- penetration depth/signed-distance 风险报告；
- clear 后禁止重入的路径验证器；
- cluster accuracy report 和主体自动门禁；
- 第三帧 primitive `2` 的切分或人工确认；
- 已确认实机夹爪模型、TCP 和执行能力；
- 时间参数化、动力学检查与实机执行。

## 7. 建议实施顺序

1. 扩展 `CollisionReport`，输出逐接触 signed distance/penetration；
2. 增加 recovery risk 与边验证单元测试；
3. 实现短程受约束脱离搜索；
4. 将 `q_escape` 接入现有 RRT-Connect；
5. 增加 clear 后禁止重入的稠密复检；
6. 在 GUI 中显示危险起点、恢复前缀和失败原因；
7. 增加聚类 accuracy report，再处理第三帧误合并；
8. 最后才考虑真实夹爪和实机执行链路。
