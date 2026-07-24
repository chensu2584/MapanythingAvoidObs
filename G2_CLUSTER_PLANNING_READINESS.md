# G2 聚类场景避障可行性

状态：环境基本体、单臂 RRT 核心和演示 GUI 已实现；完整实机规划仍被夹爪模型门禁阻断。

`G2/expoutput3` 的 6 个 snapshot 均使用米制 `base_link`，保留桌面、蓝盒子和其他物件，并以
box/cylinder 作为碰撞输入。GLB 只用于查看，规划读取同目录 `obstacles.json`。

GUI 的 `arm_body_demo` 用于回答“这样的场景重建能否驱动绕障搜索”：机身、头部、双臂和环境
均参与 HPP-FCL，边按关节与碰撞体位移稠密复检。由于 G2 URDF 中是错误的 omnipicker，演示仅
排除 4 个 `gripper_*` 碰撞几何，跟踪 `arm_l/r_end_link` 法兰并拒绝 TCP 目标。输出固定
`execution_authorized=false`。

完整规划恢复前必须提供当前左右夹爪的保守碰撞几何、相对 arm end 的安装变换和实测 TCP，
随后重新绑定 URDF/mesh/config 哈希。还需补 observed-free/unknown 工作区、轨迹时间参数化、
实时起点漂移与执行门禁，才可讨论真机试验。
