# G1 MapAnything 避障实验工作记录

本文件记录已经实际完成并验证的工作。设计意图与未来阶段见
`EARLY_EXPERIMENT_AVOIDANCE_PLAN.md`，操作命令见 `README.md`。除非另有明确记录，所有结果
均属于低速、有人监管的早期实验，不代表真机执行安全认证。

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
- 阶段三以后的 IK、完整碰撞检查、RRT-Connect、轨迹和执行未实现。
- 当前任何 `planning_ready` 都不等于 `execution_ready`；现阶段不得发送真机运动。
