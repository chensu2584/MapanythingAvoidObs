# G1 MapAnything 避障：阶段一、二

这里目前只实现早期实验的前两阶段：夹爪 TCP 位姿/标定报告，以及从 MapAnything 占据体素生成
操作地图。没有 IK、路径规划、轨迹生成或运动执行代码。

当前恢复点：`142521` 已建立全场景人工审核 schema v2 空白草稿，`39,552` 个源占据体素均可
选择；当前 `selected=0`、`review_complete=false`，等待操作者进入 GUI 标绿。自动夹爪边界判断
明确延后，不能把紫色/黄色提示当作已批准删除。完整实现与验证记录见
[`WORK_LOG.md`](WORK_LOG.md)。

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

`planning_ready=true` 只表示阶段二地图门禁通过，不表示可以执行。网格外始终视为占据；网格内
空白目前只是 `assumed_free_not_raycast_verified`。后续规划器尚未实现，且还必须额外满足 TCP
实测、2 cm 路径间距、完整机器人碰撞和实时反馈门禁。

## 测试

```bash
conda run -n MAP env PYTHONPATH=Avoid python -m unittest discover -s Avoid/tests -v
```
