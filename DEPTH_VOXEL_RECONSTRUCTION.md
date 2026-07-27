# G2 直接深度体素重建与 3box 桌面裁剪

## 直接深度对照

脚本：

```text
Avoid/scripts/reconstruct_depth_voxels.py
```

它不调用 MapAnything，不补全遮挡面，只把 G2 测得的深度反投影到 `base_link` 后进行体素融合。

原始采集 snapshot：

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP \
  python Avoid/scripts/reconstruct_depth_voxels.py \
  --input G2/session_*/snapshot_*
```

原始模式读取：

- `camera_extrinsics.json`
- `head_depth_raw16.png`
- `head_rgb.png`
- 可选的 `hand_left/right_depth_raw16.png` 和对应 RGB

`uint16` 深度按毫米解码，`0` 和 `65535` 无效。深度点使用深度相机内参反投影，再使用
`base_T_depth_camera` 变换到 `base_link`；颜色通过 RGB 相机外参和畸变参数重新投影取得。

当前 3box 的原始 session 已不存在，但保留了注册深度，可运行：

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP \
  python Avoid/scripts/reconstruct_depth_voxels.py \
  --input G2/3box/undistorted
```

输出目录：

```text
G2/3box/direct_depth_reconstruction/snapshot_*/
```

每帧包含：

- `direct_depth_voxels.npz`：`base_link` 米制稀疏体素；
- `direct_depth_voxels.glb`：彩色立方体及 G2 位姿参考标记 GLB；
- `direct_depth_manifest.json`：输入哈希、参数、视图、点数、体素数、范围和输出哈希。

3box 的 `registered_depth.npz` 当前只有 head view，因此这是单头部深度对照，不是三深度相机
融合。五帧分别得到 25,605、25,469、25,010、24,967、24,747 个 1 cm 体素。

## GLB 机器人参考标记

以下派生 GLB 使用同一套标记生成器 `Avoid/scripts/g2_glb_markers.py`：

- `snapshot_*/cleaned_voxels.glb`
- `snapshot_*/obstacles.glb`
- `direct_depth_reconstruction/snapshot_*/direct_depth_voxels.glb`

每帧显示 `base_link` 坐标原点、头部相机、左右手部相机、左右腕部法兰参考中心，以及由掌部和
两根手指构成的简约左右手。相机使用该帧导出的 `camera_to_world` 位姿；腕部法兰由
`capture_state.json` 关节角和 G2 URDF 做 FK 得到。

这里的“夹爪中心”实际是 `arm_l_end_link`/`arm_r_end_link` 法兰参考点。当前实装夹爪与
URDF 中 omnipicker 不一致，尚无可信 `arm_end_T_tcp`，因此简约手、法兰球和坐标轴：

- 只用于辨认机器人与场景的相对位置；
- 不写入 `boxes`，不参与规划碰撞；
- 不能解释为真实夹爪尺寸、TCP 或扫掠体。

原始 `scene.glb`、`scene_filtered*.glb` 和 `voxels.glb` 保留为重建证据，不被后处理脚本
覆写。需要规划或对比时使用上述三个带标记的派生 GLB。

## 3box 桌面范围

测量图中的 GLB 使用绕 X 轴 180° 的 viewer transform。将四点换回 `base_link` 后采用：

```text
X = [0.239, 1.019] m
Y = [-0.694, 0.706] m
```

桌面裁剪简化命令：

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=Avoid \
  python Avoid/scripts/simplify_g2_snapshots.py \
  --scene-root G2/3box \
  --pipeline MapAnythingPipeline \
  --table-xy-bounds 0.239 1.019 -0.694 0.706
```

流程先删除 XY 矩形外体素，再检测桌面高度，最后删除低于
`table_top_z - table_thickness` 的点。输出的 `cleaned_voxels.glb` 因每个体素立方体有半体素
尺寸，外表面可比裁剪中心范围多约 5 mm。

当前五帧桌面范围内保留体素数：

| Snapshot | 原始体素 | XY 范围内 | 最终保留 | Object 数 |
|---|---:|---:|---:|---:|
| `040712_0001` | 43,154 | 20,791 | 18,586 | 3 |
| `040725_0002` | 46,130 | 21,002 | 20,696 | 2 |
| `040817_0003` | 46,549 | 18,873 | 17,717 | 4 |
| `040844_0004` | 48,762 | 17,985 | 17,089 | 2 |
| `040911_0005` | 48,357 | 18,796 | 17,916 | 3 |

## 直接深度体素场景简化

对五组 `direct_depth_voxels.npz` 使用相同桌面范围和简化参数：

```bash
cd /home/ck/MapAnythingTest
conda run --no-capture-output -n MAP \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=Avoid \
  python Avoid/scripts/simplify_g2_snapshots.py \
  --scene-root G2/3box/direct_depth_reconstruction \
  --voxel-filename direct_depth_voxels.npz \
  --pipeline MapAnythingPipeline \
  --table-xy-bounds 0.239 1.019 -0.694 0.706
```

脚本通过 `direct_depth_manifest.json` 找回对应 undistorted snapshot 的标定相机位姿和关节状态。
每帧保留原始 `direct_depth_voxels.glb`，另外生成：

- `cleaned_voxels.glb`
- `obstacles.glb`
- `obstacles.json`

| Snapshot | 原始体素 | XY 范围内 | 最终保留 | Object 数 |
|---|---:|---:|---:|---:|
| `040712_0001` | 25,605 | 10,107 | 9,735 | 3 |
| `040725_0002` | 25,469 | 10,444 | 10,107 | 2 |
| `040817_0003` | 25,010 | 9,822 | 9,158 | 3 |
| `040844_0004` | 24,967 | 9,844 | 9,024 | 2 |
| `040911_0005` | 24,747 | 10,083 | 9,501 | 2 |

当前注册深度只有 head view，因此第三、第五帧分别比 MapAnything 简化少一个 object。这表示
遮挡物没有足够直接深度证据，不应把少检结果解释为物体确实不存在。

## 对比边界

- 原始直接深度：只有实际可见表面，无 MapAnything 补全、DBSCAN、夹爪代理删除或桌面裁剪。
- 简化直接深度：对原始直接深度应用同一套近手部过滤、DBSCAN、桌面裁剪和基本体拟合。
- MapAnything 简化：多 RGB 重建后做近手部过滤、DBSCAN、桌面裁剪和基本体拟合。
- 桌面 XY/Z 裁剪只能去掉场外背景，不能移除位于桌面范围内的机械臂。
- `obstacles.glb` 的 object 使用 8 cm 可视化膨胀；规划读取 JSON 原始尺寸，再独立应用安全裕量。
- 当前第 2 帧 raw primitive 起点无碰；其余帧仍存在机器人残留或基本体过包围，尚不能用于实机。

## 夹爪体素去除（操作者实测尺寸，2026-07-27）

模块：`avoidance/gripper_volume.py`；不使用 URDF omnipicker 几何（与实装夹爪不是同一部件），
改为把夹爪建模成锚定在**腕相机**上的长方体——腕相机外参已被真值地标重投影验证过。

操作者给定尺寸：

- 盒**中心**距相机中心 `7 cm`（即 15 cm 长度以此为中点，前后各 7.5 cm；宽度同理）；
- 长 `15 cm`（安装部位→夹爪尖）、宽 `10 cm`、高 `6 cm`；
- 方向为相机前向下俯 `45°`。

"下俯 45°" 提供两种锚定，可用 `--gripper-anchor` 选择：

| 锚定 | 定义 | 实测（MAP 体素，margin=0） |
|---|---|---|
| `optical`（默认） | 相机光轴 `+Z` 朝相机自身 `+Y`（down）旋转 45°，完全由外参决定，随腕部 roll | 0003/0004/0005 各删 165 / 267 / 170 |
| `world` | 相机前向在水平面的投影，再朝世界 `-Z`（重力）倾 45° | 各删 45 / 72 / 21 |

`optical` 明显更贴合实际夹爪（`world` 在 0005 仅删 1–21 个体素，说明腕部有 roll 时会偏）。
默认采用 `optical`。

安全裕量（`--gripper-margin`，MAP 体素删除数）：

| Snapshot | 0 cm | 1 cm | 2 cm | 3 cm | 5 cm |
|---|---:|---:|---:|---:|---:|
| `040817_0003` | 165 | 308 | 407 | 508 | 696 |
| `040844_0004` | 267 | 411 | 522 | 634 | 832 |
| `040911_0005` | 170 | 283 | 380 | 465 | 614 |

增长平缓无突跳，说明盒边界附近体素连续；默认取 `2 cm`（重建噪声会让夹爪体素略微外扩）。

这仍是**保守的去除代理**，不是已确认 TCP，不得当作夹爪运动学发布。

## 深度 + MapAnything 融合

模块：`avoidance/depth_fusion.py` + `avoidance/voxel_cleanup.py`，脚本：`scripts/fuse_depth_and_map.py`。

**融合输入是去噪并限定范围后的结果，不是原始体素。** 两路在融合前都先经过与
`simplify_g2_snapshots.py` 相同的清理：工作区（桌面 XY）裁剪 → 夹爪盒去除 → DBSCAN 去噪 →
桌面裁剪。直接融合原始体素会把背景墙、地面和散点噪声带进结果并交给规划器。

两者失效模式互补：直接深度是度量真值但只有可见表面；MapAnything 覆盖全但有残余深度误差
（头部视野 ~2 cm，仅腕部可见处 ~10 cm）。因此采用**深度优先 + 仅补空洞 + 吸附**：

1. 两者都是 `base_link` / 1 cm，但栅格 origin 不同，故先量化到**同一以原点为锚的公共格**；
2. 深度体素**全部保留**，标 `provenance=depth`；
3. MapAnything 体素只有在**距任一深度体素 ≥ `--snap-distance`（默认 3 cm）**时才被采纳为补齐，
   更近的视为深度已更准确测过的同一表面而丢弃（吸附），避免形成第二层壳；
4. 采纳者标 `provenance=map`，规划时可对该部分单独加更大安全裕量。

**相机外参一律取自深度采集**（`in/<snapshot>/camera_extrinsics.json`）。

对齐前置验证（无需配准）：97–98% 的深度体素在 MapAnything 里 10 cm 内有对应，
最近距中位 8–27 mm，系统偏移仅数毫米，故直接融合即可。

### 清理步骤（融合前，两路各做一遍）

`avoidance/voxel_cleanup.py` 的 `clean_cloud()`，顺序与 `simplify_g2_snapshots.py` 一致：

1. **工作区裁剪**：只保留实测桌面 XY 矩形内的体素（默认
   `X=[0.239,1.019]`、`Y=[-0.694,0.706]`，`--table-xy-bounds` 可改，
   `--no-workspace-crop` 可关但不建议用于规划）；
2. **夹爪去除**：切除锚定在腕相机上的操作者实测长方体（见上一节）；
3. **DBSCAN 去噪**：丢弃未聚类点和小于 `--min-cluster`（默认 24）的漂浮簇；
4. **桌面裁剪**：用高度直方图定位支撑面，删除低于 `table_top_z - --table-thickness`
   （默认 6 cm）的部分。

桌面高度检测通过 `--pipeline` 动态调用 `scene_simplify.find_support_surface`，
保持与简化流程同一套实现，不复制逻辑。

`7.24Exp` 实测（清理 → 融合，`optical`/2 cm 夹爪盒）：

| Snapshot | 深度 原始→清理后 | MapAnything 原始→清理后 | 融合 | map 补齐 | 吸附丢弃 |
|---|---|---|---:|---:|---:|
| `040817_0003` | 25,010 → 9,253 | 46,549 → 11,113 | 11,910 | 2,657 | 8,456 |
| `040844_0004` | 24,967 → 9,149 | 48,762 → 11,776 | 11,909 | 2,760 | 9,016 |
| `040911_0005` | 24,747 → 9,581 | 48,357 → 13,790 | 12,664 | 3,083 | 10,707 |

0003 帧逐级：深度 `25,010 → 9,822`(XY) `→ 9,567`(夹爪) `→ 9,445`(去噪) `→ 9,253`(桌面)；
MapAnything `46,549 → 18,873 → 18,466 → 18,419 → 11,113`。其中 XY 裁剪后的 18,873
与本文上表"XY 范围内"完全一致，可交叉核对。

> **本机与 env 机器的差异**：本文上表 MapAnything 最终保留 17,717，而此处为 11,113。
> 差异全部来自桌面裁剪：本机（旧版 pipeline）`find_support_surface` 把 `table_top_z`
> 判为 `0.6548`，env 机器（新版，含 `fit_support_box`）约为 `0.6148`，两者裁剪线差约 4 cm。
> 融合脚本是**动态调用** pipeline 的该函数，因此在 env 机器上会自动采用新版结果、与上表一致。
> 本机融合结果仍保留了完整桌面（融合后 z 直方图峰值在 `0.61`，5,449 个体素）与全部桌上物体，
> 可用；但以 env 机器的数字为准。

目视核对（侧视图）：深度真值集中在桌面顶面与物体上表面，map 补齐落在桌面下方与物体侧面/背面
——正是深度相机的遮挡区，两者互补且未形成双层壳。融合后覆盖较纯深度提升约 80–100%。

输出 `<root>/fused/<snapshot>/`：

- `fused_voxels.npz`：含 `provenance` / `provenance_names` 字段，`conf` 对深度为 1.0、map 为 0.5；
  已验证可被 `MapAnythingPipeline/scene_simplify.py` 的 `load_voxels` 直接读取；
- `fusion_report.json`：输入哈希/参数/夹爪盒定义/各类计数。

用法：

```bash
PYTHONPATH=Avoid python Avoid/scripts/fuse_depth_and_map.py \
  --root 7.24Exp \
  --pipeline MapAnythingPipeline \
  --urdf G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf \
  --tint-strength 0.85
```

目录约定为 `<root>/depth/<snap>/direct_depth_voxels.npz`、`<root>/map/<snap>/voxels.npz`、
`<root>/in/<snap>/camera_extrinsics.json`；可用 `--depth-filename` / `--map-filename` /
`--input-dirname` 调整。`--pipeline` 用于桌面高度检测（缺省找同级 `MapAnythingPipeline`
或读 `MAPANYTHING_PIPELINE`）。清理参数：`--table-xy-bounds` / `--no-workspace-crop` /
`--cluster-eps` / `--min-cluster` / `--table-thickness`。

### 融合结果 GLB

`fuse_depth_and_map.py` 默认同时导出 `fused_voxels.glb`，沿用与
`reconstruct_depth_voxels.py` / `scene_simplify.py` 相同的绕 X 轴 180° viewer transform，
因此可与 `voxels.glb`、`direct_depth_voxels.glb`、`cleaned_voxels.glb` 直接叠加比较，
并复用 `g2_glb_markers.py` 的同一套机器人参考标记。

GLB 内含：

- `fused_voxels`：体素立方体，保留采集颜色但按 provenance 着色——**绿色=深度真值，
  琥珀=MapAnything 补齐**（混合强度由 `--tint-strength` 控制，默认 0.55；
  场景本身偏暗时建议 0.85）；
- `gripper_removal_left/right`：红色半透明壳，显示被当作夹爪切除的区域（`--no-gripper-shell` 可关）；
- 全套标记：`base_link` 原点、头部/左右手相机、左右法兰参考中心与简约手。

`7.24Exp` 三帧实测 GLB 约 3.2–3.4 MB（清理后体素大幅减少）。0003 帧顶点着色校验：
绿 74,024 / 琥珀 21,256，恰为 9,253 与 2,657 个体素各 ×8 顶点，与 npz 的 `provenance` 完全一致。

```bash
PYTHONPATH=Avoid python Avoid/scripts/fuse_depth_and_map.py \
  --root 7.24Exp \
  --urdf G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf \
  --tint-strength 0.85
```

`--urdf` 用于法兰标记；本机目录结构与 `g2_glb_markers.DEFAULT_URDF` 假设的
`../G2/G2_parameters/...` 不同，故需显式指定。标记生成失败不会影响融合本身，
`fusion_report.json` 的 `outputs.glb_error` 会记录原因。`--no-glb` 可跳过导出。
