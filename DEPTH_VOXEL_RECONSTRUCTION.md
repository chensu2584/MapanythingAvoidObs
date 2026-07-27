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
