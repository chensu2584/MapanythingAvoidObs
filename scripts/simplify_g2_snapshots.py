#!/usr/bin/env python3
"""Adapt a G2 reconstruction into an Avoid planning scene (plan section 11, M1).

``MapAnythingPipeline/scene_simplify.py`` cleans a ``voxels.npz`` (spatial
gripper removal, DBSCAN denoise, Z-histogram table detection, per-cluster AABB
boxes) but writes its own ``obstacles.json`` schema.  ``avoidance.planning_scene``
needs each primitive tagged with ``primitive`` (box/cylinder) and ``role``
(support/object).  This script reuses the pipeline's tested geometry and adds
that classification layer so the avoidance GUI/planner can load the 3box data:

  * the table slab becomes a single ``support`` box (largest cluster at the
    detected table height);
  * every cluster standing above the table becomes an ``object`` box.

It stays deliberately box-only for the first version (the GUI edge-picking is
box-only too); cylinder fitting is left for later.  ``box_inflation_m`` is written
as a visualisation hint only -- planning applies its own inflation and must not
double-count it (plan section 9, phase C).

Reuses the pipeline by importing it; point ``--pipeline`` at the checkout or set
``MAPANYTHING_PIPELINE`` if the default sibling path does not apply.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
from sklearn.cluster import DBSCAN

SCRIPT_DIR = Path(__file__).resolve().parent
AVOID_ROOT = SCRIPT_DIR.parent
if str(AVOID_ROOT) not in sys.path:
    sys.path.insert(0, str(AVOID_ROOT))

from avoidance.contracts import AvoidanceError, sha256_file, write_json  # noqa: E402
from avoidance.planning_scene import load_planning_scene  # noqa: E402


def _import_pipeline(explicit: str | None):
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("MAPANYTHING_PIPELINE"):
        candidates.append(Path(os.environ["MAPANYTHING_PIPELINE"]))
    candidates.append(AVOID_ROOT.parent / "MapAnythingPipeline")
    for path in candidates:
        if (path / "scene_simplify.py").is_file():
            sys.path.insert(0, str(path))
            import scene_simplify  # type: ignore
            return scene_simplify
    raise AvoidanceError(
        "cannot find scene_simplify.py; pass --pipeline or set MAPANYTHING_PIPELINE"
    )


def _aabb(points: np.ndarray, colors: np.ndarray):
    lo, hi = points.min(0), points.max(0)
    return (lo + hi) / 2, (hi - lo), [int(v) for v in colors.mean(0)]


def support_box(world, colors, table_top_z, table_thickness, cluster_eps, min_cluster):
    """Largest horizontal cluster at the table height -> one support box."""
    slab = (world[:, 2] >= table_top_z - table_thickness) & (world[:, 2] <= table_top_z + 0.01)
    if slab.sum() < min_cluster:
        return None
    pts, cols = world[slab], colors[slab]
    labels = DBSCAN(eps=cluster_eps, min_samples=3).fit_predict(pts[:, :2])
    best, best_n = None, 0
    for lab in set(labels):
        if lab < 0:
            continue
        m = labels == lab
        if m.sum() > best_n:
            best_n, best = m.sum(), m
    if best is None:
        return None
    center, size, color = _aabb(pts[best], cols[best])
    size = np.maximum(size, 0.01)
    return {"center_m": center, "size_m": size, "color": color, "voxel_count": int(best_n)}


def simplify_snapshot(pipeline, voxels_path: Path, extrinsics_path: Path, out_dir: Path,
                      *, gripper_radius, min_count, cluster_eps, min_cluster,
                      obstacle_height, table_thickness, box_inflate, capture_state=None):
    world, colors, conf, counts, source_views, meta = pipeline.load_voxels(str(voxels_path))
    if meta["world_frame"] != "base_link":
        raise AvoidanceError(f"{voxels_path} is not base_link")
    keep = np.ones(len(world), bool)
    centers, head_z = pipeline.read_camera_centers(str(extrinsics_path))
    keep &= pipeline.remove_gripper(world, centers, gripper_radius)
    if min_count > 1:
        keep &= counts >= min_count
    # DBSCAN denoise (drop tiny floating clusters), reusing the pipeline's rule
    idx = np.where(keep)[0]
    labels = DBSCAN(eps=cluster_eps, min_samples=2).fit_predict(world[idx])
    for lab in set(labels):
        m = labels == lab
        if lab < 0 or m.sum() < min_cluster:
            keep[idx[m]] = False
    W, C = world[keep], colors[keep]

    band = (head_z - 1.0, head_z - 0.4) if head_z is not None else None
    table_top_z, mode_z, _ = pipeline.find_support_surface(W, z_band=band)

    primitives = []
    support = support_box(W, C, table_top_z, table_thickness, cluster_eps, min_cluster)
    if support is not None:
        primitives.append({"id": 0, "primitive": "box", "role": "support",
                           "center_m": support["center_m"].round(4).tolist(),
                           "size_m": support["size_m"].round(4).tolist(),
                           "color": support["color"], "voxel_count": support["voxel_count"]})

    obstacle_mask = W[:, 2] > table_top_z + obstacle_height
    boxes, _ = pipeline.axis_aligned_boxes(W[obstacle_mask], C[obstacle_mask],
                                           eps=cluster_eps, min_samples=2, min_cluster=min_cluster)
    for b in boxes:
        primitives.append({"id": len(primitives), "primitive": "box", "role": "object",
                           "center_m": b["center_m"], "size_m": b["size_m"],
                           "color": b["color"], "voxel_count": b["voxel_count"]})

    provenance = {"voxels_sha256": sha256_file(voxels_path),
                  "extrinsics_sha256": sha256_file(extrinsics_path),
                  "table_top_z_m": round(float(table_top_z), 4),
                  "kept_voxels": int(keep.sum())}
    if capture_state and Path(capture_state).is_file():
        provenance["capture_state_sha256"] = sha256_file(capture_state)

    document = {"world_frame": "base_link", "unit": "meter",
                "box_inflation_m": float(box_inflate),  # visualisation hint only
                "boxes": primitives, "markers": [], "provenance": provenance}
    out_dir.mkdir(parents=True, exist_ok=True)
    obstacles_path = out_dir / "obstacles.json"
    write_json(obstacles_path, document)
    # fail closed: the output must satisfy the planning-scene contract
    scene = load_planning_scene(obstacles_path)
    return scene, document


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene-root", type=Path, required=True,
                    help="folder holding snapshot_* reconstruction dirs")
    ap.add_argument("--pipeline", type=Path, default=None,
                    help="MapAnythingPipeline checkout (else sibling / MAPANYTHING_PIPELINE)")
    ap.add_argument("--gripper-radius", type=float, default=0.30)
    ap.add_argument("--min-count", type=int, default=1)
    ap.add_argument("--cluster-eps", type=float, default=0.03)
    ap.add_argument("--min-cluster", type=int, default=24)
    ap.add_argument("--obstacle-height", type=float, default=0.03)
    ap.add_argument("--table-thickness", type=float, default=0.06)
    ap.add_argument("--box-inflate", type=float, default=0.08)
    args = ap.parse_args()
    try:
        pipeline = _import_pipeline(str(args.pipeline) if args.pipeline else None)
        snaps = sorted(p for p in args.scene_root.iterdir()
                       if p.is_dir() and (p / "voxels.npz").is_file() and not p.name.endswith("_input"))
        if not snaps:
            raise AvoidanceError(f"no snapshot_*/voxels.npz under {args.scene_root}")
        for snap in snaps:
            ext = snap.parent / f"{snap.name}_input" / "camera_extrinsics.json"
            if not ext.is_file():
                raise AvoidanceError(f"missing raw extrinsics for {snap.name}: {ext}")
            scene, doc = simplify_snapshot(
                pipeline, snap / "voxels.npz", ext, snap,
                gripper_radius=args.gripper_radius, min_count=args.min_count,
                cluster_eps=args.cluster_eps, min_cluster=args.min_cluster,
                obstacle_height=args.obstacle_height, table_thickness=args.table_thickness,
                box_inflate=args.box_inflate,
                capture_state=snap / "capture_state.json")
            roles = [p["role"] for p in doc["boxes"]]
            print(f"{snap.name}: {len(doc['boxes'])} primitives "
                  f"(support={roles.count('support')}, object={roles.count('object')}) "
                  f"-> {snap / 'obstacles.json'}")
        return 0
    except (AvoidanceError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
