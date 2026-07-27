#!/usr/bin/env python3
"""Tk GUI for G2 head/hand MapAnything captures.

This is the G2 counterpart to capture/scripts/g1_capture_gui.py. It keeps the
same operator flow: connect, preview cameras, choose an output session folder,
then press Snapshot.  Snapshot writing and G2 FK/extrinsic composition are
delegated to g2_capture_session.py so the GUI and CLI use one calibration path.
"""

from __future__ import annotations

import argparse
import importlib.util
import queue
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

try:
    import cv2
except ImportError:  # pragma: no cover - runtime dependency
    cv2 = None

try:
    import numpy as np
except ImportError:  # pragma: no cover - runtime dependency
    np = None

try:
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import filedialog, messagebox, ttk
except ImportError:  # pragma: no cover - system dependency
    tk = None
    tkfont = None
    filedialog = None
    messagebox = None
    ttk = None

try:
    from PIL import Image, ImageTk
except ImportError:  # pragma: no cover - optional dependency
    Image = None
    ImageTk = None


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
G2_ROOT = PROJECT_ROOT / "G2"
DEFAULT_SESSION_ROOT = G2_ROOT
PREVIEW_CAMERAS = ("head", "hand_left", "hand_right")
SNAPSHOT_CAMERA_NAMES = ("head", "head_depth", "hand_left", "hand_right")


def load_capture_module() -> Any:
    path = SCRIPT_DIR / "g2_capture_session.py"
    spec = importlib.util.spec_from_file_location("g2_capture_session_gui_backend", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not import G2 capture backend: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


g2_backend = load_capture_module()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g2-root", type=Path, default=G2_ROOT)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_SESSION_ROOT)
    parser.add_argument("--capture-hand-depth", action="store_true")
    parser.add_argument("--allow-missing-images", action="store_true")
    parser.add_argument("--allow-missing-live-joints", action="store_true")
    parser.add_argument("--body-joints-json", type=str, default="")
    parser.add_argument("--timeout-ms", type=float, default=1000.0)
    parser.add_argument("--sync-threshold-ms", type=float, default=50.0)
    parser.add_argument("--discovery-timeout-sec", type=float, default=2.0)
    parser.add_argument("--skip-discovery-check", action="store_true")
    parser.add_argument("--no-dds-env", action="store_true")
    parser.add_argument("--preview-fps", type=float, default=8.0)
    parser.add_argument("--ui-font-size", type=int, default=14)
    parser.add_argument("--ui-scale", type=float, default=1.5)
    parser.add_argument("--snapshot-once", action="store_true")
    return parser.parse_args(argv)


def default_session_dir(output_root: Path) -> Path:
    return output_root / f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def require_numpy() -> None:
    if np is None:
        raise SystemExit("numpy is required for G2 capture GUI.")


def require_tkinter() -> None:
    if tk is None or ttk is None or filedialog is None or messagebox is None:
        raise SystemExit("tkinter is required for GUI mode. Use --snapshot-once on headless machines.")


def make_backend_args(args: argparse.Namespace) -> Any:
    return SimpleNamespace(
        g2_root=args.g2_root.expanduser().resolve(),
        timeout_ms=float(args.timeout_ms),
        sync_threshold_ms=float(args.sync_threshold_ms),
        capture_hand_depth=bool(args.capture_hand_depth),
        allow_missing_images=bool(args.allow_missing_images),
        allow_missing_live_joints=bool(args.allow_missing_live_joints),
    )


def bgr_to_rgb(image: Any) -> Any:
    if cv2 is None:
        return image
    array = np.asarray(image)
    if array.ndim == 3 and array.shape[-1] == 3:
        return cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
    return image


def ensure_rgb_uint8(image: Any, label: str) -> Any:
    if np is None:
        raise RuntimeError("numpy is required")
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[-1] != 3:
        raise RuntimeError(f"{label} preview must be HxWx3, got {array.shape}")
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(array)


def make_preview_photo(image: Any, max_size: tuple[int, int]) -> Any | None:
    if Image is None or ImageTk is None:
        return None
    rgb = ensure_rgb_uint8(image, "preview")
    pil = Image.fromarray(rgb)
    pil.thumbnail(max_size)
    return ImageTk.PhotoImage(pil)


class G2LiveClient:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.camera: Any | None = None
        self.robot: Any | None = None
        self.kin: Any | None = None
        self.sensor_dir: Path | None = None
        self.urdf: Path | None = None
        self.body_joints: dict[str, float] | None = None
        self.snapshot_index = 0

    def connect(self) -> None:
        if not self.args.no_dds_env:
            g2_backend.setup_dds_env()
        if not self.args.skip_discovery_check:
            g2_backend.check_discovery_endpoint(float(self.args.discovery_timeout_sec))
        from a2d_sdk.robot import CosineCamera, RobotDds

        g2_root = self.args.g2_root.expanduser().resolve()
        self.sensor_dir = g2_root / "G2_parameters/sensor"
        self.urdf = g2_root / "G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf"
        self.body_joints = g2_backend.load_body_joints(self.args.body_joints_json)
        self.kin = g2_backend.G2Kinematics(self.urdf)
        camera_names = list(SNAPSHOT_CAMERA_NAMES)
        if self.args.capture_hand_depth:
            camera_names += ["hand_left_depth", "hand_right_depth"]
        self.camera = CosineCamera(camera_names)
        self.robot = RobotDds()

    def close(self) -> None:
        if self.camera is not None:
            self.camera.close()
        self.camera = None
        if self.robot is not None:
            self.robot.shutdown()
        self.robot = None

    def get_preview_images(self) -> dict[str, Any]:
        if self.camera is None:
            raise RuntimeError("G2 client is not connected")
        images = {}
        for name in PREVIEW_CAMERAS:
            image, timestamp_ns = self.camera.get_latest_image(name)
            if image is None or timestamp_ns is None:
                raise RuntimeError(f"No live frame for {name}")
            images[name] = bgr_to_rgb(image)
        return images

    def snapshot(self, session_dir: Path) -> Path:
        if (
            self.camera is None
            or self.robot is None
            or self.kin is None
            or self.sensor_dir is None
            or self.urdf is None
            or self.body_joints is None
        ):
            raise RuntimeError("G2 client is not connected")
        self.snapshot_index += 1
        tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        snapshot_dir = session_dir / f"snapshot_{tag}_{self.snapshot_index:04d}"
        snapshot_dir.mkdir(parents=True, exist_ok=False)
        return g2_backend.capture_one(
            self.camera,
            self.robot,
            snapshot_dir,
            make_backend_args(self.args),
            self.kin,
            self.sensor_dir,
            self.urdf,
            self.body_joints,
        )


class G2CaptureGui:
    def __init__(self, root: Any, args: argparse.Namespace, client: G2LiveClient):
        self.root = root
        self.args = args
        self.client = client
        self.connected = False
        self.running = True
        self.client_lock = threading.Lock()
        self.frame_queue: queue.Queue[tuple[str, Any]] = queue.Queue(maxsize=20)
        self.photo_refs: dict[str, Any] = {}
        self.preview_labels: dict[str, Any] = {}
        self.status_var = tk.StringVar(value="Disconnected")
        self.session_dir_var = tk.StringVar(
            value=str(args.session_dir or default_session_dir(args.output_root))
        )
        self.options_var = tk.StringVar(value=self._options_text())
        self._build_ui()
        self._schedule_queue_poll()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        self.root.title("G2 MapAnything Capture")
        self.root.geometry("1320x780")
        self.root.minsize(1050, 650)
        self._configure_style()
        self.root.columnconfigure(0, weight=0, minsize=380)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)

        controls = ttk.Frame(self.root, padding=18, style="Panel.TFrame")
        controls.grid(row=0, column=0, sticky="nsew")
        controls.columnconfigure(0, weight=1)
        ttk.Label(controls, text="G2 Capture", style="Title.TLabel").grid(
            row=0, column=0, sticky="ew", pady=(0, 16)
        )
        ttk.Button(controls, text="Connect to G2", command=self.connect, style="Primary.TButton").grid(
            row=1, column=0, sticky="ew", pady=(0, 10)
        )
        ttk.Button(controls, text="Snapshot", command=self.snapshot, style="Primary.TButton").grid(
            row=2, column=0, sticky="ew", pady=(0, 18)
        )
        ttk.Label(controls, text="Session Folder", style="FieldLabel.TLabel").grid(
            row=3, column=0, sticky="w"
        )
        ttk.Entry(controls, textvariable=self.session_dir_var, font=self.entry_font).grid(
            row=4, column=0, sticky="ew", pady=(6, 10), ipady=5
        )
        ttk.Button(controls, text="Choose Session Folder...", command=self.choose_session_dir).grid(
            row=5, column=0, sticky="ew", pady=(0, 16)
        )
        ttk.Label(
            controls,
            textvariable=self.options_var,
            style="Info.TLabel",
            justify=tk.LEFT,
            wraplength=330,
        ).grid(row=6, column=0, sticky="ew")

        preview = ttk.Frame(self.root, padding=(14, 14, 16, 14))
        preview.grid(row=0, column=1, sticky="nsew")
        preview.rowconfigure(0, weight=1)
        for index, name in enumerate(PREVIEW_CAMERAS):
            preview.columnconfigure(index, weight=1, uniform="camera")
            cell = ttk.Frame(preview, padding=10, style="Preview.TFrame")
            cell.grid(row=0, column=index, sticky="nsew", padx=(0 if index == 0 else 8, 0))
            cell.columnconfigure(0, weight=1)
            cell.rowconfigure(1, weight=1)
            title = {"head": "Head RGB", "hand_left": "Left Hand RGB", "hand_right": "Right Hand RGB"}[name]
            ttk.Label(cell, text=title, style="CameraTitle.TLabel").grid(
                row=0, column=0, sticky="ew", pady=(0, 8)
            )
            label = tk.Label(
                cell,
                bg="#101216",
                fg="#d8dee9",
                text="No frame",
                font=self.preview_font,
                anchor=tk.CENTER,
                bd=0,
                highlightthickness=1,
                highlightbackground="#303640",
            )
            label.grid(row=1, column=0, sticky="nsew")
            self.preview_labels[name] = label

        status = ttk.Frame(self.root, padding=(16, 8))
        status.grid(row=1, column=0, columnspan=2, sticky="ew")
        status.columnconfigure(0, weight=1)
        ttk.Label(status, textvariable=self.status_var, style="Status.TLabel", wraplength=1000).grid(
            row=0, column=0, sticky="w"
        )

    def _configure_style(self) -> None:
        base_size = max(12, int(self.args.ui_font_size))
        self.root.tk.call("tk", "scaling", max(1.0, float(self.args.ui_scale)))
        family = "DejaVu Sans"
        if tkfont is not None:
            available = set(tkfont.families())
            if family not in available:
                family = str(tkfont.nametofont("TkDefaultFont").actual("family"))
        for font_name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            try:
                tkfont.nametofont(font_name).configure(family=family, size=base_size)
            except tk.TclError:
                pass
        self.entry_font = (family, base_size + 1)
        self.preview_font = (family, base_size + 2)
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure(".", font=(family, base_size))
        style.configure("Title.TLabel", font=(family, base_size + 8, "bold"))
        style.configure("CameraTitle.TLabel", font=(family, base_size + 4, "bold"))
        style.configure("FieldLabel.TLabel", font=(family, base_size, "bold"))
        style.configure("Info.TLabel", foreground="#374151")
        style.configure("Status.TLabel", font=(family, base_size))
        style.configure("TButton", font=(family, base_size + 1), padding=(12, 9))
        style.configure("Primary.TButton", font=(family, base_size + 2, "bold"), padding=(14, 11))
        style.configure("Panel.TFrame", background="#f4f6f8")
        style.configure("Preview.TFrame", relief="solid", borderwidth=1)

    def _options_text(self) -> str:
        return (
            f"G2 root: {self.args.g2_root}\n"
            f"Timeout: {self.args.timeout_ms:g} ms\n"
            f"Sync threshold: {self.args.sync_threshold_ms:g} ms\n"
            f"Hand depth: {'on' if self.args.capture_hand_depth else 'off'}\n"
            "Snapshot output is native G2 session format."
        )

    def connect(self) -> None:
        if self.connected:
            self.set_status("Already connected")
            return
        try:
            with self.client_lock:
                self.client.connect()
            self.connected = True
        except Exception as exc:
            self.show_error("Connect failed", exc)
            return
        self.set_status("Connected")
        threading.Thread(target=self._frame_loop, name="g2-preview", daemon=True).start()

    def choose_session_dir(self) -> None:
        initial = Path(self.session_dir_var.get()).expanduser()
        if not initial.exists():
            initial = initial.parent if initial.parent.exists() else Path.cwd()
        selected = filedialog.askdirectory(
            title="Choose or create parent session folder",
            initialdir=str(initial),
            mustexist=False,
            parent=self.root,
        )
        if selected:
            self.session_dir_var.set(selected)

    def snapshot(self) -> None:
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to G2 first.", parent=self.root)
            return
        session_dir = Path(self.session_dir_var.get()).expanduser()
        try:
            with self.client_lock:
                saved = self.client.snapshot(session_dir)
        except Exception as exc:
            self.show_error("Snapshot failed", exc)
            return
        self.set_status(f"Saved {saved}")

    def _frame_loop(self) -> None:
        interval = 1.0 / max(float(self.args.preview_fps), 0.1)
        while self.running and self.connected:
            try:
                with self.client_lock:
                    images = self.client.get_preview_images()
                self._put_queue(("frames", images))
            except Exception:
                self._put_queue(("error", traceback.format_exc()))
                time.sleep(1.0)
            time.sleep(interval)

    def _put_queue(self, item: tuple[str, Any]) -> None:
        try:
            self.frame_queue.put_nowait(item)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.frame_queue.put_nowait(item)

    def _schedule_queue_poll(self) -> None:
        try:
            while True:
                kind, payload = self.frame_queue.get_nowait()
                if kind == "frames":
                    self._update_previews(payload)
                elif kind == "error":
                    self.set_status(f"Preview error: {payload.strip().splitlines()[-1]}")
        except queue.Empty:
            pass
        if self.running:
            self.root.after(50, self._schedule_queue_poll)

    def _update_previews(self, images: dict[str, Any]) -> None:
        for name, image in images.items():
            label = self.preview_labels[name]
            max_width = max(label.winfo_width() - 16, 180)
            max_height = max(label.winfo_height() - 16, 140)
            photo = make_preview_photo(image, (max_width, max_height))
            if photo is None:
                label.configure(text=f"{name}\n{np.asarray(image).shape}", image="")
                continue
            self.photo_refs[name] = photo
            label.configure(image=photo, text="")

    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def show_error(self, title: str, exc: Exception) -> None:
        details = traceback.format_exc()
        self.set_status(f"{title}: {exc}")
        messagebox.showerror(title, f"{exc}\n\n{details}", parent=self.root)

    def on_close(self) -> None:
        self.running = False
        try:
            with self.client_lock:
                self.client.close()
        finally:
            self.root.destroy()


def snapshot_once(args: argparse.Namespace) -> Path:
    client = G2LiveClient(args)
    session_dir = args.session_dir or default_session_dir(args.output_root)
    try:
        client.connect()
        return client.snapshot(session_dir)
    finally:
        client.close()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    require_numpy()
    if args.snapshot_once:
        saved = snapshot_once(args)
        print(saved)
        return 0
    require_tkinter()
    root = tk.Tk()
    G2CaptureGui(root, args, G2LiveClient(args))
    root.mainloop()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
