#!/usr/bin/env python3
"""Interactive G2 clustered-scene avoidance demonstrator."""

from __future__ import annotations

import argparse
import json
import queue
import subprocess
import sys
import tempfile
import threading
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
sys.path.insert(0, str(ROOT))
from avoidance.contracts import AvoidanceError, write_json
from avoidance.planning_scene import load_planning_scene

WORKER = ROOT / "scripts/g2_gui_worker.py"
DEFAULT_SCENES = WORKSPACE / "G2/expoutput3"


def configure_plot() -> None:
    import matplotlib
    matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    matplotlib.rcParams["font.size"] = 11
    matplotlib.rcParams["axes.titlesize"] = 13
    matplotlib.rcParams["axes.labelsize"] = 11
    matplotlib.rcParams["legend.fontsize"] = 10


class RobotWorker:
    def run(self, request: dict[str, Any]) -> dict[str, Any]:
        with tempfile.TemporaryDirectory(prefix="g2_gui_") as directory:
            request_path, response_path = Path(directory)/"request.json", Path(directory)/"response.json"
            write_json(request_path, request)
            result = subprocess.run(["conda","run","-n","robot","env",f"PYTHONPATH={ROOT}","python",str(WORKER),"--request",str(request_path),"--response",str(response_path)], capture_output=True, text=True, timeout=120)
            if not response_path.exists():
                raise AvoidanceError(result.stderr or "robot worker returned no response")
            response = json.loads(response_path.read_text())
            if not response.get("ok"):
                raise AvoidanceError(response.get("error", "worker failed"))
            return response


def snapshots(root: Path) -> list[Path]:
    root = root.expanduser().resolve()
    if (root/"obstacles.json").exists(): return [root]
    return [p for p in sorted(root.glob("snapshot_*")) if (p/"obstacles.json").exists() and (p/"capture_state.json").exists()]


def box_faces(center: np.ndarray, size: np.ndarray) -> list[Any]:
    half = size/2
    corners = np.asarray([center + half*[x,y,z] for x,y,z in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]])
    return [[corners[i] for i in face] for face in [(0,1,2,3),(4,5,6,7),(0,1,5,4),(2,3,7,6),(1,2,6,5),(3,0,4,7)]]


class Renderer:
    colors = {"body":"#334155","head":"#16825d","left":"#e04431","right":"#2563c7"}
    def __init__(self, figure: Any):
        self.figure, self.ax = figure, figure.add_subplot(111, projection="3d")

    def draw(self, scene: Any, skeleton: dict[str,Any] | None, centers: list[dict[str,Any]] | None = None, path: list[Any] | None = None, title: str = "") -> None:
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        from matplotlib.lines import Line2D
        self.ax.clear(); points = []
        for item in scene.primitives:
            color = np.asarray(item.color)/255
            if item.kind == "box":
                self.ax.add_collection3d(Poly3DCollection(box_faces(item.center_m,item.size_m),facecolors=[color],edgecolors="#475569",alpha=.42,linewidths=.6))
                self.ax.add_collection3d(Poly3DCollection(box_faces(item.center_m,item.size_m+.2),facecolors=(0,0,0,0),edgecolors="#111827",alpha=.3,linewidths=.45,linestyles=":"))
            else:
                theta,z=np.meshgrid(np.linspace(0,2*np.pi,28),[-item.height_m/2,item.height_m/2])
                self.ax.plot_surface(item.center_m[0]+item.radius_m*np.cos(theta),item.center_m[1]+item.radius_m*np.sin(theta),item.center_m[2]+z,color=color,alpha=.55)
            points.extend(item.bounds_m.tolist())
        for marker in scene.markers:
            if marker.identifier not in {"left_gripper","right_gripper"}:
                self.ax.scatter(*marker.center_m,c=[np.asarray(marker.color)/255],marker="x",s=22); points.append(marker.center_m)
        if centers:
            c=np.asarray([x["position_m"] for x in centers]); self.ax.scatter(c[:,0],c[:,1],c[:,2],s=7,color="#64748b",alpha=.3); points.extend(c)
        if skeleton:
            for group,items in skeleton.items():
                p=np.asarray([x["position_m"] for x in items]); self.ax.plot(p[:,0],p[:,1],p[:,2],color=self.colors[group],linewidth=3,marker="o",markersize=4); points.extend(p)
        if path:
            p=np.asarray([[m[0][3],m[1][3],m[2][3]] for m in path]); self.ax.plot(p[:,0],p[:,1],p[:,2],color="#f0b429",linewidth=3,marker="."); points.extend(p)
        p=np.asarray(points); low,high=p.min(0),p.max(0); center=(low+high)/2; radius=max(np.max(high-low)*.58,.65)
        self.ax.set_xlim(center[0]-radius,center[0]+radius); self.ax.set_ylim(center[1]-radius,center[1]+radius); self.ax.set_zlim(max(-.15,center[2]-radius),center[2]+radius); self.ax.set_box_aspect((1,1,1))
        self.ax.set(xlabel="X / m",ylabel="Y / m",zlabel="Z / m",title=title); self.ax.view_init(25,-62)
        self.ax.legend(handles=[Line2D([0],[0],color="#e04431",lw=3,label="左臂"),Line2D([0],[0],color="#2563c7",lw=3,label="右臂"),Line2D([0],[0],color="#f0b429",lw=3,label="法兰路径"),Line2D([0],[0],color="#111827",ls=":",label="10 cm 安全边界")],loc="upper right")
        self.ax.set_position([0.02, 0.02, 0.96, 0.93])


class GUI:
    def __init__(self, root: Any, scene_root: Path, *, ui_scale: float = 1.25):
        import tkinter as tk
        import tkinter.font as tkfont
        from tkinter import ttk
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
        configure_plot(); self.root=root; self.worker=RobotWorker(); self.events=queue.Queue(); self.busy=False; self.scene=None; self.desc=None; self.plan=None; self.index=0; self.playing=False
        current_scaling = float(root.tk.call("tk", "scaling"))
        root.tk.call("tk", "scaling", current_scaling * ui_scale)
        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(size=12)
        text_font = tkfont.nametofont("TkTextFont")
        text_font.configure(size=12)
        fixed_font = tkfont.nametofont("TkFixedFont")
        fixed_font.configure(size=11)
        style = ttk.Style(root)
        style.configure("TButton", padding=(11, 7))
        style.configure("TCombobox", padding=4)
        style.configure("Section.TLabel", font=(default_font.actual("family"), 13, "bold"))
        self.paths=snapshots(scene_root); self.snapshot=tk.StringVar(value=self.paths[0].name if self.paths else ""); self.arm=tk.StringVar(value="left"); self.status=tk.StringVar(value="正在加载..."); self.timeline=tk.DoubleVar(); self.goals=[tk.DoubleVar() for _ in range(7)]; self.labels=[tk.StringVar(value="0.000") for _ in range(7)]
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        window_width = max(1200, int(screen_width * 0.92))
        window_height = max(760, int(screen_height * 0.88))
        window_x = max(0, (screen_width - window_width) // 2)
        window_y = max(0, (screen_height - window_height) // 2)
        panel_width = max(380, min(520, int(screen_width * 0.13)))
        root.title("G2 场景避障演示")
        root.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")
        root.minsize(1120,720)
        def present_window() -> None:
            root.lift()
            root.attributes("-topmost", True)
            root.focus_force()
            root.after(600, lambda: root.attributes("-topmost", False))
        root.after(250, present_window)
        tk.Label(root,text="机械臂本体算法演示 | 当前夹爪模型与 TCP 未确认，gripper_* 几何已排除，结果禁止用于实机执行",bg="#9f2d20",fg="white",padx=16,pady=11,font=(default_font.actual("family"),12,"bold"),anchor="w").pack(fill="x")

        body=ttk.Frame(root)
        body.pack(fill="both",expand=True)
        controls_host=tk.Frame(body,width=panel_width,bg="#eef1f4",highlightthickness=0)
        controls_host.pack(side="left",fill="y")
        controls_host.pack_propagate(False)
        control_canvas=tk.Canvas(controls_host,bg="#eef1f4",highlightthickness=0,width=panel_width)
        scrollbar=ttk.Scrollbar(controls_host,orient="vertical",command=control_canvas.yview)
        control_canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right",fill="y")
        control_canvas.pack(side="left",fill="both",expand=True)
        controls=ttk.Frame(control_canvas,padding=16)
        controls_window=control_canvas.create_window((0,0),window=controls,anchor="nw")
        controls.bind("<Configure>",lambda _event: control_canvas.configure(scrollregion=control_canvas.bbox("all")))
        control_canvas.bind("<Configure>",lambda event: control_canvas.itemconfigure(controls_window,width=event.width))
        control_canvas.bind("<Enter>",lambda _event: control_canvas.bind_all("<MouseWheel>",lambda event: control_canvas.yview_scroll(int(-event.delta/120),"units")))
        control_canvas.bind("<Leave>",lambda _event: control_canvas.unbind_all("<MouseWheel>"))
        ttk.Separator(body,orient="vertical").pack(side="left",fill="y")
        view=ttk.Frame(body)
        view.pack(side="left",fill="both",expand=True)

        ttk.Label(controls,text="Snapshot",style="Section.TLabel").pack(anchor="w"); combo=ttk.Combobox(controls,textvariable=self.snapshot,values=[p.name for p in self.paths],state="readonly"); combo.pack(fill="x",pady=(8,10))
        row=ttk.Frame(controls); row.pack(fill="x"); ttk.Button(row,text="加载场景",command=self.load).pack(side="left",fill="x",expand=True); arm=ttk.Combobox(row,textvariable=self.arm,values=("left","right"),state="readonly",width=8); arm.pack(side="right",padx=(6,0)); arm.bind("<<ComboboxSelected>>",lambda e:self.load())
        ttk.Separator(controls).pack(fill="x",pady=16); ttk.Label(controls,text="目标关节 / rad",style="Section.TLabel").pack(anchor="w",pady=(0,6))
        self.sliders=[]
        for i in range(7):
            r=ttk.Frame(controls); r.pack(fill="x",pady=5); ttk.Label(r,text=f"J{i+1}",width=3).pack(side="left"); s=ttk.Scale(r,variable=self.goals[i],from_=-3,to=3,command=lambda v,j=i:self.labels[j].set(f"{float(v):.3f}")); s.pack(side="left",fill="x",expand=True,padx=(4,8)); ttk.Label(r,textvariable=self.labels[i],width=7,anchor="e").pack(side="right"); self.sliders.append(s)
        actions=ttk.Frame(controls); actions.pack(fill="x",pady=(14,8)); ttk.Button(actions,text="恢复起点",command=self.reset).pack(side="left"); self.plan_button=ttk.Button(actions,text="开始规划",command=self.start_plan,state="disabled"); self.plan_button.pack(side="right")
        playback=ttk.Frame(controls); playback.pack(fill="x",pady=8); self.play_button=ttk.Button(playback,text="播放",command=self.toggle,state="disabled"); self.play_button.pack(side="left"); self.time=ttk.Scale(playback,variable=self.timeline,from_=0,to=0,state="disabled",command=self.seek); self.time.pack(side="left",fill="x",expand=True,padx=(10,0))
        ttk.Label(controls,text="诊断",style="Section.TLabel").pack(anchor="w",pady=(16,6))
        self.status_label=tk.Label(controls,textvariable=self.status,bg="white",fg="#263238",padx=12,pady=12,justify="left",anchor="nw",wraplength=panel_width-70,relief="solid",borderwidth=1,font=(default_font.actual("family"),11))
        self.status_label.pack(fill="x")
        fig=Figure(figsize=(10,8),dpi=100,facecolor="#f8fafc"); self.renderer=Renderer(fig); self.canvas=FigureCanvasTkAgg(fig,master=view); toolbar=NavigationToolbar2Tk(self.canvas,view,pack_toolbar=False); toolbar.pack(fill="x"); self.canvas.get_tk_widget().pack(fill="both",expand=True)
        root.after(100,self.load); root.after(80,self.poll)

    def selected(self) -> Path:
        return next(p for p in self.paths if p.name==self.snapshot.get())
    def async_run(self, action: str, request: dict[str,Any]) -> None:
        self.busy=True
        def task():
            try:self.events.put((action,self.worker.run(request)))
            except Exception as exc:self.events.put(("error",str(exc)))
        threading.Thread(target=task,daemon=True).start()
    def load(self) -> None:
        if self.busy:return
        path=self.selected(); self.scene=load_planning_scene(path); self.renderer.draw(self.scene,None,title=path.name+" | 加载机器人"); self.canvas.draw_idle(); self.status.set("正在加载 G2 并检查起点...")
        self.async_run("describe",{"action":"describe","scene":str(path),"capture_state":str(path/"capture_state.json"),"arm":self.arm.get()})
    def start_plan(self) -> None:
        if self.busy or not self.desc:return
        path=self.selected(); self.status.set("正在运行 RRT-Connect 与稠密碰撞复检...")
        self.async_run("plan",{"action":"plan","scene":str(path),"capture_state":str(path/"capture_state.json"),"arm":self.arm.get(),"goal_arm":[v.get() for v in self.goals]})
    def poll(self) -> None:
        try:
            while True:
                action,data=self.events.get_nowait(); self.busy=False
                if action=="describe":
                    self.desc=data; self.reset(); self.plan_button.config(state="normal"); c=data["start_collision"]; self.status.set(f"起点：{'通过' if c['valid'] else '碰撞'}\n最小环境净距：{c['minimum_environment_clearance_m']:.3f} m\n排除错误末端几何：{len(data['collision_policy']['ignored_collision_geometries'])}\n实机执行：禁止"); self.renderer.draw(self.scene,data["skeleton"],data["collision_geometry_centers"],title=self.selected().name+" | 起点"); self.canvas.draw_idle()
                elif action=="plan": self.apply_plan(data)
                else:self.status.set("失败："+data)
        except queue.Empty:pass
        self.root.after(80,self.poll)
    def reset(self) -> None:
        if not self.desc:return
        for i,value in enumerate(self.desc["start_arm_joint_positions_rad"]): self.sliders[i].config(from_=self.desc["arm_lower_limits_rad"][i],to=self.desc["arm_upper_limits_rad"][i]); self.goals[i].set(value); self.labels[i].set(f"{value:.3f}")
    def apply_plan(self,data:dict[str,Any])->None:
        self.plan=data; m=data["manifest"]
        if m["status"]!="demo_planned":self.status.set(f"规划未完成\n{m['status']} / {m['reason']}");return
        self.index=0; count=len(data["skeleton_path"]); self.time.config(to=count-1,state="normal"); self.play_button.config(state="normal"); p=m["planner"]; self.status.set(f"算法演示规划完成\n路径点：{count}\nRRT：{p['reason']}\n碰撞查询：{p['collision_checks']}\n最小环境净距：{m['path']['minimum_environment_clearance_m']:.3f} m\n夹爪净距：未验证\n实机执行：禁止"); self.draw_index(0)
    def draw_index(self,index:int)->None:
        self.index=max(0,min(index,len(self.plan["skeleton_path"])-1)); m=self.plan["manifest"]; self.renderer.draw(self.scene,self.plan["skeleton_path"][self.index],self.plan["collision_geometry_centers_path"][self.index],m["path"]["base_T_tracked_frame"],f"{self.selected().name} | 路径点 {self.index+1}/{len(self.plan['skeleton_path'])}"); self.canvas.draw_idle()
    def seek(self,value:str)->None:
        if self.plan:self.draw_index(int(round(float(value))))
    def toggle(self)->None:
        self.playing=not self.playing; self.play_button.config(text="暂停" if self.playing else "播放")
        if self.playing:self.step()
    def step(self)->None:
        if not self.playing:return
        if self.index+1>=len(self.plan["skeleton_path"]):self.playing=False;self.play_button.config(text="播放");return
        self.timeline.set(self.index+1);self.draw_index(self.index+1);self.root.after(140,self.step)


def smoke(root: Path) -> int:
    import matplotlib; matplotlib.use("Agg"); configure_plot()
    from matplotlib.figure import Figure
    path=snapshots(root)[0]; scene=load_planning_scene(path); worker=RobotWorker(); base={"scene":str(path),"capture_state":str(path/"capture_state.json"),"arm":"left"}
    desc=worker.run({"action":"describe",**base}); goal=np.asarray(desc["start_arm_joint_positions_rad"]);goal[0]+=.03; plan=worker.run({"action":"plan",**base,"goal_arm":goal.tolist()}); manifest=plan["manifest"]
    if manifest["status"]!="demo_planned":raise AvoidanceError(str(manifest))
    fig=Figure(figsize=(13,8),dpi=120); Renderer(fig).draw(scene,plan["skeleton_path"][-1],plan["collision_geometry_centers_path"][-1],manifest["path"]["base_T_tracked_frame"],path.name+" | GUI smoke test")
    output=ROOT/"reports/avoidance_gui_smoke.png";output.parent.mkdir(exist_ok=True);fig.savefig(output,bbox_inches="tight")
    write_json(ROOT/"reports/avoidance_gui_smoke.json",{"status":"passed","mode":"arm_body_demo","execution_authorized":False,"snapshot":path.name,"plan_status":manifest["status"],"planner_reason":manifest["planner"]["reason"],"waypoint_count":len(plan["skeleton_path"]),"minimum_environment_clearance_m":manifest["path"]["minimum_environment_clearance_m"],"ignored_collision_geometries":manifest["collision_policy"]["ignored_collision_geometries"],"screenshot":str(output)})
    print("GUI smoke test passed");return 0


def main()->int:
    p=argparse.ArgumentParser();p.add_argument("--scene-root",type=Path,default=DEFAULT_SCENES);p.add_argument("--smoke-test",action="store_true");p.add_argument("--ui-scale",type=float,default=1.25);args=p.parse_args()
    if args.smoke_test:return smoke(args.scene_root)
    if not 0.8 <= args.ui_scale <= 2.0:
        raise AvoidanceError("--ui-scale must be between 0.8 and 2.0")
    import tkinter as tk
    root=tk.Tk();GUI(root,args.scene_root,ui_scale=args.ui_scale);root.mainloop();return 0
if __name__=="__main__":raise SystemExit(main())
