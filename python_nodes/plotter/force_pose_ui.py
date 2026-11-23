"""
Tkinter front-end for the force / pose experiment viewer.

    python force_pose_ui.py [files or folders ...]

Needs force_pose_core.py in the same folder.
"""
from __future__ import annotations

import sys
import traceback
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from force_pose_core import (
    KINDS, DataStore, SubplotSpec, TraceSpec, axis_options, build_figure,
    exp_sort_key, export_figure, read_description,
)

DEFAULT_X = "pose_data: time_s"
DEFAULT_Y = "force_data: force_z"


# --------------------------------------------------------------------------- #
# Column picker popup
# --------------------------------------------------------------------------- #
class AxisPicker(tk.Toplevel):
    """Modal popup: filter box + tree of sources -> columns."""

    def __init__(self, anchor: tk.Widget, options: list[str], current: str):
        super().__init__(anchor)
        self.title("Select column")
        self.transient(anchor.winfo_toplevel())
        self.options, self.current = options, current
        self.result: str | None = None
        self._labels: dict[str, str] = {}  # tree iid -> "source: column"
        self.filter_var = tk.StringVar()

        top = ttk.Frame(self, padding=(8, 8, 8, 4))
        top.pack(fill="x")
        ttk.Label(top, text="Filter:").pack(side="left")
        entry = ttk.Entry(top, textvariable=self.filter_var)
        entry.pack(side="left", fill="x", expand=True, padx=(4, 0))
        entry.focus_set()

        mid = ttk.Frame(self, padding=(8, 0))
        mid.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(mid, show="tree", selectmode="browse", height=18)
        vsb = ttk.Scrollbar(mid, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")

        bottom = ttk.Frame(self, padding=8)
        bottom.pack(fill="x")
        ttk.Button(bottom, text="Cancel", command=self.destroy).pack(side="right")
        ttk.Button(bottom, text="OK", command=self._ok).pack(side="right", padx=6)

        self.filter_var.trace_add("write", lambda *_: self._rebuild())
        self.tree.bind("<Double-1>", self._on_double)
        self.bind("<Return>", lambda e: self._ok())
        self.bind("<Escape>", lambda e: self.destroy())
        self._rebuild(initial=True)

        self.update_idletasks()
        x = anchor.winfo_rootx()
        y = min(anchor.winfo_rooty() + anchor.winfo_height(), max(self.winfo_screenheight() - 520, 0))
        self.geometry(f"380x480+{x}+{y}")
        self.wait_visibility()
        self.grab_set()

    def _rebuild(self, initial: bool = False):
        words = self.filter_var.get().lower().split()
        self.tree.delete(*self.tree.get_children())
        self._labels.clear()
        groups: dict[str, list[str]] = {}
        for label in self.options:
            if all(w in label.lower() for w in words):
                groups.setdefault(label.split(": ", 1)[0], []).append(label)
        current_src = self.current.split(": ", 1)[0] if self.current else None
        for gi, (src, labels) in enumerate(groups.items()):
            parent = self.tree.insert("", "end", iid=f"s{gi}", text=f"{src}  ({len(labels)})",
                                      open=bool(words) or (initial and src == current_src))
            for li, label in enumerate(labels):
                iid = f"s{gi}_{li}"
                self.tree.insert(parent, "end", iid=iid, text=label.split(": ", 1)[1])
                self._labels[iid] = label
                if initial and label == self.current:
                    self.tree.selection_set(iid)
                    self.tree.see(iid)
        if words and self._labels:
            self.tree.selection_set(next(iter(self._labels)))

    def _on_double(self, event):
        iid = self.tree.identify_row(event.y)
        if iid in self._labels:
            self.tree.selection_set(iid)
            self._ok()

    def _ok(self):
        sel = self.tree.selection()
        if sel and sel[0] in self._labels:
            self.result = self._labels[sel[0]]
            self.destroy()


def pick_column(anchor: tk.Widget, options: list[str], current: str) -> str | None:
    dlg = AxisPicker(anchor, options, current)
    anchor.wait_window(dlg)
    return dlg.result


# --------------------------------------------------------------------------- #
# Small widgets
# --------------------------------------------------------------------------- #
class ScrollFrame(ttk.Frame):
    """Vertically scrollable frame; put children into `.inner`."""

    def __init__(self, master):
        super().__init__(master)
        self.canvas = tk.Canvas(self, highlightthickness=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)
        self.inner.bind("<Configure>",
                        lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self._win = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self._win, width=e.width))
        self.canvas.configure(yscrollcommand=vsb.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.bind_all(seq, self._on_wheel, add="+")

    def _inside(self, widget):
        while widget is not None:
            if widget == self:
                return True
            widget = getattr(widget, "master", None)
        return False

    def _on_wheel(self, e):
        try:
            target = self.winfo_containing(e.x_root, e.y_root)
        except KeyError:
            return
        if not self._inside(target):
            return
        if e.num == 4:
            step = -1
        elif e.num == 5:
            step = 1
        else:
            step = -1 if e.delta > 0 else 1
        self.canvas.yview_scroll(step, "units")


class TraceRow(ttk.Frame):
    """One trace: x column, y column (both chosen in a popup) and y-axis number."""

    def __init__(self, master, options, on_remove, x, y, axis):
        super().__init__(master)
        self.options = options
        self.x_var, self.y_var = tk.StringVar(value=x), tk.StringVar(value=y)
        self.axis_var = tk.StringVar(value=str(axis))
        ttk.Label(self, text="x:").pack(side="left")
        ttk.Button(self, textvariable=self.x_var, width=32,
                   command=lambda: self._pick(self.x_var)).pack(side="left", padx=(2, 10))
        ttk.Label(self, text="y:").pack(side="left")
        ttk.Button(self, textvariable=self.y_var, width=32,
                   command=lambda: self._pick(self.y_var)).pack(side="left", padx=(2, 10))
        ttk.Label(self, text="y-axis #:").pack(side="left")
        ttk.Spinbox(self, from_=1, to=8, width=3, textvariable=self.axis_var).pack(side="left", padx=(2, 10))
        ttk.Button(self, text="✕", width=3, command=lambda: on_remove(self)).pack(side="left")

    def _pick(self, var: tk.StringVar):
        label = pick_column(self, self.options, var.get())
        if label:
            var.set(label)

    def get(self) -> TraceSpec:
        try:
            axis = max(1, int(self.axis_var.get()))
        except ValueError:
            axis = 1
        return TraceSpec(self.x_var.get(), self.y_var.get(), axis)


class SubplotPanel(ttk.LabelFrame):
    """One subplot: one or more traces."""

    def __init__(self, master, options, on_remove):
        super().__init__(master, padding=6)
        self.options = options
        self.rows: list[TraceRow] = []
        head = ttk.Frame(self)
        head.pack(fill="x")
        ttk.Button(head, text="+ Trace", command=self.add_trace).pack(side="left")
        self.split_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="Split y axes (traces with the same y-axis # share one)",
                        variable=self.split_var).pack(side="left", padx=12)
        self.equal_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="Lock axis scaling (1:1)",
                        variable=self.equal_var).pack(side="left", padx=4)
        ttk.Button(head, text="Remove subplot", command=lambda: on_remove(self)).pack(side="right")
        self.body = ttk.Frame(self)
        self.body.pack(fill="x", pady=(4, 0))
        self.add_trace()

    def add_trace(self):
        if self.rows:
            prev = self.rows[-1].get()
            x, y = prev.x, prev.y
        else:
            x, y = DEFAULT_X, DEFAULT_Y
        row = TraceRow(self.body, self.options, self.remove_trace, x, y, len(self.rows) + 1)
        row.pack(fill="x", pady=1)
        self.rows.append(row)

    def remove_trace(self, row):
        if len(self.rows) <= 1:  # keep at least one trace per subplot
            return
        self.rows.remove(row)
        row.destroy()

    def get_spec(self) -> SubplotSpec:
        return SubplotSpec([r.get() for r in self.rows],
                           equal_aspect=self.equal_var.get(),
                           split_y=self.split_var.get())


# --------------------------------------------------------------------------- #
# Application
# --------------------------------------------------------------------------- #
class App:
    def __init__(self, root: tk.Tk, initial_paths=()):
        self.root = root
        root.title("Force / pose experiment viewer")
        root.geometry("1350x850")
        self.store = DataStore()
        self.desc_cols: list[str] = []
        self.desc_rows: dict = {}
        self._keys: dict[str, object] = {}  # tree iid -> experiment key
        self.options = axis_options()
        self.panels: list[SubplotPanel] = []
        self.smooth_var = tk.StringVar(value="1")
        self.maxpts_var = tk.StringVar(value="0")
        self.status = tk.StringVar(value="Add data files to start.")
        self._build_ui()
        if initial_paths:
            self.store.add_paths(initial_paths)
            self.refresh_table()
        self.add_panel()

    # ---- layout ---------------------------------------------------------- #
    def _build_ui(self):
        ttk.Label(self.root, textvariable=self.status, relief="sunken", anchor="w",
                  padding=(6, 2)).pack(side="bottom", fill="x")
        paned = ttk.PanedWindow(self.root, orient="vertical")
        paned.pack(fill="both", expand=True, padx=6, pady=6)

        # experiments table -------------------------------------------------
        top = ttk.LabelFrame(paned, text="Experiments (select the ones to plot)", padding=6)
        paned.add(top, weight=1)
        bar = ttk.Frame(top)
        bar.pack(fill="x", pady=(0, 4))
        for text, cmd in [
            ("Add files…", self.add_files),
            ("Add folder…", self.add_folder),
            ("Load description CSV…", self.load_description),
            ("Select all", self.select_all),
            ("Remove selected", self.remove_selected),
            ("Clear data", self.clear_data),
        ]:
            ttk.Button(bar, text=text, command=cmd).pack(side="left", padx=(0, 4))

        frame = ttk.Frame(top)
        frame.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(frame, show="headings", selectmode="extended", height=8)
        vsb = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        hsb = ttk.Scrollbar(frame, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        self.tree.tag_configure("nodata", foreground="gray55")
        self.refresh_table()

        # subplot configuration ---------------------------------------------
        bottom = ttk.LabelFrame(paned, text="Plots", padding=6)
        paned.add(bottom, weight=2)
        bar2 = ttk.Frame(bottom)
        bar2.pack(fill="x", pady=(0, 4))
        ttk.Button(bar2, text="+ Add subplot", command=self.add_panel).pack(side="left")
        ttk.Label(bar2, text="   Derivative smoothing (samples):").pack(side="left")
        ttk.Spinbox(bar2, from_=1, to=999, width=5, textvariable=self.smooth_var).pack(side="left", padx=4)
        ttk.Label(bar2, text="   Max points per trace (0 = all):").pack(side="left")
        ttk.Spinbox(bar2, from_=0, to=10_000_000, increment=10_000, width=9,
                    textvariable=self.maxpts_var).pack(side="left", padx=4)
        ttk.Button(bar2, text="Plot ▶", command=self.plot).pack(side="right")
        ttk.Button(bar2, text="Save HTML…", command=self.save_html).pack(side="right", padx=6)

        self.scroll = ScrollFrame(bottom)
        self.scroll.pack(fill="both", expand=True)

    # ---- experiments table ------------------------------------------------ #
    def refresh_table(self):
        selected = {self._keys.get(i) for i in self.tree.selection()}
        heads = ["exp #"] + KINDS + self.desc_cols
        ids = [f"c{i}" for i in range(len(heads))]
        self.tree.configure(columns=ids)
        for i, (cid, head) in enumerate(zip(ids, heads)):
            self.tree.heading(cid, text=head)
            if i == 0:
                self.tree.column(cid, width=60, anchor="center", stretch=False)
            elif i <= len(KINDS):
                self.tree.column(cid, width=130, anchor="center", stretch=False)
            else:
                self.tree.column(cid, width=170, anchor="w", stretch=False)

        self.tree.delete(*self.tree.get_children())
        self._keys.clear()
        keys = set(self.store.experiments()) | set(self.desc_rows)
        n_data = 0
        for k in sorted(keys, key=exp_sort_key):
            has = [self.store.has(kind, k) for kind in KINDS]
            n_data += any(has)
            vals = [str(k)] + ["✓" if h else "" for h in has]
            vals += [self.desc_rows.get(k, {}).get(c, "") for c in self.desc_cols]
            iid = f"exp:{k}"
            self.tree.insert("", "end", iid=iid, values=vals, tags=() if any(has) else ("nodata",))
            self._keys[iid] = k
            if k in selected:
                self.tree.selection_add(iid)
        self.status.set(f"{n_data} experiment(s) with data files, {len(self.desc_rows)} description row(s).")

    def add_files(self):
        paths = filedialog.askopenfilenames(
            title="Select data files",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if paths:
            self._add(paths)

    def add_folder(self):
        folder = filedialog.askdirectory(title="Select folder with data files")
        if folder:
            self._add([folder])

    def _add(self, paths):
        added, skipped = self.store.add_paths(paths)
        self.refresh_table()
        msg = f"Added {added} file(s)."
        if skipped:
            msg += f" Ignored {len(skipped)} file(s) with unrecognised names."
        self.status.set(msg)

    def load_description(self):
        path = filedialog.askopenfilename(
            title="Select experiment description CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            self.desc_cols, self.desc_rows = read_description(path)
        except Exception as e:
            messagebox.showerror("Could not read description file", str(e))
            return
        self.refresh_table()

    def select_all(self):
        self.tree.selection_set(self.tree.get_children())

    def remove_selected(self):
        keys = [self._keys[i] for i in self.tree.selection()]
        self.store.remove([k for k in keys if isinstance(k, int)])
        self.refresh_table()

    def clear_data(self):
        self.store.clear()
        self.refresh_table()

    # ---- subplot panels ---------------------------------------------------- #
    def add_panel(self):
        panel = SubplotPanel(self.scroll.inner, self.options, self.remove_panel)
        panel.pack(fill="x", pady=4, padx=2)
        self.panels.append(panel)
        self._renumber()

    def remove_panel(self, panel):
        self.panels.remove(panel)
        panel.destroy()
        self._renumber()

    def _renumber(self):
        for i, p in enumerate(self.panels, start=1):
            p.configure(text=f"Subplot {i}")

    # ---- plotting ---------------------------------------------------------- #
    def _int(self, var: tk.StringVar, minimum: int, default: int) -> int:
        try:
            value = max(minimum, int(var.get()))
        except ValueError:
            value = default
        var.set(str(value))
        return value

    def _collect(self):
        keys = [self._keys[i] for i in self.tree.selection()]
        nums = sorted(k for k in keys if isinstance(k, int)
                      and any(self.store.has(kind, k) for kind in KINDS))
        if not nums:
            messagebox.showwarning("Nothing selected",
                                   "Select at least one experiment that has data files.")
            return None
        if not self.panels:
            messagebox.showwarning("No subplots", "Add at least one subplot.")
            return None
        smooth = self._int(self.smooth_var, 1, 1)
        max_points = self._int(self.maxpts_var, 0, 0)
        return nums, [p.get_spec() for p in self.panels], smooth, max_points

    def _render(self, save_path=None):
        collected = self._collect()
        if collected is None:
            return
        nums, specs, smooth, max_points = collected
        self.status.set("Loading data and building figure…")
        self.root.update_idletasks()
        try:
            fig, warns = build_figure(self.store, nums, specs, smooth, max_points=max_points)
            if not fig.data:
                self.status.set("Nothing to plot.")
                messagebox.showwarning("Nothing to plot", "\n".join(warns[:20]) or "No data found.")
                return
            path = export_figure(fig, save_path)
        except Exception:
            self.status.set("Error while plotting.")
            messagebox.showerror("Error", traceback.format_exc())
            return
        self.status.set(f"Plotted {len(nums)} experiment(s) in {len(specs)} subplot(s) → {path}")
        if warns:
            more = "\n…" if len(warns) > 20 else ""
            messagebox.showwarning("Some data was skipped", "\n".join(warns[:20]) + more)

    def plot(self):
        self._render()

    def save_html(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".html", filetypes=[("HTML files", "*.html")],
            title="Save interactive plot")
        if path:
            self._render(save_path=path)


def main():
    root = tk.Tk()
    App(root, sys.argv[1:])
    root.mainloop()


if __name__ == "__main__":
    main()
