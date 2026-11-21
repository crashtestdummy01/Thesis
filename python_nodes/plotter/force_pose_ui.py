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
        except KeyError:  # e.g. a combobox popdown
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
    """One x/y pair of dropdowns."""

    def __init__(self, master, options, on_remove, x, y):
        super().__init__(master)
        self.x_var, self.y_var = tk.StringVar(value=x), tk.StringVar(value=y)
        ttk.Label(self, text="x:").pack(side="left")
        ttk.Combobox(self, textvariable=self.x_var, values=options, state="readonly",
                     width=34).pack(side="left", padx=(2, 10))
        ttk.Label(self, text="y:").pack(side="left")
        ttk.Combobox(self, textvariable=self.y_var, values=options, state="readonly",
                     width=34).pack(side="left", padx=(2, 10))
        ttk.Button(self, text="✕", width=3, command=lambda: on_remove(self)).pack(side="left")

    def get(self):
        return self.x_var.get(), self.y_var.get()


class SubplotPanel(ttk.LabelFrame):
    """One subplot: one or more traces (x/y dropdown pairs)."""

    def __init__(self, master, options, on_remove):
        super().__init__(master, padding=6)
        self.options = options
        self.rows: list[TraceRow] = []
        head = ttk.Frame(self)
        head.pack(fill="x")
        ttk.Button(head, text="+ Trace", command=self.add_trace).pack(side="left")
        self.equal_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="Lock axis scaling (1:1)",
                        variable=self.equal_var).pack(side="left", padx=12)
        ttk.Button(head, text="Remove subplot", command=lambda: on_remove(self)).pack(side="right")
        self.body = ttk.Frame(self)
        self.body.pack(fill="x", pady=(4, 0))
        self.add_trace()

    def add_trace(self):
        x, y = self.rows[-1].get() if self.rows else (DEFAULT_X, DEFAULT_Y)
        row = TraceRow(self.body, self.options, self.remove_trace, x, y)
        row.pack(fill="x", pady=1)
        self.rows.append(row)

    def remove_trace(self, row):
        if len(self.rows) <= 1:  # keep at least one trace per subplot
            return
        self.rows.remove(row)
        row.destroy()

    def get_spec(self) -> SubplotSpec:
        return SubplotSpec([TraceSpec(*r.get()) for r in self.rows],
                           equal_aspect=self.equal_var.get())


# --------------------------------------------------------------------------- #
# Application
# --------------------------------------------------------------------------- #
class App:
    def __init__(self, root: tk.Tk, initial_paths=()):
        self.root = root
        root.title("Force / pose experiment viewer")
        root.geometry("1300x850")
        self.store = DataStore()
        self.desc_cols: list[str] = []
        self.desc_rows: dict = {}
        self._keys: dict[str, object] = {}  # tree iid -> experiment key
        self.options = axis_options()
        self.panels: list[SubplotPanel] = []
        self.smooth_var = tk.StringVar(value="1")
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
                self.tree.column(cid, width=140, anchor="center", stretch=False)
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
        try:
            smooth = max(1, int(self.smooth_var.get()))
        except ValueError:
            smooth = 1
            self.smooth_var.set("1")
        return nums, [p.get_spec() for p in self.panels], smooth

    def _render(self, save_path=None):
        collected = self._collect()
        if collected is None:
            return
        nums, specs, smooth = collected
        self.status.set("Loading data and building figure…")
        self.root.update_idletasks()
        try:
            fig, warns = build_figure(self.store, nums, specs, smooth)
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
