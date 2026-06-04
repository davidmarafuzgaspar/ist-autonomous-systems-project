from __future__ import annotations

import sys
import tkinter as tk
import tkinter.font as tkfont
from typing import Callable

BG = "#1e1e1e"
BG2 = "#2a2a2a"
FG = "#f0f0f0"
MUTED = "#9ca3af"
ACCENT = "#4a9eff"
BORDER = "#404040"

CANVAS_BG = "#252525"
BOARD = "#fafafa"
LINE = "#d4d4d4"
CELL_FREE = "#ffffff"
CELL_BLOCK = "#6b7280"
CELL_START = "#22c55e"
CELL_GOAL = "#4a9eff"
POLICY = "#4a9eff"
OBSTACLE = "#6b7280"
ROBOT = "#e91e63"
TRAIL = "#ff80ab"

POS_SCALE_MAX = 150.0
NEG_SCALE_MAX = 60.0

FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 11, "bold")
FONT_TITLE = ("Segoe UI", 14, "bold")


def setup(root: tk.Misc) -> None:
    global FONT, FONT_BOLD, FONT_TITLE
    root.configure(bg=BG)
    try:
        names = set(tkfont.families())
        for family in ("Segoe UI", "Ubuntu", "Helvetica Neue", "Arial"):
            if family in names:
                FONT = (family, 10)
                FONT_BOLD = (family, 11, "bold")
                FONT_TITLE = (family, 14, "bold")
                break
    except (tk.TclError, RuntimeError):
        if sys.platform.startswith("linux"):
            FONT = ("Ubuntu", 10)
            FONT_BOLD = ("Ubuntu", 11, "bold")
            FONT_TITLE = ("Ubuntu", 14, "bold")


def label(parent: tk.Misc, text: str, *, muted: bool = False) -> tk.Label:
    return tk.Label(
        parent,
        text=text,
        font=FONT,
        fg=MUTED if muted else FG,
        bg=parent.cget("bg") if parent.cget("bg") else BG,
        anchor="w",
    )


def button(parent: tk.Misc, text: str, command: Callable[[], None], *, primary: bool = False) -> tk.Button:
    return tk.Button(
        parent,
        text=text,
        command=command,
        font=FONT_BOLD if primary else FONT,
        bg=ACCENT if primary else BG2,
        fg="#ffffff" if primary else FG,
        activebackground="#3d8bfd" if primary else "#353535",
        activeforeground="#ffffff",
        relief="flat",
        padx=12,
        pady=6,
        cursor="hand2",
    )


def entry(parent: tk.Misc, *, width: int = 6) -> tk.Entry:
    return tk.Entry(
        parent,
        width=width,
        font=FONT,
        bg=BG2,
        fg=FG,
        insertbackground=FG,
        relief="flat",
        highlightthickness=1,
        highlightbackground=BORDER,
        highlightcolor=ACCENT,
    )


def radio(
    parent: tk.Misc,
    text: str,
    variable: tk.StringVar,
    value: str,
    *,
    command: Callable[[], None] | None = None,
) -> tk.Radiobutton:
    return tk.Radiobutton(
        parent,
        text=text,
        variable=variable,
        value=value,
        command=command,
        font=FONT,
        fg=FG,
        bg=parent.cget("bg") if parent.cget("bg") else BG,
        selectcolor=BG2,
        activebackground=BG,
        activeforeground=FG,
        anchor="w",
    )


def line(parent: tk.Misc) -> None:
    tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", pady=10)
