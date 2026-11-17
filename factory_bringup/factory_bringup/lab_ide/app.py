#!/usr/bin/env python3
"""
Vision Lab IDE: create a lab project for a world, edit its script, build and run it with the simulation.

    ./run_lab_ide.sh [PROJECT_FOLDER]          (in the simulator project folder; no build needed)
    ros2 run factory_bringup lab_ide           (after colcon build)

A lab project is a folder with labide.json (its world and run options) and lab_<name>.py scripts.
  New project      pick a name, a folder and a world (pipeline, pipeline-tree, dual, straight, ...): the
                   IDE writes a starter script for that world: its stations, arms, bents, bins and docks,
                   with a plan per arm, or one of the vision_lab templates / an existing lab as a start
  Build            saves, checks the scripts for Python errors (double-click an error to jump to it) and
                   runs colcon build of factory_bringup when its installed copy is older than the source
  Run              Build, then Gazebo (run_vision_lab.sh sim --world W --drawn-cameras ...), waits until
                   the world is up, then the lab (run_vision_lab.sh run lab_<name>.py --world W)
  Start simulation / Run lab only / Stop: each pane has its own Stop button (Ctrl+C to the process group)
"""

import datetime
import hashlib
import json
import os
import re
import shlex
import shutil
import signal
import sys
from typing import Dict, List, Optional

from PyQt5.QtCore import QProcess, QProcessEnvironment, QRect, QRegExp, QSettings, QSize, Qt, QTimer
from PyQt5.QtGui import (QColor, QFont, QFontDatabase, QKeySequence, QPainter, QSyntaxHighlighter, QTextCharFormat,
                         QTextCursor, QTextFormat)
from PyQt5.QtWidgets import (QAction, QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDockWidget,
                             QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
                             QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
                             QTabWidget, QTextEdit, QToolBar, QVBoxLayout, QWidget)

from factory_bringup.lab_ide import generator, world_info

APP_NAME = "Vision Lab IDE"
PROJECT_FILE = "labide.json"
DEFAULT_PROJECTS = os.path.expanduser("~/vision_lab_projects")
ANSI = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
TRACE = re.compile(r'File "([^"]+)", line (\d+)')
DEFAULT_OPTIONS = {
    "drawn_cameras": True,      # pipeline: the lab draws the station images (Gazebo renders no cameras)
    "headless": False,          # Gazebo without its 3D window
    "nvidia": False,            # render on the NVIDIA GPU of a hybrid laptop
    "fake": False,              # pipeline: no Gazebo at all (--fake)
    "world_products": True,     # single-arm worlds: the world's own products, no order window
    "lab_window": True,         # the Vision Lab image window
    "sim_args": "",
    "lab_args": "",
    "env": "",                  # KEY=VALUE ... for both (e.g. PIPELINE_CAMERA=lite)
}


# ====================================================================== paths
def find_sim_project() -> str:
    """The simulator project folder (the one with run_vision_lab.sh)."""
    for cand in (os.environ.get("FACTORY_PROJECT_DIR"), QSettings("factory_bringup", "lab_ide").value("sim_project")):
        if cand and os.path.isfile(os.path.join(cand, "run_vision_lab.sh")):
            return os.path.abspath(cand)
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        if os.path.isfile(os.path.join(d, "run_vision_lab.sh")):
            return d
        d = os.path.dirname(d)
    return os.path.expanduser("~/ros2_ws/src/factory_bent_project")


def ws_dir_of(sim_project: str) -> str:
    ws = os.path.abspath(os.path.join(sim_project, "..", ".."))
    return ws if os.path.isdir(os.path.join(ws, "src")) else os.path.expanduser("~/ros2_ws")


def _md5(path: str) -> Optional[str]:
    try:
        with open(path, "rb") as f:
            return hashlib.md5(f.read()).hexdigest()
    except OSError:
        return None


def stale_install(sim_project: str) -> List[str]:
    """Source files of factory_bringup whose installed copy is missing or different (empty = up to date)."""
    src = os.path.join(sim_project, "factory_bringup")
    pkg = os.path.join(src, "factory_bringup")
    ws = ws_dir_of(sim_project)
    lib = os.path.join(ws, "install", "factory_bringup", "lib")
    site = None
    if os.path.isdir(lib):
        for d in sorted(os.listdir(lib)):
            cand = os.path.join(lib, d, "site-packages", "factory_bringup")
            if d.startswith("python") and os.path.isdir(cand):
                site = cand
    if site is None:
        return ["(factory_bringup is not installed: install/factory_bringup is missing)"]
    out = []
    for root, dirs, files in os.walk(pkg):
        dirs[:] = [d for d in dirs if d != "__pycache__"]
        for f in files:
            if not f.endswith(".py"):
                continue
            s = os.path.join(root, f)
            rel = os.path.relpath(s, pkg)
            if not _belongs_to_package(rel):
                continue
            if _md5(s) != _md5(os.path.join(site, rel)):
                out.append(os.path.join("factory_bringup", rel))
    share = os.path.join(ws, "install", "factory_bringup", "share", "factory_bringup")
    for sub, pattern in (("launch", ".launch.py"), ("lab_examples", ".py")):
        d = os.path.join(src, sub)
        if os.path.isdir(d):
            dst = "vision_lab_examples" if sub == "lab_examples" else sub
            for f in sorted(os.listdir(d)):
                if f.endswith(pattern) and _md5(os.path.join(d, f)) != _md5(os.path.join(share, dst, f)):
                    out.append(os.path.join(sub, f))
    return out


def _belongs_to_package(rel: str) -> bool:
    """Only the packages setup.py installs (factory_bringup, .vision_lab, .vision_lab.detectors/templates,
    .lab_ide); other folders in the source tree are not copied by colcon."""
    parts = rel.split(os.sep)[:-1]
    return parts in ([], ["vision_lab"], ["vision_lab", "detectors"], ["vision_lab", "templates"], ["lab_ide"])


# ====================================================================== editor
class PythonHighlighter(QSyntaxHighlighter):
    KEYWORDS = ("and as assert async await break class continue def del elif else except False finally for from "
                "global if import in is lambda None nonlocal not or pass raise return True try while with yield "
                "self").split()

    def __init__(self, doc):
        super().__init__(doc)

        def fmt(color, bold=False, italic=False):
            f = QTextCharFormat()
            f.setForeground(QColor(color))
            if bold:
                f.setFontWeight(QFont.Bold)
            f.setFontItalic(italic)
            return f
        self.rules = [(QRegExp(r"\b" + k + r"\b"), fmt("#c678dd", bold=True)) for k in self.KEYWORDS]
        self.rules += [
            (QRegExp(r"\b(print|len|range|dict|list|set|str|int|float|isinstance|hasattr|getattr|sorted|sum|min|max)"
                     r"\b"), fmt("#56b6c2")),
            (QRegExp(r"\b[A-Z][A-Z0-9_]{2,}\b"), fmt("#e5c07b", bold=True)),             # CONSTANTS
            (QRegExp(r"\bdef\s+(\w+)"), fmt("#61afef", bold=True)),
            (QRegExp(r"\b[0-9]+(\.[0-9]+)?\b"), fmt("#d19a66")),
            (QRegExp(r"\"[^\"\\]*(\\.[^\"\\]*)*\""), fmt("#98c379")),
            (QRegExp(r"'[^'\\]*(\\.[^'\\]*)*'"), fmt("#98c379")),
            (QRegExp(r"#[^\n]*"), fmt("#7f848e", italic=True)),
        ]
        self.doc_fmt = fmt("#98c379", italic=True)

    def highlightBlock(self, text):
        for rx, f in self.rules:
            i = rx.indexIn(text, 0)
            while i >= 0:
                n = rx.matchedLength()
                self.setFormat(i, n, f)
                i = rx.indexIn(text, i + max(n, 1))
        # triple-quoted strings (docstrings) across lines
        self.setCurrentBlockState(0)
        start = 0
        if self.previousBlockState() != 1:
            start = text.find('"""')
            if start < 0:
                return
            end = text.find('"""', start + 3)
            if end >= 0:
                self.setFormat(start, end + 3 - start, self.doc_fmt)
                return
            self.setFormat(start, len(text) - start, self.doc_fmt)
            self.setCurrentBlockState(1)
            return
        end = text.find('"""')
        if end >= 0:
            self.setFormat(0, end + 3, self.doc_fmt)
        else:
            self.setFormat(0, len(text), self.doc_fmt)
            self.setCurrentBlockState(1)


class _LineArea(QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor

    def sizeHint(self):
        return QSize(self.editor.line_area_width(), 0)

    def paintEvent(self, event):
        self.editor.paint_line_area(event)


class CodeEditor(QPlainTextEdit):
    """A Python editor: line numbers, highlighting, current line, 4-space indent, auto-indent."""

    def __init__(self, path: str, parent=None):
        super().__init__(parent)
        self.path = path
        font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        font.setPointSize(11)
        self.setFont(font)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.setTabStopDistance(self.fontMetrics().horizontalAdvance(" ") * 4)
        self.area = _LineArea(self)
        self.highlighter = PythonHighlighter(self.document())
        self.blockCountChanged.connect(lambda _n: self.setViewportMargins(self.line_area_width(), 0, 0, 0))
        self.updateRequest.connect(self._update_area)
        self.cursorPositionChanged.connect(self._highlight_line)
        self.setViewportMargins(self.line_area_width(), 0, 0, 0)
        with open(path, encoding="utf-8") as f:
            self.setPlainText(f.read())
        self.document().setModified(False)
        self._highlight_line()

    # ---- line numbers
    def line_area_width(self) -> int:
        return 12 + self.fontMetrics().horizontalAdvance("9") * max(3, len(str(self.blockCount())))

    def _update_area(self, rect, dy):
        if dy:
            self.area.scroll(0, dy)
        else:
            self.area.update(0, rect.y(), self.area.width(), rect.height())

    def resizeEvent(self, e):
        super().resizeEvent(e)
        cr = self.contentsRect()
        self.area.setGeometry(QRect(cr.left(), cr.top(), self.line_area_width(), cr.height()))

    def paint_line_area(self, event):
        p = QPainter(self.area)
        p.fillRect(event.rect(), self.palette().alternateBase())
        block = self.firstVisibleBlock()
        n = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())
        p.setPen(QColor("#888888"))
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                p.drawText(0, top, self.area.width() - 6, self.fontMetrics().height(), Qt.AlignRight, str(n + 1))
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            n += 1

    def _highlight_line(self):
        sel = QTextEdit.ExtraSelection()
        color = self.palette().highlight().color()
        color.setAlpha(40)
        sel.format.setBackground(color)
        sel.format.setProperty(QTextFormat.FullWidthSelection, True)
        sel.cursor = self.textCursor()
        sel.cursor.clearSelection()
        self.setExtraSelections([sel])

    # ---- keys
    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Tab and not e.modifiers():
            self.textCursor().insertText("    ")
            return
        if e.key() in (Qt.Key_Return, Qt.Key_Enter):
            line = self.textCursor().block().text()
            indent = line[:len(line) - len(line.lstrip())]
            if line.rstrip().endswith(":"):
                indent += "    "
            super().keyPressEvent(e)
            self.textCursor().insertText(indent)
            return
        super().keyPressEvent(e)

    def goto_line(self, line: int):
        block = self.document().findBlockByNumber(max(0, line - 1))
        c = QTextCursor(block)
        c.select(QTextCursor.LineUnderCursor)
        self.setTextCursor(c)
        self.centerCursor()
        self.setFocus()

    def save(self):
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(self.toPlainText())
        self.document().setModified(False)


# ====================================================================== process panes
class ProcessPane(QWidget):
    """Output of one process (build, simulation, lab) with Stop / Clear; stops the whole process group."""

    def __init__(self, title: str, on_jump=None, parent=None):
        super().__init__(parent)
        self.title = title
        self.on_jump = on_jump
        self.proc: Optional[QProcess] = None
        self.pgid: Optional[int] = None
        self.on_finished = []
        self.status = QLabel("idle")
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop)
        clear = QPushButton("Clear")
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(20000)
        self.out.setLineWrapMode(QPlainTextEdit.NoWrap)
        font = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        font.setPointSize(10)
        self.out.setFont(font)
        self.out.mouseDoubleClickEvent = self._double_click
        clear.clicked.connect(self.out.clear)
        top = QHBoxLayout()
        top.addWidget(self.status, 1)
        top.addWidget(clear)
        top.addWidget(self.stop_btn)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.addLayout(top)
        lay.addWidget(self.out)
        self._kill_timers: List[QTimer] = []

    def running(self) -> bool:
        return self.proc is not None and self.proc.state() != QProcess.NotRunning

    def append(self, text: str, color: Optional[str] = None):
        c = self.out.textCursor()
        c.movePosition(QTextCursor.End)
        if color:
            f = QTextCharFormat()
            f.setForeground(QColor(color))
            c.insertText(text, f)
        else:
            c.insertText(text, QTextCharFormat())
        self.out.setTextCursor(c)
        self.out.ensureCursorVisible()

    def note(self, text: str):
        self.append(text.rstrip("\n") + "\n", "#2a8adf")

    def start(self, argv: List[str], cwd: str, env: Dict[str, str]) -> bool:
        if self.running():
            self.note(f"{self.title} is already running.")
            return False
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.setWorkingDirectory(cwd)
        penv = QProcessEnvironment.systemEnvironment()
        for k, v in env.items():
            penv.insert(k, v)
        self.proc.setProcessEnvironment(penv)
        self.proc.readyReadStandardOutput.connect(self._read)
        self.proc.finished.connect(self._finished)
        self.proc.errorOccurred.connect(lambda err: self.note(f"cannot start: {self.proc.errorString()}"))
        # own process group (setsid), so Stop reaches gz sim / ros2 run and their children
        prog = argv
        if shutil.which("setsid"):
            prog = ["setsid"] + argv
        self.note(f"$ (cd {shlex.quote(cwd)} && {' '.join(shlex.quote(a) for a in argv)})")
        self.proc.start(prog[0], prog[1:])
        ok = self.proc.waitForStarted(5000)
        self.pgid = int(self.proc.processId()) if ok and prog[0] == "setsid" else None
        self.status.setText("running" if ok else "failed to start")
        self.stop_btn.setEnabled(ok)
        return ok

    def _read(self):
        data = bytes(self.proc.readAllStandardOutput()).decode("utf-8", "replace")
        data = ANSI.sub("", data).replace("\r\n", "\n").replace("\r", "\n")
        low = data.lower()
        color = "#d0473f" if ("error" in low or "traceback" in low or "failed" in low) else None
        self.append(data, color)

    def _finished(self, code, status):
        for t in self._kill_timers:
            t.stop()
        self._kill_timers = []
        if self.pgid:                         # children left in the group (background jobs ignore Ctrl+C)
            try:
                os.killpg(self.pgid, signal.SIGTERM)
            except OSError:
                pass
            self.pgid = None
        self.stop_btn.setEnabled(False)
        how = "crashed / killed" if status == QProcess.CrashExit else f"exit code {code}"
        self.status.setText(f"finished ({how})")
        self.note(f"--- {self.title} finished ({how})")
        for cb in list(self.on_finished):
            cb(code if status == QProcess.NormalExit else -1)

    def _signal(self, sig):
        if not self.running():
            return
        pid = int(self.proc.processId())
        try:
            os.killpg(pid, sig)               # setsid: pid is the process group
        except (ProcessLookupError, PermissionError, OSError):
            try:
                os.kill(pid, sig)
            except OSError:
                pass

    def stop(self):
        if not self.running():
            return
        self.note(f"Stopping {self.title} (Ctrl+C) ...")
        self.status.setText("stopping ...")
        self._signal(signal.SIGINT)
        for delay, sig in ((8000, signal.SIGTERM), (15000, signal.SIGKILL)):
            t = QTimer(self)
            t.setSingleShot(True)
            t.timeout.connect(lambda s=sig: self._signal(s))
            t.start(delay)
            self._kill_timers.append(t)

    def _double_click(self, event):
        c = self.out.cursorForPosition(event.pos())
        line = c.block().text()
        m = TRACE.search(line) or re.search(r"([^\s:\"']+\.py):(\d+)", line)
        if m and self.on_jump:
            self.on_jump(m.group(1), int(m.group(2)))
        QPlainTextEdit.mouseDoubleClickEvent(self.out, event)


# ====================================================================== dialogs
class NewProjectDialog(QDialog):
    def __init__(self, labs_dir: str, world: str = "pipeline", parent=None, script_only: bool = False):
        super().__init__(parent)
        self.labs_dir = labs_dir
        self.setWindowTitle("New lab script" if script_only else "New lab project")
        self.name = QLineEdit("my_lab")
        self.folder = QLineEdit(DEFAULT_PROJECTS)
        browse = QPushButton("...")
        browse.clicked.connect(self._browse)
        self.world = QComboBox()
        for w in world_info.WORLDS:
            self.world.addItem(f"{w.key}  -  {w.title}", w.key)
        self.world.setCurrentIndex(max(0, self.world.findData(world)))
        self.starter = QComboBox()
        for sid, label in generator.starters(labs_dir):
            self.starter.addItem(label, sid)
        self.starter.setMinimumWidth(520)
        self.title = QLineEdit()
        self.title.setPlaceholderText("one-line title (optional)")
        form = QFormLayout(self)
        form.addRow("Name", self.name)
        if not script_only:
            row = QHBoxLayout()
            row.addWidget(self.folder, 1)
            row.addWidget(browse)
            form.addRow("Projects folder", row)
            form.addRow("World", self.world)
        form.addRow("Starter", self.starter)
        form.addRow("Title", self.title)
        hint = QLabel("The project folder is <projects folder>/<name>; the script is lab_<name>.py in it.")
        hint.setWordWrap(True)
        form.addRow(hint)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def _browse(self):
        d = QFileDialog.getExistingDirectory(self, "Projects folder", self.folder.text())
        if d:
            self.folder.setText(d)

    def values(self):
        return dict(name=generator.slug(self.name.text() or "my_lab"), folder=self.folder.text().strip(),
                    world=self.world.currentData(), starter=self.starter.currentData(),
                    title=self.title.text().strip() or None)


# ====================================================================== main window
class LabIDE(QMainWindow):
    def __init__(self, project: Optional[str] = None):
        super().__init__()
        self.settings = QSettings("factory_bringup", "lab_ide")
        self.sim_project = find_sim_project()
        self.project_dir: Optional[str] = None
        self.config: Dict = {}
        self.world_infos: Dict[str, Dict] = {}
        self._pending_run = False
        self._wait_timer = QTimer(self)
        self._wait_timer.timeout.connect(self._poll_sim)
        self._wait_started = 0.0
        self._probe: Optional[QProcess] = None
        self.resize(1500, 950)
        self._build_ui()
        self._update_title()
        if project:
            self.open_project(project)
        else:
            last = self.settings.value("last_project")
            if last and os.path.isfile(os.path.join(last, PROJECT_FILE)):
                self.open_project(last)

    # ---------------------------------------------------------------- UI
    def _build_ui(self):
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self.close_tab)
        self.setCentralWidget(self.tabs)

        # panes
        self.build_pane = ProcessPane("Build", self.jump_to)
        self.sim_pane = ProcessPane("Simulation", self.jump_to)
        self.lab_pane = ProcessPane("Lab", self.jump_to)
        self.out_tabs = QTabWidget()
        self.out_tabs.addTab(self.build_pane, "Build")
        self.out_tabs.addTab(self.sim_pane, "Simulation (Gazebo)")
        self.out_tabs.addTab(self.lab_pane, "Lab")
        dock = QDockWidget("Output", self)
        dock.setObjectName("output")
        dock.setWidget(self.out_tabs)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        self.sim_pane.on_finished.append(self._sim_finished)
        self.lab_pane.on_finished.append(lambda code: self.statusBar().showMessage(f"Lab finished (exit {code})"))

        # left: project, world, files, options
        left = QWidget()
        lv = QVBoxLayout(left)
        self.project_label = QLabel("No project. File > New project (Ctrl+N) or Open project (Ctrl+O).")
        self.project_label.setWordWrap(True)
        lv.addWidget(self.project_label)
        wbox = QGroupBox("World")
        wl = QVBoxLayout(wbox)
        self.world_combo = QComboBox()
        for w in world_info.WORLDS:
            self.world_combo.addItem(w.key, w.key)
            self.world_combo.setItemData(self.world_combo.count() - 1, w.title, Qt.ToolTipRole)
        self.world_combo.currentIndexChanged.connect(self._world_changed)
        wl.addWidget(self.world_combo)
        self.world_text = QPlainTextEdit()
        self.world_text.setReadOnly(True)
        self.world_text.setLineWrapMode(QPlainTextEdit.NoWrap)
        f = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        f.setPointSize(9)
        self.world_text.setFont(f)
        self.world_text.setMinimumHeight(220)
        wl.addWidget(self.world_text)
        gen = QPushButton("Generate script for this world ...")
        gen.clicked.connect(self.new_script)
        wl.addWidget(gen)
        lv.addWidget(wbox, 3)

        fbox = QGroupBox("Scripts (double-click to open; the selected one runs)")
        fl = QVBoxLayout(fbox)
        self.files = QListWidget()
        self.files.itemDoubleClicked.connect(lambda it: self.open_file(it.data(Qt.UserRole)))
        self.files.currentItemChanged.connect(self._main_changed)
        fl.addWidget(self.files)
        lv.addWidget(fbox, 2)

        obox = QGroupBox("Run options")
        ol = QFormLayout(obox)
        self.opt = {}
        for key, label, tip in (
                ("drawn_cameras", "Drawn cameras (pipeline)", "Gazebo renders no cameras; the lab draws them (--drawn-cameras)"),
                ("headless", "Gazebo without 3D window", "--headless (open it later with: gz sim -g)"),
                ("nvidia", "NVIDIA GPU (hybrid laptop)", "--nvidia: PRIME render offload"),
                ("fake", "No Gazebo (--fake, pipeline)", "the pipeline lab simulates the belts itself"),
                ("world_products", "World's products (single-arm worlds)", "--world-products order_gui:=false"),
                ("lab_window", "Vision Lab window", "off = --no-window")):
            cb = QCheckBox(label)
            cb.setToolTip(tip)
            cb.toggled.connect(self._options_changed)
            ol.addRow(cb)
            self.opt[key] = cb
        for key, label, tip in (("sim_args", "Simulation args", "extra arguments for run_vision_lab.sh sim"),
                                ("lab_args", "Lab args", "extra arguments for run_vision_lab.sh run, e.g. "
                                                         "--belt-speed 0.5 --arm locate=vision"),
                                ("env", "Environment", "KEY=VALUE ... for both, e.g. PIPELINE_CAMERA=lite")):
            le = QLineEdit()
            le.setToolTip(tip)
            le.editingFinished.connect(self._options_changed)
            ol.addRow(label, le)
            self.opt[key] = le
        lv.addWidget(obox)
        ldock = QDockWidget("Project", self)
        ldock.setObjectName("project")
        ldock.setWidget(left)
        ldock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.LeftDockWidgetArea, ldock)
        self.resizeDocks([ldock], [430], Qt.Horizontal)
        self.resizeDocks([dock], [300], Qt.Vertical)

        # actions
        def act(text, slot, shortcut=None, tip=None):
            a = QAction(text, self)
            a.triggered.connect(slot)
            if shortcut:
                a.setShortcut(QKeySequence(shortcut))
            if tip:
                a.setToolTip(tip)
            return a
        self.a_new = act("New project", self.new_project, "Ctrl+N")
        self.a_open = act("Open project", self.open_project_dialog, "Ctrl+O")
        self.a_save = act("Save", self.save_current, "Ctrl+S")
        self.a_save_all = act("Save all", self.save_all, "Ctrl+Shift+S")
        self.a_build = act("Build", lambda: self.build(), "Ctrl+B",
                           "Save, check the scripts, colcon build factory_bringup if its install is out of date")
        self.a_rebuild = act("Rebuild factory_bringup", lambda: self.build(force=True), None,
                             "colcon build --packages-select factory_bringup, always")
        self.a_run = act("Run", self.run_all, "F5", "Build, start the simulation, then the lab")
        self.a_sim = act("Start simulation", self.start_sim, "F6")
        self.a_lab = act("Run lab only", self.run_lab, "F7", "the lab on the simulation that is running")
        self.a_stop = act("Stop all", self.stop_all, "Shift+F5")
        self.a_kill_gz = act("Kill leftover Gazebo", self.kill_gazebo)
        self.a_find = act("Find ...", self.find, "Ctrl+F")
        self.a_find_next = act("Find next", self.find_next, "F3")
        self.a_sim_dir = act("Simulator project folder ...", self.choose_sim_project)
        self.a_open_file = act("Open script ...", self.open_file_dialog)
        self.a_new_script = act("New script for this world ...", self.new_script)

        mb = self.menuBar()
        m = mb.addMenu("&File")
        for a in (self.a_new, self.a_open, self.a_new_script, self.a_open_file, self.a_save, self.a_save_all):
            m.addAction(a)
        self.recent_menu = m.addMenu("Recent projects")
        self._fill_recent()
        m.addSeparator()
        m.addAction(self.a_sim_dir)
        m.addSeparator()
        m.addAction(act("Quit", self.close, "Ctrl+Q"))
        m = mb.addMenu("&Edit")
        m.addAction(self.a_find)
        m.addAction(self.a_find_next)
        m = mb.addMenu("&Build")
        m.addAction(self.a_build)
        m.addAction(self.a_rebuild)
        m = mb.addMenu("&Run")
        for a in (self.a_run, self.a_sim, self.a_lab, self.a_stop, self.a_kill_gz):
            m.addAction(a)
        m = mb.addMenu("&Help")
        m.addAction(act("Commands", self.show_help))

        tb = QToolBar("Main")
        tb.setObjectName("main")
        tb.setToolButtonStyle(Qt.ToolButtonTextOnly)
        for a in (self.a_new, self.a_open, self.a_save, None, self.a_build, self.a_run, self.a_sim, self.a_lab,
                  self.a_stop):
            tb.addSeparator() if a is None else tb.addAction(a)
        self.addToolBar(tb)
        self._load_options(DEFAULT_OPTIONS)
        self._world_changed()

    # ---------------------------------------------------------------- project
    def labs_dir(self) -> str:
        return os.path.join(self.sim_project, "labs")

    def _update_title(self):
        name = self.config.get("name") if self.project_dir else None
        self.setWindowTitle(f"{APP_NAME}" + (f" - {name} [{self.config.get('world')}]" if name else ""))
        self.statusBar().showMessage(f"Simulator project: {self.sim_project}"
                                     + (f"    Lab project: {self.project_dir}" if self.project_dir else ""))

    def _fill_recent(self):
        self.recent_menu.clear()
        for p in self._recent():
            a = QAction(p, self)
            a.triggered.connect(lambda _c=False, p=p: self.open_project(p))
            self.recent_menu.addAction(a)

    def _recent(self) -> List[str]:
        r = self.settings.value("recent") or []
        if isinstance(r, str):
            r = [r]
        return [p for p in r if os.path.isfile(os.path.join(p, PROJECT_FILE))]

    def _add_recent(self, path):
        r = [path] + [p for p in self._recent() if p != path]
        self.settings.setValue("recent", r[:10])
        self.settings.setValue("last_project", path)
        self._fill_recent()

    def new_project(self):
        dlg = NewProjectDialog(self.labs_dir(), self.world_combo.currentData() or "pipeline", self)
        if dlg.exec_() != QDialog.Accepted:
            return
        v = dlg.values()
        folder = os.path.join(os.path.expanduser(v["folder"]), v["name"])
        if os.path.exists(os.path.join(folder, PROJECT_FILE)):
            QMessageBox.warning(self, APP_NAME, f"{folder} already is a lab project; open it instead.")
            return
        try:
            info = self.world_info(v["world"])
            text = generator.generate(v["world"], v["name"], v["starter"], v["title"], self.labs_dir(), info)
        except Exception as exc:                       # noqa: BLE001
            QMessageBox.critical(self, APP_NAME, f"Cannot generate the script:\n{exc}")
            return
        os.makedirs(folder, exist_ok=True)
        script = generator.lab_file_name(v["name"])
        with open(os.path.join(folder, script), "w") as f:
            f.write(text)
        cfg = {"name": v["name"], "world": v["world"], "main": script, "starter": v["starter"],
               "created": datetime.datetime.now().isoformat(timespec="seconds"), "options": dict(DEFAULT_OPTIONS)}
        with open(os.path.join(folder, PROJECT_FILE), "w") as f:
            json.dump(cfg, f, indent=2)
        self.open_project(folder)

    def open_project_dialog(self):
        d = QFileDialog.getExistingDirectory(self, "Open lab project (a folder with labide.json, or any folder "
                                                   "with lab scripts)", DEFAULT_PROJECTS)
        if d:
            self.open_project(d)

    def open_project(self, folder: str):
        folder = os.path.abspath(os.path.expanduser(folder))
        if os.path.isfile(folder):
            folder = os.path.dirname(folder)
        cfg_path = os.path.join(folder, PROJECT_FILE)
        if os.path.isfile(cfg_path):
            with open(cfg_path) as f:
                cfg = json.load(f)
        else:                                          # a plain folder of lab files (e.g. labs/): make it a project
            scripts = sorted(f for f in os.listdir(folder) if f.endswith(".py"))
            if not scripts:
                QMessageBox.warning(self, APP_NAME, f"{folder} has no labide.json and no Python scripts.")
                return
            cfg = {"name": os.path.basename(folder), "world": "pipeline", "main": scripts[0],
                   "options": dict(DEFAULT_OPTIONS)}
            if QMessageBox.question(self, APP_NAME, f"Make {folder} a lab project (writes {PROJECT_FILE})?") \
                    == QMessageBox.Yes:
                with open(cfg_path, "w") as f:
                    json.dump(cfg, f, indent=2)
        if not self.close_all_tabs():
            return
        self.project_dir, self.config = folder, cfg
        self.config.setdefault("options", dict(DEFAULT_OPTIONS))
        self._load_options(dict(DEFAULT_OPTIONS, **self.config["options"]))
        self.world_combo.blockSignals(True)
        self.world_combo.setCurrentIndex(max(0, self.world_combo.findData(cfg.get("world", "pipeline"))))
        self.world_combo.blockSignals(False)
        self._world_changed()
        self.refresh_files()
        main = os.path.join(folder, cfg.get("main", ""))
        if os.path.isfile(main):
            self.open_file(main)
        self.project_label.setText(f"<b>{cfg.get('name')}</b><br>{folder}")
        self._add_recent(folder)
        self._update_title()

    def save_config(self):
        if not self.project_dir:
            return
        if os.path.isfile(os.path.join(self.project_dir, PROJECT_FILE)) or self.config:
            with open(os.path.join(self.project_dir, PROJECT_FILE), "w") as f:
                json.dump(self.config, f, indent=2)

    def refresh_files(self):
        self.files.blockSignals(True)
        self.files.clear()
        if self.project_dir:
            for f in sorted(os.listdir(self.project_dir)):
                if f.endswith(".py"):
                    it = QListWidgetItem(f + ("   (runs)" if f == self.config.get("main") else ""))
                    it.setData(Qt.UserRole, os.path.join(self.project_dir, f))
                    self.files.addItem(it)
                    if f == self.config.get("main"):
                        self.files.setCurrentItem(it)
        self.files.blockSignals(False)

    def _main_changed(self, cur, _prev):
        if cur is None or not self.project_dir:
            return
        main = os.path.basename(cur.data(Qt.UserRole))
        if main != self.config.get("main"):
            self.config["main"] = main
            self.save_config()
            QTimer.singleShot(0, self.refresh_files)       # not while the list emits this signal

    def new_script(self):
        if not self.project_dir:
            self.new_project()
            return
        world = self.world_combo.currentData()
        dlg = NewProjectDialog(self.labs_dir(), world, self, script_only=True)
        dlg.name.setText(f"{world.replace('-', '_')}_lab")
        if dlg.exec_() != QDialog.Accepted:
            return
        v = dlg.values()
        path = os.path.join(self.project_dir, generator.lab_file_name(v["name"]))
        if os.path.exists(path) and QMessageBox.question(self, APP_NAME, f"Overwrite {path}?") != QMessageBox.Yes:
            return
        try:
            text = generator.generate(world, v["name"], v["starter"], v["title"], self.labs_dir(),
                                      self.world_info(world))
        except Exception as exc:                       # noqa: BLE001
            QMessageBox.critical(self, APP_NAME, f"Cannot generate the script:\n{exc}")
            return
        self.close_path(path)
        with open(path, "w") as f:
            f.write(text)
        self.config["main"] = os.path.basename(path)
        self.save_config()
        self.refresh_files()
        self.open_file(path)

    # ---------------------------------------------------------------- world
    def world_info(self, key: str) -> Dict:
        if key not in self.world_infos:
            QApplication.setOverrideCursor(Qt.WaitCursor)
            try:
                self.world_infos[key] = world_info.describe(key)
            finally:
                QApplication.restoreOverrideCursor()
        return self.world_infos[key]

    def _world_changed(self, *_):
        key = self.world_combo.currentData()
        if not key:
            return
        try:
            info = self.world_info(key)
            text = world_info.summary_text(info) + "\n\n"
            if info["kind"] == "pipeline":
                text += "Arms (pick -> place, rule, bins):\n"
                for a in info["arms"]:
                    text += (f"  {a['name']:3s} {a['pick_conv']:3s} -> {a['place_conv']:3s} {a['rule']:10s} "
                             f"{', '.join(a['bins'])}\n")
                text += "Docks:\n" + "".join(f"  {d['name']:9s} {d['title']}\n" for d in info["docks"])
            else:
                text += "Bins: " + ", ".join(f"{k} ({v[0]:.1f}, {v[1]:.1f})" for k, v in info["bins"].items())
        except Exception as exc:                       # noqa: BLE001
            text = f"Cannot read the world {key}:\n{exc}"
        self.world_text.setPlainText(text)
        pipeline = world_info.entry(key).kind == "pipeline"
        self.opt["drawn_cameras"].setEnabled(pipeline)
        self.opt["fake"].setEnabled(pipeline)
        self.opt["world_products"].setEnabled(not pipeline)
        if self.project_dir and self.config.get("world") != key:
            self.config["world"] = key
            self.save_config()
            self._update_title()
            self._sync_world_line(key)

    def _sync_world_line(self, key: str):
        """Offer to set WORLD = '<key>' in the script that runs."""
        ed = self.editor_for(os.path.join(self.project_dir, self.config.get("main", "")))
        if ed is None:
            return
        text = ed.toPlainText()
        m = re.search(r"^WORLD = (.*)$", text, flags=re.M)
        if not m or m.group(1).strip() in (repr(key), f'"{key}"'):
            return
        if QMessageBox.question(self, APP_NAME, f"The world is now {key}. Set WORLD = {key!r} in "
                                f"{os.path.basename(ed.path)}?\n(For a script written for this world use "
                                f"'Generate script for this world'.)") == QMessageBox.Yes:
            c = ed.textCursor()
            c.beginEditBlock()
            c.setPosition(m.start(1))
            c.setPosition(m.end(1), QTextCursor.KeepAnchor)
            c.insertText(repr(key))
            c.endEditBlock()

    # ---------------------------------------------------------------- options
    def _load_options(self, opts: Dict):
        for k, w in self.opt.items():
            w.blockSignals(True)
            if isinstance(w, QCheckBox):
                w.setChecked(bool(opts.get(k, DEFAULT_OPTIONS[k])))
            else:
                w.setText(str(opts.get(k, DEFAULT_OPTIONS[k])))
            w.blockSignals(False)

    def options(self) -> Dict:
        return {k: (w.isChecked() if isinstance(w, QCheckBox) else w.text().strip()) for k, w in self.opt.items()}

    def _options_changed(self, *_):
        if self.project_dir:
            self.config["options"] = self.options()
            self.save_config()

    def _env(self) -> Dict[str, str]:
        env = {"PYTHONUNBUFFERED": "1", "FACTORY_PROJECT_DIR": self.sim_project}
        for tok in shlex.split(self.options()["env"] or ""):
            if "=" in tok:
                k, v = tok.split("=", 1)
                env[k] = v
        if self.project_dir:
            env["VISION_LAB_DIR"] = self.project_dir
        return env

    # ---------------------------------------------------------------- editors
    def editor_for(self, path: str) -> Optional[CodeEditor]:
        path = os.path.abspath(path)
        for i in range(self.tabs.count()):
            ed = self.tabs.widget(i)
            if os.path.abspath(ed.path) == path:
                return ed
        return None

    def open_file(self, path: str) -> Optional[CodeEditor]:
        ed = self.editor_for(path)
        if ed is None:
            try:
                ed = CodeEditor(path)
            except OSError as exc:
                QMessageBox.warning(self, APP_NAME, str(exc))
                return None
            ed.document().modificationChanged.connect(lambda _m, ed=ed: self._tab_label(ed))
            self.tabs.addTab(ed, os.path.basename(path))
            self.tabs.setTabToolTip(self.tabs.indexOf(ed), path)
        self.tabs.setCurrentWidget(ed)
        return ed

    def open_file_dialog(self):
        p, _ = QFileDialog.getOpenFileName(self, "Open script", self.project_dir or self.labs_dir(), "Python (*.py)")
        if p:
            self.open_file(p)

    def _tab_label(self, ed):
        i = self.tabs.indexOf(ed)
        if i >= 0:
            self.tabs.setTabText(i, os.path.basename(ed.path) + (" *" if ed.document().isModified() else ""))

    def save_current(self):
        ed = self.tabs.currentWidget()
        if ed is not None:
            ed.save()

    def save_all(self):
        for i in range(self.tabs.count()):
            ed = self.tabs.widget(i)
            if ed.document().isModified():
                ed.save()

    def close_tab(self, i) -> bool:
        ed = self.tabs.widget(i)
        if ed.document().isModified():
            r = QMessageBox.question(self, APP_NAME, f"Save {os.path.basename(ed.path)}?",
                                     QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
            if r == QMessageBox.Cancel:
                return False
            if r == QMessageBox.Save:
                ed.save()
        self.tabs.removeTab(i)
        ed.deleteLater()
        return True

    def close_path(self, path: str):
        ed = self.editor_for(path)
        if ed is not None:
            self.tabs.removeTab(self.tabs.indexOf(ed))
            ed.deleteLater()

    def close_all_tabs(self) -> bool:
        while self.tabs.count():
            if not self.close_tab(0):
                return False
        return True

    def jump_to(self, path: str, line: int):
        if not os.path.isabs(path) and self.project_dir:
            path = os.path.join(self.project_dir, path)
        if os.path.isfile(path):
            ed = self.open_file(path)
            if ed:
                ed.goto_line(line)

    def find(self):
        ed = self.tabs.currentWidget()
        if ed is None:
            return
        text, ok = QInputDialog.getText(self, "Find", "Text:", text=getattr(self, "_find_text", ""))
        if ok and text:
            self._find_text = text
            self.find_next()

    def find_next(self):
        ed = self.tabs.currentWidget()
        text = getattr(self, "_find_text", "")
        if ed is None or not text:
            return
        if not ed.find(text):
            ed.moveCursor(QTextCursor.Start)
            ed.find(text)

    # ---------------------------------------------------------------- build
    def main_script(self) -> Optional[str]:
        if not self.project_dir:
            return None
        p = os.path.join(self.project_dir, self.config.get("main", ""))
        return p if os.path.isfile(p) else None

    def check_scripts(self) -> bool:
        """Python syntax check of the project's scripts; errors go to the Build pane."""
        ok = True
        files = []
        if self.project_dir:
            files = [os.path.join(self.project_dir, f) for f in sorted(os.listdir(self.project_dir)) if f.endswith(".py")]
        for f in files:
            try:
                with open(f, encoding="utf-8") as fh:
                    compile(fh.read(), f, "exec")
            except SyntaxError as exc:
                ok = False
                self.build_pane.append(f'  File "{f}", line {exc.lineno}\n    {(exc.text or "").rstrip()}\n'
                                       f"SyntaxError: {exc.msg}\n", "#d0473f")
        if files:
            self.build_pane.note(f"Checked {len(files)} script(s): " + ("OK" if ok else "errors (double-click to jump)"))
        return ok

    def build(self, force: bool = False, then=None) -> bool:
        """Save, check the scripts, colcon build factory_bringup when needed; then() after a good build."""
        self.save_all()
        self.out_tabs.setCurrentWidget(self.build_pane)
        if self.build_pane.running():
            self.build_pane.note("A build is running already.")
            return False
        if not self.check_scripts():
            self.statusBar().showMessage("Build failed: Python errors in the scripts")
            return False
        stale = stale_install(self.sim_project)
        if not force and not stale:
            self.build_pane.note("factory_bringup is installed and up to date: no colcon build needed.")
            if then:
                then()
            return True
        if stale:
            self.build_pane.note(f"factory_bringup install is out of date ({len(stale)} file(s), e.g. "
                                 f"{', '.join(stale[:4])}): colcon build")
        ws = ws_dir_of(self.sim_project)
        distro = os.environ.get("ROS_DISTRO", "jazzy")
        cmd = (f"source /opt/ros/{distro}/setup.bash && cd {shlex.quote(ws)} && "
               f"colcon build --packages-select factory_bringup")
        self.build_pane.on_finished = [lambda code: self._built(code, then)]
        return self.build_pane.start(["bash", "-c", cmd], ws, self._env())

    def _built(self, code, then):
        self.build_pane.on_finished = []
        if code == 0:
            self.statusBar().showMessage("Build finished")
            if then:
                then()
        else:
            self.statusBar().showMessage("Build FAILED (see the Build pane)")
            self._pending_run = False

    # ---------------------------------------------------------------- run
    def _world(self) -> str:
        return self.world_combo.currentData()

    def _pipeline(self) -> bool:
        return world_info.entry(self._world()).kind == "pipeline"

    def sim_argv(self) -> List[str]:
        o = self.options()
        argv = ["bash", os.path.join(self.sim_project, "run_vision_lab.sh"), "sim", "--world", self._world()]
        if self._pipeline():
            if o["drawn_cameras"]:
                argv.append("--drawn-cameras")
        elif o["world_products"]:
            argv += ["--world-products", "order_gui:=false"]
        if o["headless"]:
            argv.append("--headless")
        if o["nvidia"]:
            argv.append("--nvidia")
        return argv + shlex.split(o["sim_args"] or "")

    def lab_argv(self, script: str) -> List[str]:
        o = self.options()
        argv = ["bash", os.path.join(self.sim_project, "run_vision_lab.sh"), "run", script, "--world", self._world()]
        if self._pipeline() and o["fake"]:
            argv.append("--fake")
        if not o["lab_window"]:
            argv.append("--no-window")
        return argv + shlex.split(o["lab_args"] or "")

    def _check_ready(self) -> bool:
        if not os.path.isfile(os.path.join(self.sim_project, "run_vision_lab.sh")):
            QMessageBox.warning(self, APP_NAME, f"run_vision_lab.sh not found in {self.sim_project}.\n"
                                                "File > Simulator project folder ... to choose it.")
            return False
        if not self.main_script():
            QMessageBox.information(self, APP_NAME, "Open or create a lab project first (File > New project).")
            return False
        return True

    def run_all(self):
        """Build, then the simulation (unless --fake or one is running), then the lab."""
        if not self._check_ready():
            return
        self._pending_run = True
        self.build(then=self._after_build_run)

    def _after_build_run(self):
        if not self._pending_run:
            return
        if self._pipeline() and self.options()["fake"]:
            self._pending_run = False
            self.run_lab(skip_build=True)
            return
        if self.sim_pane.running() or self._gazebo_running():
            self.lab_pane.note("The simulation is running already: starting the lab.")
            self._pending_run = False
            self.run_lab(skip_build=True)
            return
        self.start_sim(wait_then_lab=True)

    def _gazebo_running(self) -> bool:
        if not shutil.which("pgrep"):
            return False
        p = QProcess()
        p.start("pgrep", ["-f", "gz sim"])
        p.waitForFinished(3000)
        return p.exitCode() == 0

    def start_sim(self, wait_then_lab: bool = False):
        if not self._check_ready():
            return
        if self.sim_pane.running():
            self.sim_pane.note("The simulation is running already.")
            return
        if self._gazebo_running():
            r = QMessageBox.question(self, APP_NAME, "Gazebo is already running (outside the IDE?). Use it?\n"
                                     "Yes = run the lab on it, No = start nothing (Run > Kill leftover Gazebo).")
            if r == QMessageBox.Yes and wait_then_lab:
                self._pending_run = False
                self.run_lab(skip_build=True)
            return
        self.out_tabs.setCurrentWidget(self.sim_pane)
        if not self.sim_pane.start(self.sim_argv(), self.sim_project, self._env()):
            return
        if wait_then_lab:
            self.sim_pane.note("Waiting for the world to come up, then the lab starts ...")
            self._wait_started = 0.0
            self._wait_timer.start(2000)

    def _poll_sim(self):
        """Is the world up? pipeline: its Gazebo services; single-arm worlds: the overhead camera topic."""
        self._wait_started += 2.0
        if not self.sim_pane.running():
            self._wait_timer.stop()
            self._pending_run = False
            return
        if self._probe is not None and self._probe.state() != QProcess.NotRunning:
            return
        if self._wait_started > 180:
            self._wait_timer.stop()
            self.sim_pane.note("The world did not report ready within 180 s; starting the lab anyway.")
            self._start_lab_after_sim()
            return
        info = self.world_info(self._world())
        if self._pipeline():
            cmd = "gz service -l 2>/dev/null"
            want = f"/world/{info['gazebo_world']}/"
        else:
            distro = os.environ.get("ROS_DISTRO", "jazzy")
            ws = ws_dir_of(self.sim_project)
            cmd = (f"source /opt/ros/{distro}/setup.bash >/dev/null 2>&1; "
                   f"source {shlex.quote(ws)}/install/setup.bash >/dev/null 2>&1; ros2 topic list 2>/dev/null")
            want = "/overhead_camera/image"
        self._probe = QProcess(self)

        def done(*_a, p=self._probe):
            out = bytes(p.readAllStandardOutput()).decode("utf-8", "replace")
            if want in out and self._wait_timer.isActive():
                self._wait_timer.stop()
                self.sim_pane.note("The world is up; the lab starts in 5 s.")
                QTimer.singleShot(5000, self._start_lab_after_sim)
        self._probe.finished.connect(done)
        self._probe.start("bash", ["-c", cmd])

    def _start_lab_after_sim(self):
        if self._pending_run and self.sim_pane.running():
            self._pending_run = False
            self.run_lab(skip_build=True)

    def _sim_finished(self, code):
        self._wait_timer.stop()
        self._pending_run = False
        self.statusBar().showMessage(f"Simulation finished (exit {code})")

    def run_lab(self, skip_build: bool = False):
        if not self._check_ready():
            return
        if not skip_build:
            self.save_all()
            if not self.check_scripts():
                self.out_tabs.setCurrentWidget(self.build_pane)
                return
        script = self.main_script()
        self.out_tabs.setCurrentWidget(self.lab_pane)
        self.lab_pane.start(self.lab_argv(script), self.project_dir, self._env())

    def stop_all(self):
        self._pending_run = False
        self._wait_timer.stop()
        for p in (self.lab_pane, self.sim_pane, self.build_pane):
            p.stop()

    def kill_gazebo(self):
        if QMessageBox.question(self, APP_NAME, "Kill every 'gz sim' process on this computer?") == QMessageBox.Yes:
            QProcess.execute("pkill", ["-f", "gz sim"])
            self.sim_pane.note("pkill -f 'gz sim' sent.")

    def choose_sim_project(self):
        d = QFileDialog.getExistingDirectory(self, "Simulator project folder (with run_vision_lab.sh)",
                                             self.sim_project)
        if d and os.path.isfile(os.path.join(d, "run_vision_lab.sh")):
            self.sim_project = d
            self.settings.setValue("sim_project", d)
            self._update_title()
        elif d:
            QMessageBox.warning(self, APP_NAME, f"No run_vision_lab.sh in {d}.")

    def show_help(self):
        QMessageBox.information(self, APP_NAME, __doc__)

    def closeEvent(self, e):
        running = [p.title for p in (self.lab_pane, self.sim_pane, self.build_pane) if p.running()]
        if running and QMessageBox.question(self, APP_NAME, f"Stop {', '.join(running)} and quit?") != QMessageBox.Yes:
            e.ignore()
            return
        if not self.close_all_tabs():
            e.ignore()
            return
        self.stop_all()
        for p in (self.lab_pane, self.sim_pane, self.build_pane):
            if p.running():
                p.proc.waitForFinished(4000)
                p._signal(signal.SIGKILL)
        e.accept()


def main(argv=None):
    argv = list(sys.argv if argv is None else argv)
    app = QApplication(argv[:1])
    app.setApplicationName(APP_NAME)
    project = argv[1] if len(argv) > 1 and not argv[1].startswith("-") else None
    w = LabIDE(project)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
