"""Shared terminal presentation only; no ROS, service or replay operations."""

import curses
import os
import unicodedata


def clip(text, columns):
    result, used = "", 0
    for char in str(text):
        if unicodedata.category(char).startswith("C"):
            char = " "
        width = 0 if unicodedata.combining(char) else (
            2 if unicodedata.east_asian_width(char) in ("W", "F") else 1)
        if used + width > columns:
            break
        result += char
        used += width
    return result


class ConsolePanel:
    def __init__(self, screen, brand="ROBOT  /  控制台"):
        self.screen, self.brand, self.colors = screen, brand, {}
        try:
            if curses.has_colors():
                curses.start_color()
                curses.use_default_colors()
                extended = curses.COLORS >= 256
                palette = (
                    ("accent", 81 if extended else curses.COLOR_CYAN, -1),
                    ("selected", 255 if extended else curses.COLOR_WHITE,
                     24 if extended else curses.COLOR_BLUE),
                    ("good", 114 if extended else curses.COLOR_GREEN, -1),
                    ("warn", 221 if extended else curses.COLOR_YELLOW, -1),
                    ("error", 203 if extended else curses.COLOR_RED, -1),
                    ("muted", 245 if extended else curses.COLOR_WHITE, -1),
                    ("card", 255 if extended else curses.COLOR_WHITE,
                     235 if extended else -1),
                )
                for number, (name, foreground, background) in enumerate(palette, 1):
                    curses.init_pair(number, foreground, background)
                    self.colors[name] = curses.color_pair(number)
        except curses.error:
            pass
        self.screen.keypad(True)
        try:
            curses.curs_set(0)
        except curses.error:
            pass

    def color(self, name):
        return self.colors.get(name, 0)

    def line(self, row, text, attr=0, col=1, width=None):
        height, screen_width = self.screen.getmaxyx()
        width = min(width or screen_width, screen_width - col - 1)
        if 0 <= row < height and width > 0:
            try:
                self.screen.addstr(row, col, clip(text, width), attr)
            except curses.error:
                pass  # Resize can race with painting; redraw on the next event.

    def panel(self, title, progress="", footer="↑↓ 选择    Enter 确认    ←/Esc 返回",
              tone="accent"):
        self.screen.erase()
        height, width = self.screen.getmaxyx()
        if height < 20 or width < 64:
            self.line(2, "请放大终端：至少 64 列 × 20 行。", curses.A_BOLD)
            self.line(4, "Q / Ctrl-C 返回；不会停止后台任务。")
            return None
        panel_width, panel_height = min(110, width - 4), min(32, height - 2)
        left, top = (width - panel_width) // 2, (height - panel_height) // 2
        right, bottom = left + panel_width - 1, top + panel_height - 1
        border = self.color("muted")
        self.line(top, "╭" + "─" * (panel_width - 2) + "╮", border, left)
        self.line(bottom, "╰" + "─" * (panel_width - 2) + "╯", border, left)
        for row in range(top + 1, bottom):
            self.line(row, "│", border, left)
            self.line(row, "│", border, right)
        self.line(top + 1, self.brand, self.color("accent") | curses.A_BOLD,
                  left + 3, panel_width - 21)
        self.line(top + 1, progress, self.color("muted"), right - 15, 13)
        tone = "error" if "失败" in title else "good" if "通过" in title else tone
        self.line(top + 3, title, curses.A_BOLD | self.color(tone), left + 3, panel_width - 6)
        self.line(bottom - 2, footer, self.color("muted"), left + 3, panel_width - 6)
        return left + 3, top + 5, panel_width - 6, bottom - 4

    def card(self, row, col, width, label, value):
        attr = self.color("card")
        for y in range(row, row + 4):
            self.line(y, " " * width, attr, col, width)
        self.line(row + 1, label, attr | curses.A_DIM, col + 2, width - 4)
        self.line(row + 2, value, attr | curses.A_BOLD, col + 2, width - 4)


class ReplayDashboard(ConsolePanel):
    """Full-screen view/input adapter; command eligibility stays in the client."""

    def __init__(self):
        self.screen = None
        self.notice = "等待回放节点就绪；不会自动播放。"

    def __enter__(self):
        try:
            screen = curses.initscr()
            self.screen = screen
            curses.noecho()
            curses.cbreak()
            super().__init__(screen, "ROBOT  /  回放")
            screen.timeout(50)
            return self
        except BaseException:
            self.__exit__()
            raise

    def __exit__(self, *_):
        if self.screen is not None:
            try:
                self.screen.keypad(False)
                curses.nocbreak()
                curses.echo()
                curses.endwin()
            except curses.error:
                pass

    def read(self):
        key = self.screen.getch()
        command = {ord(" "): "toggle", curses.KEY_DOWN: "step",
                   curses.KEY_RIGHT: "fast_forward", ord("q"): "quit",
                   ord("Q"): "quit", 3: "quit"}.get(key)
        return [command] if command else []

    def draw(self, state, connected):
        name = state["state"] if state and connected else "connecting"
        label, tone = {
            "connecting": ("连接中", "warn"), "initializing": ("初始化中", "warn"),
            "playing": ("正在播放", "good"), "paused": ("已暂停", "warn"),
            "pause_requested": ("正在暂停", "warn"), "stepping": ("单帧处理中", "accent"),
            "fast_forward": ("正在前进", "accent"), "finishing": ("收尾校验中", "accent"),
            "finished": ("已到结尾", "good"), "error": ("回放错误", "error"),
        }[name]
        area = self.panel("●  " + label, "ROS " + os.environ.get("ROS_DOMAIN_ID", "?"),
                          "空格 播放/暂停   ↓ 单帧   → 前进   Q 菜单", tone)
        if area:
            left, row, width, bottom = area
            count = state["acknowledged"] if state else 0
            total = state["total_targets"] if state else 0
            ratio = min(1.0, count / total) if total else 0.0
            frame = state.get("frame_index") if state else None
            time_text = f"{state['position_sec']:.3f} s" if state else "—"
            values = (("原始帧号", str(frame) if frame is not None else "—"),
                      ("视频时间", time_text), ("VIO 已处理", str(count)))
            card_width = (width - 4) // 3
            for index, (title, value) in enumerate(values):
                self.card(row, left + index * (card_width + 2), card_width, title, value)
            self.line(row + 5, f"VIO {count} / {total}     {100 * ratio:.1f}%",
                      self.color("muted"), left, width)
            fill = round(width * ratio)
            self.line(row + 6, "━" * width, self.color("muted"), left, width)
            if fill:
                self.line(row + 6, "━" * fill, self.color("accent"), left, width)
            note = self.notice
            if not connected and state:
                note = "连接暂不可用；上方为最后收到的进度。"
            self.line(bottom, note, self.color("muted"), left, width)
        self.screen.refresh()
