#!/usr/bin/env python3
"""
mac_client.py — AR Explorer Remote Control
==========================================
Tkinter GUI for placing USDZ models and AR markers on the iPhone via WebSocket.

When the GUI connects, the iPhone automatically sends the list of bundled
.usdz model names, which populate the clickable model buttons.

Requirements:
    pip install websockets          # tested with websockets >= 10.0

Usage:
    python mac_client.py <iphone-ip> [port]
    python mac_client.py 172.20.10.1
    python mac_client.py 172.20.10.1 8080

The iPhone's IP and port are shown in the AR Explorer HUD.
"""

import asyncio
import json
import queue
import sys
import threading
import tkinter as tk
from tkinter import ttk, font as tkfont
from typing import Optional

try:
    import websockets
except ImportError:
    print("Error: 'websockets' library not found.")
    print("Install it with:  pip install websockets")
    sys.exit(1)


# ============================================================================
# WebSocket worker — background thread with its own asyncio event loop
# ============================================================================

class WSWorker:
    """
    Manages a WebSocket connection in a dedicated asyncio event loop running
    in a background daemon thread.

    Thread-safe API:
        worker.send(dict)       — GUI → iPhone  (may be called from any thread)
        worker.receive_queue    — queue.Queue of dicts; poll from the GUI thread

    Internal control dicts placed on receive_queue (never sent over the wire):
        {"_status": "connecting"}
        {"_status": "connected"}
        {"_status": "disconnected"}
        {"_status": "error", "_msg": "..."}
    """

    def __init__(self, uri: str) -> None:
        self.uri = uri
        self.receive_queue: queue.Queue = queue.Queue()

        self._loop = asyncio.new_event_loop()
        self._send_q: Optional[asyncio.Queue] = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def send(self, msg: dict) -> None:
        """Schedule a JSON message to be sent. Safe to call from any thread."""
        if self._send_q is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._send_q.put_nowait, msg)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connect())

    async def _connect(self) -> None:
        self._send_q = asyncio.Queue()
        self.receive_queue.put({"_status": "connecting"})
        try:
            async with websockets.connect(self.uri, open_timeout=10) as ws:
                self.receive_queue.put({"_status": "connected"})
                await asyncio.gather(self._sender(ws), self._receiver(ws))
        except ConnectionRefusedError:
            self.receive_queue.put({
                "_status": "error",
                "_msg": "Connection refused — is AR Explorer running on the iPhone?",
            })
        except (TimeoutError, asyncio.TimeoutError):
            self.receive_queue.put({
                "_status": "error",
                "_msg": "Connection timed out — check the IP and that both devices share a network",
            })
        except OSError as e:
            self.receive_queue.put({"_status": "error", "_msg": str(e)})
        except Exception as e:
            self.receive_queue.put({"_status": "error", "_msg": str(e)})
        finally:
            self.receive_queue.put({"_status": "disconnected"})

    async def _sender(self, ws) -> None:
        assert self._send_q is not None
        while True:
            msg = await self._send_q.get()
            if msg is None:
                return
            try:
                await ws.send(json.dumps(msg))
            except Exception:
                return

    async def _receiver(self, ws) -> None:
        try:
            async for raw in ws:
                try:
                    self.receive_queue.put(json.loads(raw))
                except json.JSONDecodeError:
                    pass
        except Exception:
            pass


# ============================================================================
# Tkinter GUI
# ============================================================================

COLORS = ["green", "red", "blue", "yellow", "orange", "purple", "white", "cyan", "pink"]
COLS_PER_ROW = 4          # model buttons per row
WINDOW_MIN_WIDTH = 470    # minimum window width in pixels


class App:
    """Main application window."""

    def __init__(self, root: tk.Tk, uri: str) -> None:
        self.root = root
        self.uri = uri
        self.worker: Optional[WSWorker] = None
        self.selected_model: Optional[str] = None
        self.model_buttons: dict[str, tk.Button] = {}

        # Fonts — created once, reused for all model buttons
        self._normal_font = tkfont.Font(size=10)
        self._bold_font   = tkfont.Font(size=10, weight="bold")

        root.title("AR Explorer — Remote Control")
        root.resizable(False, False)
        root.minsize(WINDOW_MIN_WIDTH, 0)

        self._build_ui()

        # Start the polling loop before the first connection so it is always
        # running regardless of how many times _connect() is called later.
        self._poll_queue()
        self._connect()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        # ── 1. Models ─────────────────────────────────────────────────────────
        models_lf = ttk.LabelFrame(main, text="Models", padding=6)
        models_lf.pack(fill="x", pady=(0, 8))

        # Hint label shown below the section title
        ttk.Label(
            models_lf,
            text="Click to select · Long-press the AR surface on iPhone to place",
            foreground="gray",
            font=tkfont.Font(size=9),
        ).pack(anchor="w", padx=2, pady=(0, 4))

        self.models_inner = ttk.Frame(models_lf)
        self.models_inner.pack(fill="x")

        # Placeholder shown before model_list arrives
        self._waiting_label = ttk.Label(
            self.models_inner,
            text="Waiting for model list from iPhone…",
            foreground="gray",
        )
        self._waiting_label.pack(padx=4, pady=4)

        # ── 2. Placement ──────────────────────────────────────────────────────
        place_lf = ttk.LabelFrame(main, text="Placement", padding=6)
        place_lf.pack(fill="x", pady=(0, 8))

        grid = ttk.Frame(place_lf)
        grid.pack(fill="x")

        # Helper: label + entry cell
        def row_entry(parent, label_text, default, r, c):
            ttk.Label(parent, text=label_text).grid(
                row=r, column=c, sticky="e", padx=(0, 4), pady=3)
            e = ttk.Entry(parent, width=8)
            e.insert(0, default)
            e.grid(row=r, column=c + 1, sticky="w", padx=(0, 16), pady=3)
            return e

        self.e_ahead = row_entry(grid, "Ahead (m):",      "3.0", 0, 0)
        self.e_lr    = row_entry(grid, "Left / Right (m):", "0.0", 0, 2)
        self.e_ud    = row_entry(grid, "Up / Down (m):",    "0.0", 1, 0)
        self.e_scale = row_entry(grid, "Scale:",            "1.0", 1, 2)

        ttk.Label(grid, text="Label (optional):").grid(
            row=2, column=0, sticky="e", padx=(0, 4), pady=3)
        self.e_label = ttk.Entry(grid, width=32)
        self.e_label.grid(row=2, column=1, columnspan=3, sticky="ew", pady=3)
        grid.columnconfigure(3, weight=1)

        # Quick placement buttons
        quick = ttk.Frame(place_lf)
        quick.pack(fill="x", pady=(4, 2))
        ttk.Button(quick, text="Place 3 m Ahead",
                   command=lambda: self._place_model_quick(3.0)).pack(side="left", padx=(0, 6))
        ttk.Button(quick, text="Place 5 m Ahead",
                   command=lambda: self._place_model_quick(5.0)).pack(side="left")

        # Big "Place Model" button
        ttk.Button(place_lf, text="Place Model",
                   command=self._place_model).pack(fill="x", ipady=5, pady=(6, 0))

        # ── 3. Simple Marker ──────────────────────────────────────────────────
        marker_lf = ttk.LabelFrame(
            main, text="Simple Marker  (uses same coordinates above)", padding=6)
        marker_lf.pack(fill="x", pady=(0, 8))

        color_row = ttk.Frame(marker_lf)
        color_row.pack(fill="x")
        ttk.Label(color_row, text="Color:").pack(side="left", padx=(0, 6))
        self.color_var = tk.StringVar(value="green")
        ttk.Combobox(
            color_row,
            textvariable=self.color_var,
            values=COLORS,
            state="readonly",
            width=10,
        ).pack(side="left")

        ttk.Button(marker_lf, text="Place Marker",
                   command=self._place_marker).pack(fill="x", ipady=5, pady=(6, 0))

        # ── 4. Clear All ──────────────────────────────────────────────────────
        ttk.Button(main, text="Clear All",
                   command=self._clear_all).pack(fill="x", ipady=5, pady=(0, 4))

        # ── 5. Status bar ─────────────────────────────────────────────────────
        ttk.Separator(self.root, orient="horizontal").pack(fill="x")
        status_bar = ttk.Frame(self.root, padding=(8, 5))
        status_bar.pack(fill="x")

        # Coloured dot (orange / green / red)
        self._dot = tk.Label(status_bar, text="●", fg="orange", font=("", 13))
        self._dot.pack(side="left", padx=(0, 5))

        self._status_var = tk.StringVar(value=f"Connecting to {self.uri}…")
        ttk.Label(status_bar, textvariable=self._status_var).pack(side="left")

        # Reconnect button — always in the layout, disabled while connected
        self._reconnect_btn = ttk.Button(
            status_bar, text="Reconnect",
            command=self._connect, state="disabled")
        self._reconnect_btn.pack(side="right")

    # ── Model button management ───────────────────────────────────────────────

    def _populate_models(self, names: list) -> None:
        """Replace model button area with one button per model name."""
        for w in self.models_inner.winfo_children():
            w.destroy()
        self.model_buttons.clear()
        self.selected_model = None

        if not names:
            ttk.Label(
                self.models_inner,
                text="No .usdz files bundled in the app. "
                     "Drag .usdz files into Xcode → ARExplorer target.",
                foreground="gray",
            ).pack(padx=4, pady=4)
            return

        btn_frame = ttk.Frame(self.models_inner)
        btn_frame.pack(fill="x")

        for i, name in enumerate(names):
            row, col = divmod(i, COLS_PER_ROW)
            btn = tk.Button(
                btn_frame,
                text=name,
                font=self._normal_font,
                relief="raised",
                bd=2,
                padx=10,
                pady=5,
                cursor="hand2",
                command=lambda n=name: self._select_model(n),
            )
            btn.grid(row=row, column=col, padx=3, pady=3, sticky="ew")
            btn_frame.columnconfigure(col, weight=1)
            self.model_buttons[name] = btn

        # Auto-select the first model
        self._select_model(names[0])

    def _select_model(self, name: str) -> None:
        """Highlight the selected model button; deselect all others."""
        for n, btn in self.model_buttons.items():
            btn.config(text=n, font=self._normal_font, relief="raised")
        self.selected_model = name
        self.model_buttons[name].config(
            text=f"✓  {name}",
            font=self._bold_font,
            relief="sunken",
        )

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _fval(self, entry: ttk.Entry, fallback: float = 0.0) -> float:
        try:
            return float(entry.get())
        except ValueError:
            return fallback

    def _get_xyz(self) -> tuple:
        """Convert GUI inputs to ARKit camera-space coordinates.

        ARKit convention:  -z = forward,  +x = right,  +y = up
        """
        return (
            self._fval(self.e_lr,    0.0),    # x: left/right
            self._fval(self.e_ud,    0.0),    # y: up/down
            -self._fval(self.e_ahead, 3.0),   # z: -ahead = in front
        )

    # ── Button actions ────────────────────────────────────────────────────────

    def _place_model(self) -> None:
        if not self.selected_model:
            self._status_var.set("⚠  Select a model first")
            return
        x, y, z = self._get_xyz()
        label = self.e_label.get().strip()
        self.worker.send({
            "action": "place_model",
            "model":  self.selected_model,
            "x": x, "y": y, "z": z,
            "label": label,
            "scale": self._fval(self.e_scale, 1.0),
        })

    def _place_model_quick(self, dist: float) -> None:
        if not self.selected_model:
            self._status_var.set("⚠  Select a model first")
            return
        label = self.e_label.get().strip() or self.selected_model
        self.worker.send({
            "action": "place_model",
            "model":  self.selected_model,
            "x": 0.0, "y": 0.0, "z": -dist,
            "label": label,
            "scale": self._fval(self.e_scale, 1.0),
        })

    def _place_marker(self) -> None:
        x, y, z = self._get_xyz()
        self.worker.send({
            "action": "place",
            "x": x, "y": y, "z": z,
            "label": self.e_label.get().strip(),
            "color": self.color_var.get(),
        })

    def _clear_all(self) -> None:
        self.worker.send({"action": "clear"})

    # ── Connection management ─────────────────────────────────────────────────

    def _connect(self) -> None:
        """Create and start a new WSWorker. Safe to call for initial connect
        and for reconnect after a disconnect."""
        self._reconnect_btn.config(state="disabled")
        self._dot.config(fg="orange")
        self._status_var.set(f"Connecting to {self.uri}…")

        self.worker = WSWorker(self.uri)
        self.worker.start()

    # ── Queue polling (runs continuously via root.after) ─────────────────────

    def _poll_queue(self) -> None:
        if self.worker is not None:
            try:
                while True:
                    self._handle(self.worker.receive_queue.get_nowait())
            except queue.Empty:
                pass
        self.root.after(100, self._poll_queue)

    def _handle(self, msg: dict) -> None:
        """Dispatch an incoming dict from the receive_queue."""
        st = msg.get("_status")

        if st == "connecting":
            self._dot.config(fg="orange")
            self._status_var.set(f"Connecting to {self.uri}…")

        elif st == "connected":
            self._dot.config(fg="green")
            self._status_var.set(f"Connected to {self.uri}")
            self._reconnect_btn.config(state="disabled")

        elif st == "error":
            self._dot.config(fg="red")
            self._status_var.set(msg.get("_msg", "Connection error"))
            # "disconnected" will follow immediately; Reconnect enabled there

        elif st == "disconnected":
            self._dot.config(fg="red")
            # Only overwrite status text if it wasn't set by an error already
            if self._status_var.get().startswith("Connected"):
                self._status_var.set("Disconnected")
            self._reconnect_btn.config(state="normal")

        elif msg.get("action") == "model_list":
            self._populate_models(msg.get("models", []))


# ============================================================================
# Entry point
# ============================================================================

def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python mac_client.py <iphone-ip> [port]")
        print("  The iPhone's IP and port are shown in the AR Explorer HUD.")
        print("  Example: python mac_client.py 172.20.10.1")
        sys.exit(1)

    ip   = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
    uri  = f"ws://{ip}:{port}"

    root = tk.Tk()
    App(root, uri)
    root.mainloop()


if __name__ == "__main__":
    main()
