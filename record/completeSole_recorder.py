import asyncio
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from bleak import BleakScanner, BleakClient

# =========================
# Your ESP32 settings
# =========================
DEVICE_NAME = "ESP32-Insole"
SERVICE_UUID = "12345678-1234-1234-1234-123456789012"
CHAR_UUID    = "abcdef12-3456-789a-bcde-123456789abc"

# =========================
# Utilities
# =========================
def now_ts():
    return time.strftime("%Y-%m-%d_%H-%M-%S")

def pc_epoch_ms() -> int:
    return int(time.time() * 1000)

def is_header(line: str) -> bool:
    return line.startswith("seq,t_ms,")

def is_status(line: str) -> bool:
    return line.startswith("STATUS,")

def is_data_line(line: str) -> bool:
    if not line:
        return False
    if is_header(line) or is_status(line):
        return False
    parts = line.strip().split(",")
    if len(parts) < 2:
        return False
    return parts[0].isdigit() and parts[1].isdigit()

def parse_seq(line: str) -> int | None:
    try:
        return int(line.split(",", 1)[0])
    except Exception:
        return None

def patch_header(line: str) -> str:
    s = line.rstrip("\n")
    if s.endswith(",pc_ts_ms,drop_seq"):
        return s + "\n"
    return s + ",pc_ts_ms,drop_seq\n"

def append_cols(line: str, pc_ms: int, drop_seq: int) -> str:
    s = line.rstrip("\n")
    return f"{s},{pc_ms},{drop_seq}\n"


@dataclass
class StreamState:
    connected: bool = False
    recording: bool = False

    bytes_rx: int = 0
    lines_rx: int = 0

    buffer: bytearray = field(default_factory=bytearray)

    recorded_lines: list[str] = field(default_factory=list)
    last_lines: list[str] = field(default_factory=list)

    last_seq: int | None = None
    total_drop_seq: int = 0
    header_written: bool = False


class BleWorker:
    def __init__(self, ui_event_q: "queue.Queue[tuple]"):
        self.ui_event_q = ui_event_q
        self.loop: asyncio.AbstractEventLoop | None = None
        self.thread: threading.Thread | None = None
        self.client: BleakClient | None = None
        self.state = StreamState()
        self._stop_event = threading.Event()

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop_event.set()
        if self.loop:
            self.loop.call_soon_threadsafe(lambda: None)

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._run_forever())
        finally:
            try:
                pending = asyncio.all_tasks(loop=self.loop)
                for t in pending:
                    t.cancel()
                self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self.loop.close()

    async def _run_forever(self):
        while not self._stop_event.is_set():
            await asyncio.sleep(0.1)
        try:
            await self._disconnect_internal()
        except Exception:
            pass

    def submit(self, coro_func, *args):
        if not self.loop:
            raise RuntimeError("BLE loop not started")
        return asyncio.run_coroutine_threadsafe(coro_func(*args), self.loop)

    async def scan(self, timeout_s: float = 5.0):
        self.ui_event_q.put(("log", f"Scanning {timeout_s:.1f}s ..."))
        devices = await BleakScanner.discover(timeout=timeout_s)

        results = []
        for d in devices:
            name = d.name or ""
            addr = d.address
            uuids = []
            try:
                md = getattr(d, "metadata", {}) or {}
                uuids = md.get("uuids", []) or []
            except Exception:
                uuids = []
            results.append((name, addr, uuids))

        def score(item):
            name, addr, uuids = item
            s = 0
            if name == DEVICE_NAME:
                s += 100
            if SERVICE_UUID.lower() in [u.lower() for u in (uuids or [])]:
                s += 50
            if DEVICE_NAME.lower() in name.lower():
                s += 25
            return -s

        results.sort(key=score)
        self.ui_event_q.put(("scan_results", results))

    async def connect(self, address: str):
        await self._disconnect_internal()

        self.ui_event_q.put(("log", f"Connecting to {address} ..."))
        client = BleakClient(address)
        try:
            await client.connect(timeout=15.0)
        except Exception as e:
            self.ui_event_q.put(("error", f"Connect failed: {e}"))
            return

        self.client = client
        self.state = StreamState(connected=True)
        self.ui_event_q.put(("connected", True))

        try:
            await client.start_notify(CHAR_UUID, self._on_notify)
        except Exception as e:
            self.ui_event_q.put(("error", f"Notify subscribe failed: {e}"))
            await self._disconnect_internal()
            return

        self.ui_event_q.put(("log", "Subscribed to notifications."))

    async def disconnect(self):
        await self._disconnect_internal()

    async def _disconnect_internal(self):
        if self.client:
            try:
                self.ui_event_q.put(("log", "Disconnecting ..."))
                await self.client.stop_notify(CHAR_UUID)
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception:
                pass

        self.client = None
        was_connected = self.state.connected
        self.state.connected = False
        self.state.recording = False
        if was_connected:
            self.ui_event_q.put(("connected", False))

    def set_recording(self, enabled: bool):
        self.state.recording = enabled
        self.ui_event_q.put(("recording", enabled))

    def clear_recording(self):
        st = self.state
        st.recorded_lines.clear()
        st.lines_rx = 0
        st.bytes_rx = 0
        st.buffer.clear()
        st.last_lines.clear()
        st.last_seq = None
        st.total_drop_seq = 0
        st.header_written = False
        self.ui_event_q.put(("log", "Cleared buffer/recording."))

    def _on_notify(self, sender: int, data: bytearray):
        st = self.state
        st.bytes_rx += len(data)
        st.buffer.extend(data)

        while True:
            nl = st.buffer.find(b"\n")
            if nl < 0:
                break
            line_bytes = st.buffer[: nl + 1]
            del st.buffer[: nl + 1]

            try:
                line = line_bytes.decode("utf-8", errors="replace")
            except Exception:
                line = str(line_bytes)

            st.lines_rx += 1

            st.last_lines.append(line.rstrip("\n"))
            if len(st.last_lines) > 12:
                st.last_lines = st.last_lines[-12:]

            if st.recording:
                pc_ms = pc_epoch_ms()

                if is_header(line) and not st.header_written:
                    st.recorded_lines.append(patch_header(line))
                    st.header_written = True
                    st.last_seq = None
                    continue

                if is_data_line(line):
                    seq = parse_seq(line)
                    drop_seq = 0
                    if seq is not None and st.last_seq is not None:
                        expected = st.last_seq + 1
                        if seq > expected:
                            drop_seq = seq - expected
                            st.total_drop_seq += drop_seq
                    if seq is not None:
                        st.last_seq = seq
                    st.recorded_lines.append(append_cols(line, pc_ms, drop_seq))
                else:
                    st.recorded_lines.append(append_cols(line, pc_ms, 0))

        if st.lines_rx % 10 == 0:
            self.ui_event_q.put(("stats", st.bytes_rx, st.lines_rx, list(st.last_lines), st.total_drop_seq))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Insole , BLE CSV Recorder (auto-save on stop)")
        self.geometry("950x620")

        self.ui_event_q: "queue.Queue[tuple]" = queue.Queue()
        self.ble = BleWorker(self.ui_event_q)
        self.ble.start()

        self.selected_addr: str | None = None
        self.scan_results: list[tuple[str, str, list[str]]] = []

        self._build_ui()
        self.after(100, self._poll_events)

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        top = ttk.Frame(self)
        top.pack(fill="x", **pad)

        self.btn_scan = ttk.Button(top, text="Scan", command=self.on_scan)
        self.btn_scan.pack(side="left")

        self.btn_connect = ttk.Button(top, text="Connect", command=self.on_connect, state="disabled")
        self.btn_connect.pack(side="left", padx=6)

        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self.on_disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", padx=6)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=10)

        self.btn_start = ttk.Button(top, text="Start Recording", command=self.on_start_recording, state="disabled")
        self.btn_start.pack(side="left")

        self.btn_stop = ttk.Button(top, text="Stop & Save...", command=self.on_stop_and_save, state="disabled")
        self.btn_stop.pack(side="left", padx=6)

        self.btn_clear = ttk.Button(top, text="Clear", command=self.on_clear, state="disabled")
        self.btn_clear.pack(side="left", padx=6)

        status = ttk.Frame(self)
        status.pack(fill="x", **pad)

        self.lbl_conn = ttk.Label(status, text="Disconnected")
        self.lbl_conn.pack(side="left")

        self.lbl_rec = ttk.Label(status, text="Recording: OFF")
        self.lbl_rec.pack(side="left", padx=20)

        self.lbl_stats = ttk.Label(status, text="Bytes: 0 , Lines: 0 , DropSeq: 0")
        self.lbl_stats.pack(side="left", padx=20)

        mid = ttk.PanedWindow(self, orient="horizontal")
        mid.pack(fill="both", expand=True, **pad)

        left = ttk.Frame(mid)
        right = ttk.Frame(mid)
        mid.add(left, weight=2)
        mid.add(right, weight=3)

        ttk.Label(left, text="Scan results (name , address , uuids)").pack(anchor="w")
        self.lst = tk.Listbox(left, height=18)
        self.lst.pack(fill="both", expand=True)
        self.lst.bind("<<ListboxSelect>>", self.on_select_device)

        ttk.Label(right, text="Live preview (last lines , raw incoming)").pack(anchor="w")
        self.txt_preview = tk.Text(right, height=18, wrap="none")
        self.txt_preview.pack(fill="both", expand=True)

        bottom = ttk.Frame(self)
        bottom.pack(fill="both", expand=True, **pad)

        ttk.Label(bottom, text="Log").pack(anchor="w")
        self.txt_log = tk.Text(bottom, height=10, wrap="word")
        self.txt_log.pack(fill="both", expand=True)

    # ---------- GUI actions ----------
    def on_scan(self):
        self.btn_connect.config(state="disabled")
        self.selected_addr = None
        self.lst.delete(0, tk.END)
        self._log("Scan requested.")
        try:
            self.ble.submit(self.ble.scan, 5.0)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_select_device(self, _evt=None):
        sel = self.lst.curselection()
        if not sel:
            self.selected_addr = None
            self.btn_connect.config(state="disabled")
            return
        idx = sel[0]
        try:
            _name, addr, _uuids = self.scan_results[idx]
        except Exception:
            self.selected_addr = None
            self.btn_connect.config(state="disabled")
            return
        self.selected_addr = addr
        self.btn_connect.config(state="normal")

    def on_connect(self):
        if not self.selected_addr:
            return
        self.btn_connect.config(state="disabled")
        try:
            self.ble.submit(self.ble.connect, self.selected_addr)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_disconnect(self):
        try:
            self.ble.submit(self.ble.disconnect)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_start_recording(self):
        self.ble.set_recording(True)
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.btn_clear.config(state="normal")
        self._log("Recording ON. Stop will open Save dialog immediately.")

    def on_stop_and_save(self):
        # Stop recording first
        self.ble.set_recording(False)
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.btn_clear.config(state="normal")

        # Immediately open save dialog
        self._save_dialog_and_write()

    def on_clear(self):
        self.ble.clear_recording()

    # ---------- Save handling ----------
    def _save_dialog_and_write(self):
        lines = list(self.ble.state.recorded_lines)
        if not lines:
            messagebox.showinfo("No data", "No recorded lines to save.")
            return

        # If header didn't arrive during recording, inject patched header.
        if not any(is_header(l) for l in lines):
            base = "seq,t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,cap0,cap1,cap2,cap3,cap4,cap5\n"
            lines.insert(0, patch_header(base))

        default_name = f"insole_{now_ts()}.csv"
        path = filedialog.asksaveasfilename(
            title="Save CSV",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            self._log("Save canceled (recording stopped).")
            return

        out_path = Path(path)
        text = "".join(lines)
        if not text.endswith("\n"):
            text += "\n"

        try:
            out_path.write_text(text, encoding="utf-8")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))
            return

        drops = self.ble.state.total_drop_seq
        self._log(f"Saved: {out_path} (total drop_seq={drops})")

    # ---------- Event polling ----------
    def _poll_events(self):
        try:
            while True:
                evt = self.ui_event_q.get_nowait()
                self._handle_event(evt)
        except queue.Empty:
            pass
        self.after(100, self._poll_events)

    def _handle_event(self, evt: tuple):
        etype = evt[0]

        if etype == "log":
            self._log(evt[1])

        elif etype == "error":
            self._log("ERROR: " + evt[1])
            messagebox.showerror("BLE error", evt[1])

        elif etype == "scan_results":
            self.scan_results = evt[1]
            self.lst.delete(0, tk.END)
            for name, addr, uuids in self.scan_results:
                uu = ",".join(uuids) if uuids else ""
                self.lst.insert(tk.END, f"{name} , {addr} , {uu}")
            self._log(f"Scan done. Found {len(self.scan_results)} devices.")
            if self.scan_results:
                self.lst.selection_set(0)
                self.on_select_device()

        elif etype == "connected":
            connected = bool(evt[1])
            if connected:
                self.lbl_conn.config(text="Connected")
                self.btn_disconnect.config(state="normal")
                self.btn_start.config(state="normal")
                self.btn_clear.config(state="normal")
                self.btn_connect.config(state="disabled")
                self._log("Connected.")
            else:
                self.lbl_conn.config(text="Disconnected")
                self.lbl_rec.config(text="Recording: OFF")
                self.btn_disconnect.config(state="disabled")
                self.btn_start.config(state="disabled")
                self.btn_stop.config(state="disabled")
                self.btn_clear.config(state="disabled")
                self.btn_connect.config(state="normal" if self.selected_addr else "disabled")
                self._log("Disconnected.")

        elif etype == "recording":
            rec = bool(evt[1])
            self.lbl_rec.config(text=f"Recording: {'ON' if rec else 'OFF'}")

        elif etype == "stats":
            bytes_rx, lines_rx, last_lines, drop_total = evt[1], evt[2], evt[3], evt[4]
            self.lbl_stats.config(text=f"Bytes: {bytes_rx} , Lines: {lines_rx} , DropSeq: {drop_total}")
            self.txt_preview.delete("1.0", tk.END)
            self.txt_preview.insert(tk.END, "\n".join(last_lines))

        else:
            self._log(f"Unknown event: {evt}")

    def _log(self, msg: str):
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)


def main():
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
