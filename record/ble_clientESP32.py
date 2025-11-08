import asyncio
import threading
import queue
import json
import csv
import struct
import datetime
import tkinter as tk
from tkinter import scrolledtext, filedialog, messagebox
from bleak import BleakClient, BleakScanner

SERVICE_UUID      = "12345678-1234-1234-1234-123456789012"
DATA_CHAR_UUID    = "abcdef12-3456-789a-bcde-123456789abc"
CONTROL_CHAR_UUID = "12345678-1234-1234-1234-123456789013"

MAGIC_V2 = 0xB0B0
VER_V2   = 2
REC_V2_SIZE = 14  # dt(u16) + 6*int16

MAGIC_V1 = 0xBEEF
VER_V1   = 1

class BLEWorker(threading.Thread):
    def __init__(self, data_queue):
        super().__init__(daemon=True)
        self.loop = asyncio.new_event_loop()
        self.client = None
        self.device_address = None
        self.data_queue = data_queue
        self.running = False

    async def find_and_connect(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == "ESP32-IMU":
                self.device_address = d.address
                break
        if not self.device_address:
            self.data_queue.put({'type':'error','msg':'ESP32-IMU not found'})
            return False

        self.client = BleakClient(self.device_address, loop=self.loop)
        await self.client.connect()

        # Determine MTU (platform dependent). On Linux, exchange_mtu works; others ignore.
        payload_cap = 20  # safe default (ATT_MTU 23 -> 20 payload)
        try:
            if hasattr(self.client, "exchange_mtu"):
                mtu = await self.client.exchange_mtu(247)
                # bleak returns full MTU or None; usable payload = mtu-3
                if isinstance(mtu, int) and mtu >= 23:
                    payload_cap = max(20, mtu - 3)
                elif hasattr(self.client, "mtu_size"):
                    payload_cap = max(20, int(getattr(self.client, "mtu_size")) - 3)
            elif hasattr(self.client, "mtu_size"):
                payload_cap = max(20, int(getattr(self.client, "mtu_size")) - 3)
        except Exception as e:
            self.data_queue.put({'type':'info','msg':f"MTU info unavailable: {e}"})

        # Tell firmware the usable payload so it can size packets
        try:
            await self.client.write_gatt_char(CONTROL_CHAR_UUID, f"MTU:{payload_cap}".encode())
            self.data_queue.put({'type':'info','msg':f"Usable payload = {payload_cap} bytes (told device)"})
        except Exception as e:
            self.data_queue.put({'type':'info','msg':f"Could not send MTU to device: {e}"})

        await self.client.start_notify(DATA_CHAR_UUID, self.notification_handler)
        self.data_queue.put({'type':'info','msg':'🔗 Connected'})
        return True

    def parse_v2(self, b: bytes):
        if len(b) < 4: return None
        magic = b[0] | (b[1] << 8)
        ver   = b[2]
        if magic != MAGIC_V2 or ver != VER_V2:
            return None
        count = b[3]
        needed = 4 + count * REC_V2_SIZE
        if len(b) < needed:
            return None
        offs = 4
        samples = []
        # reconstruct absolute esp_timestamp from dt sums (ms)
        t_abs = 0
        ts_pc = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        for i in range(count):
            dt = struct.unpack_from("<H", b, offs)[0]; offs += 2
            t_abs += dt
            ax, ay, az, gx, gy, gz = struct.unpack_from("<hhhhhh", b, offs); offs += 12
            samples.append({
                'pc_timestamp': ts_pc,
                'esp_timestamp': t_abs,  # relative ms since first sample in stream; you can realign later
                'accel': [ax/1000.0, ay/1000.0, az/1000.0],   # mg -> g
                'gyro':  [gx/1000.0, gy/1000.0, gz/1000.0],   # mdps -> dps
            })
        return samples

    def parse_v1(self, b: bytes):
        # legacy: u16 magic | u8 ver | u8 count | u32 seq_start | count*(u32 t_ms + 6*float)
        if len(b) < 8: return None
        magic = b[0] | (b[1] << 8)
        ver   = b[2]
        if magic != MAGIC_V1 or ver != VER_V1:
            return None
        count = b[3]
        needed = 8 + count * (4 + 24)
        if len(b) < needed:
            return None
        offs = 8
        ts_pc = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        samples = []
        for i in range(count):
            t_ms = struct.unpack_from("<I", b, offs)[0]; offs += 4
            ax, ay, az, gx, gy, gz = struct.unpack_from("<ffffff", b, offs); offs += 24
            samples.append({
                'pc_timestamp': ts_pc,
                'esp_timestamp': t_ms,
                'accel': [ax, ay, az],
                'gyro':  [gx, gy, gz],
            })
        return samples

    async def notification_handler(self, sender, data: bytearray):
        # Try v2 compact
        s = self.parse_v2(data)
        if s is not None:
            for e in s:
                self.data_queue.put({'type':'data','entry':e})
            return
        # Try v1 legacy
        s = self.parse_v1(data)
        if s is not None:
            for e in s:
                self.data_queue.put({'type':'data','entry':e})
            return
        # Finally, try JSON (battery/warnings). If it fails, ignore silently (it's binary noise).
        try:
            payload = json.loads(data.decode('utf-8'))
            self.data_queue.put({'type':'json','obj':payload})
        except Exception:
            # do not spam errors on binary payloads
            return

    async def handle_commands(self):
        while self.running:
            await asyncio.sleep(0.05)
        if self.client and self.client.is_connected:
            try: await self.client.stop_notify(DATA_CHAR_UUID)
            except Exception: pass
            await self.client.disconnect()
            self.data_queue.put({'type':'info','msg':'🔌 Disconnected'})

    def run(self):
        self.running = True
        asyncio.set_event_loop(self.loop)
        try:
            connected = self.loop.run_until_complete(self.find_and_connect())
            if connected:
                self.loop.run_until_complete(self.handle_commands())
        except Exception as e:
            self.data_queue.put({'type':'error','msg':str(e)})
        finally:
            self.loop.close()

    def send(self, command: str):
        if self.client and self.client.is_connected:
            asyncio.run_coroutine_threadsafe(
                self.client.write_gatt_char(CONTROL_CHAR_UUID, command.encode()),
                self.loop
            )
            self.data_queue.put({'type':'info','msg':f"Sent: {command}"})

    def stop(self):
        self.running = False


class IMUGUI:
    def __init__(self, root):
        self.root = root
        root.title("ESP32-IMU GUI (MTU-aware compact)")
        self.queue = queue.Queue()
        self.ble = None
        self.recording = False
        self.data_log = []
        self.seq = 0

        frm = tk.Frame(root)
        frm.pack(padx=10, pady=5)

        tk.Button(frm, text="Connect", command=self.connect).grid(row=0, column=0, padx=2)
        tk.Button(frm, text="Disconnect", command=self.disconnect).grid(row=0, column=1, padx=2)

        tk.Label(frm, text="Rate (Hz):").grid(row=1, column=0, sticky="e")
        self.rate_var = tk.StringVar(value="200")
        tk.Entry(frm, textvariable=self.rate_var, width=6).grid(row=1, column=1)
        tk.Button(frm, text="Set Rate", command=self.set_rate).grid(row=1, column=2, padx=4)

        tk.Button(frm, text="Start Recording", command=self.start_recording).grid(row=2, column=0, pady=5)
        tk.Button(frm, text="Stop & Save CSV", command=self.stop_recording).grid(row=2, column=1, pady=5)

        self.log = scrolledtext.ScrolledText(root, width=92, height=26, state='disabled')
        self.log.pack(padx=10, pady=5)

        self.root.after(100, self.process_queue)

    def connect(self):
        if self.ble and self.ble.is_alive():
            messagebox.showinfo("Info", "Already connected")
            return
        self.ble = BLEWorker(self.queue)
        self.ble.start()

    def disconnect(self):
        if self.ble:
            self.ble.stop()

    def set_rate(self):
        hz = self.rate_var.get()
        if not hz.isdigit() or not (1 <= int(hz) <= 1000):
            messagebox.showerror("Error", "Rate must be 1..1000 Hz")
            return
        self.send_cmd(f"RATE:{hz}")

    def send_cmd(self, cmd):
        if self.ble:
            self.ble.send(cmd)
        else:
            messagebox.showwarning("Warning", "Not connected")

    def start_recording(self):
        if self.recording:
            messagebox.showinfo("Info", "Recording already in progress")
            return
        self.data_log.clear()
        self.seq = 0
        self.recording = True
        self.append_log("📼 Recording started\n")

    def stop_recording(self):
        if not self.recording:
            messagebox.showinfo("Info", "Recording is not active")
            return
        self.recording = False
        self.append_log("⏹️ Recording stopped\n")
        self.save_csv()

    def process_queue(self):
        try:
            while True:
                msg = self.queue.get_nowait()
                if msg['type'] == 'data':
                    e = msg['entry']
                    self.seq += 1
                    line = (f"{e['pc_timestamp']} | #{self.seq:08d} "
                            f"A[{e['accel'][0]:.3f}, {e['accel'][1]:.3f}, {e['accel'][2]:.3f}] "
                            f"G[{e['gyro'][0]:.3f}, {e['gyro'][1]:.3f}, {e['gyro'][2]:.3f}]\n")
                    self.append_log(line)
                    if self.recording:
                        self.data_log.append({
                            'pc_timestamp': e['pc_timestamp'],
                            'seq': self.seq,
                            'esp_timestamp_ms': e['esp_timestamp'],
                            'ax_g': e['accel'][0], 'ay_g': e['accel'][1], 'az_g': e['accel'][2],
                            'gx_dps': e['gyro'][0], 'gy_dps': e['gyro'][1], 'gz_dps': e['gyro'][2],
                        })
                elif msg['type'] == 'json':
                    self.append_log(f"JSON: {json.dumps(msg['obj'])}\n")
                elif msg['type'] == 'info':
                    self.append_log(f"{msg['msg']}\n")
                elif msg['type'] == 'error':
                    self.append_log(f"ERROR: {msg['msg']}\n")
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_queue)

    def append_log(self, text):
        self.log.configure(state='normal')
        self.log.insert(tk.END, text)
        self.log.yview(tk.END)
        self.log.configure(state='disabled')

    def save_csv(self):
        if not self.data_log:
            messagebox.showinfo("Info", "No data to save")
            return
        fname = filedialog.asksaveasfilename(defaultextension='.csv',
                                             filetypes=[('CSV Files','*.csv')])
        if not fname:
            return
        fields = ['pc_timestamp','seq','esp_timestamp_ms','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps']
        with open(fname, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.data_log)
        messagebox.showinfo("Success", f"CSV saved: {fname}")

if __name__ == '__main__':
    root = tk.Tk()
    app = IMUGUI(root)
    root.mainloop()
