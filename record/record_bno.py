#!/usr/bin/env python3
"""
bno_recorder.py - ESP32-S3 BNO08x High-Speed CSV Recorder (Windows/Unix)
- Setzt DTR/RTS stabil auf 1 (host connected)
- Kein Reset-Puls (vermeidet CDC-Stolperfallen)
- Legt Datei erst bei erster Datenzeile an (kein Header-only / keine leeren Dateien)
- Live-Statistiken, Gap-Erkennung, Auto-Port, Port-Listing, optionaler Port-Reset

Usage:
  python bno_recorder.py --list
  python bno_recorder.py --port COM6 --duration 75
  python bno_recorder.py --auto --duration 60 --echo
  python bno_recorder.py --reset COM6
"""

import argparse
import sys
import time
import os
from datetime import datetime
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    print("[!] Missing dependency 'pyserial'. Install with:\n    pip install pyserial")
    sys.exit(1)

HEADER = "seq,t_ms,ax,ay,az"


# -------- Serial-Port Hilfen --------

def list_serial_ports(verbose: bool = True):
    ports = list(list_ports.comports())
    if verbose:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device:>8}  {p.description}")
    return ports


def guess_port() -> Optional[str]:
    """Auto-detect ESP32-S3 USB CDC"""
    ports = list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc for k in ["esp32", "usb serial", "usb jtag", "cdc"]) or \
           any(k in hwid for k in ["303a", "1a86", "10c4"]):
            candidates.append(p.device)
    candidates = sorted(set(candidates))
    return candidates[0] if len(candidates) == 1 else None


def reset_port(port: str) -> bool:
    """Einfacher COM-Reset durch kurzes Öffnen + DTR/RTS-Toggle"""
    print(f"[i] Resetting {port}...")
    for i in range(3):
        try:
            print(f"    Attempt {i+1}/3...")
            ser = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0.2,
                write_timeout=0.2,
                dsrdtr=False,
                rtscts=False,
                xonxoff=False,
            )
            # kurzer Toggle
            ser.dtr = False
            ser.rts = False
            time.sleep(0.1)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.1)
            ser.dtr = False
            ser.rts = False
            time.sleep(0.1)
            ser.close()
            time.sleep(0.3)
            print("    OK")
        except Exception as e:
            print(f"    Error: {e}")
            time.sleep(0.3)
    print("[✓] Port reset complete\n")
    return True


def open_serial(
    port: str, baud: int,
    dtr: bool = True, rts: bool = True,
    handshake_timeout: float = 5.0,
    auto_reset: bool = True
) -> serial.Serial:
    """
    Öffnet Port, setzt DTR/RTS stabil und wartet optional kurz auf erste Header-/Datenzeile.
    Kein Flush nach Handshake (sonst verliert man den frühen Header).
    """
    print(f"[i] Öffne {port} @ {baud}...")
    last_error = None

    for attempt in range(3):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.5,
                write_timeout=1,
                dsrdtr=False,
                rtscts=False,
                xonxoff=False,
            )

            # Stabiler Leitungszustand -> host connected
            ser.dtr = bool(dtr)
            ser.rts = bool(rts)
            time.sleep(0.15)

            # Optional: bis zu 'handshake_timeout' Sekunden auf Header ODER Daten warten
            print("[i] Warte auf Header oder erste Daten...")
            start = time.time()
            while time.time() - start < handshake_timeout:
                line = ser.readline()
                if not line:
                    continue
                s = line.decode("utf-8", errors="ignore").strip()
                # Header oder plausible CSV-Zeile akzeptieren
                if s == HEADER or s.startswith("seq,"):
                    print("[✓] Header empfangen.")
                    return ser
                if s.count(",") == 4 and s.split(",")[0].isdigit():
                    print("[✓] Erste Daten empfangen.")
                    return ser
                # Kommentare anzeigen (optional)
                if s.startswith("#"):
                    print(f"    {s}")

            print("[w] Innerhalb Timeout kein Header/Daten gesehen – fahre trotzdem fort...")
            return ser

        except serial.SerialException as e:
            last_error = e
            if attempt < 2:
                print(f"[w] Port busy, retry {attempt+1}/3...")
                if auto_reset and attempt == 0:
                    reset_port(port)
                time.sleep(1)
            else:
                break

    raise SystemExit(
        f"\n[!] Konnte {port} nicht öffnen: {last_error}\n"
        f"\nHinweise:"
        f"\n  • Teste mit: py -m serial.tools.miniterm {port} {baud} --dtr 1 --rts 1"
        f"\n  • Stelle sicher, dass kein anderer Monitor (PIO, putty, etc.) offen ist.\n"
    )


# -------- CSV / Aufnahme --------

def looks_like_csv(line: str) -> bool:
    """Validiert CSV Zeile: seq,t_ms,ax,ay,az"""
    if line == HEADER:
        return True
    if line.count(",") != 4:
        return False
    parts = line.split(",")
    return parts[0].isdigit()


def record(
    port: str,
    baud: int,
    outfile: Optional[str],
    duration: float,
    echo: bool,
    dtr: bool,
    rts: bool,
    handshake_timeout: float
):
    # Dateiname vorbereiten; Datei erst bei erster Datenzeile anlegen
    if not outfile:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        outfile = f"bno086_wrist_{ts}.csv"

    ser = open_serial(port, baud, dtr=dtr, rts=rts, handshake_timeout=handshake_timeout)
    print(f"[i] Duration: {duration}s" if duration > 0 else "[i] Duration: unlimited (Ctrl+C to stop)")

    start = time.time()
    last_status = start
    lines = 0
    bytes_rx = 0
    last_seq = -1
    gaps = 0

    # Lazy file creation
    f = None
    outfile_opened = False

    # Real-time Hz Schätzer
    samples_window = []

    try:
        while True:
            if duration > 0 and (time.time() - start) >= duration:
                break

            raw = ser.readline()
            if not raw:
                continue

            bytes_rx += len(raw)

            try:
                s = raw.decode("utf-8", errors="ignore").strip()
            except UnicodeDecodeError:
                continue

            # Kommentare durchreichen (optional)
            if s.startswith("#"):
                if echo:
                    print(f"  [ESP32] {s}")
                continue

            # Ungültig skippen
            if not looks_like_csv(s):
                continue

            # Duplicate Header skippen
            if s == HEADER:
                continue

            # Ab erster gültiger Datenzeile Datei eröffnen + Header schreiben
            if not outfile_opened:
                try:
                    f = open(outfile, "w", newline="", encoding="utf-8")
                    f.write(HEADER + "\n")
                    outfile_opened = True
                    print(f"[i] Recording → {outfile}")
                except Exception as e:
                    print(f"[!] Konnte Datei nicht anlegen: {e}")
                    break

            # Daten schreiben
            f.write(s + "\n")
            lines += 1
            samples_window.append(time.time())

            # Gap detection via seq
            try:
                seq = int(s.split(",")[0])
                if last_seq >= 0 and seq != last_seq + 1:
                    gap = seq - last_seq - 1
                    print(f"[!] Gap: {gap} samples missing (seq {last_seq}→{seq})")
                    gaps += 1
                last_seq = seq
            except Exception:
                pass

            # Echo-Drossel
            if echo and (lines % 200 == 0):
                print(f"    {s}")

            # Status alle ~2s
            now = time.time()
            if now - last_status >= 2.0:
                elapsed = now - start
                avg_hz = lines / elapsed if elapsed > 0 else 0.0

                # ~2s-Fenster für "now"-Hz
                samples_window = [t for t in samples_window if t > now - 2.0]
                rt_hz = len(samples_window) / 2.0 if samples_window else 0.0

                mb = bytes_rx / (1024 * 1024)
                print(f"[i] {lines:7d} lines | {avg_hz:6.1f} Hz avg | {rt_hz:6.1f} Hz now | {mb:.2f} MB | {elapsed:.0f}s")
                last_status = now

                if outfile_opened:
                    f.flush()

    except KeyboardInterrupt:
        print("\n[i] Stopped by Ctrl+C")
    except Exception as e:
        print(f"\n[!] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            ser.close()
        except Exception:
            pass
        if f:
            try:
                f.close()
            except Exception:
                pass

    # Abschluss
    elapsed = time.time() - start
    mb = bytes_rx / (1024 * 1024)

    if lines == 0:
        # Keine Daten empfangen → nichts persistieren
        if outfile_opened:
            try:
                os.remove(outfile)
                print(f"\n[i] Keine Daten empfangen, leere Datei entfernt: {outfile}")
            except Exception as e:
                print(f"\n[w] Leere Datei konnte nicht gelöscht werden: {e}")
        else:
            print("\n[i] Keine Daten empfangen, keine Datei angelegt.")
        print("=" * 70)
        print(f"  Duration:  {elapsed:.2f}s")
        print(f"  Lines:     0")
        print(f"  Data Size: {mb:.2f} MB")
        print("=" * 70)
        return

    avg_hz = lines / elapsed if elapsed > 0 else 0.0
    print("\n" + "=" * 70)
    print(f"✓ Recording Complete!")
    print(f"  File:      {outfile}")
    print(f"  Duration:  {elapsed:.2f}s")
    print(f"  Lines:     {lines:,}")
    print(f"  Avg Rate:  {avg_hz:.1f} Hz")
    print(f"  Data Size: {mb:.2f} MB")
    if gaps > 0:
        print(f"  Gaps:      {gaps} detected")
    print("=" * 70)


# -------- CLI --------

def main(argv=None):
    p = argparse.ArgumentParser(
        description="ESP32-S3 BNO08x High-Speed CSV Recorder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python bno_recorder.py --list
  python bno_recorder.py --port COM6 --duration 75
  python bno_recorder.py --auto --duration 300 --echo
  python bno_recorder.py --reset COM6
        """
    )
    p.add_argument("--list", action="store_true", help="Zeige verfügbare Ports")
    p.add_argument("--reset", metavar="PORT", help="Reset COM Port (löst Windows Locks)")
    p.add_argument("--auto", action="store_true", help="Auto-detect ESP32 Port")
    p.add_argument("--port", help="COM Port (z.B. COM6)")
    p.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    p.add_argument("--duration", type=float, default=60.0, help="Aufnahmedauer in Sekunden (0 = unlimited)")
    p.add_argument("--outfile", help="Output CSV Dateiname")
    p.add_argument("--echo", action="store_true", help="Zeige Daten im Terminal")
    p.add_argument("--dtr", type=int, choices=[0, 1], default=1, help="Setze DTR (0/1, default: 1)")
    p.add_argument("--rts", type=int, choices=[0, 1], default=1, help="Setze RTS (0/1, default: 1)")
    p.add_argument("--handshake-timeout", type=float, default=5.0,
                   help="Zeitfenster zum Warten auf Header/Daten (s)")

    args = p.parse_args(argv)

    if args.list:
        list_serial_ports(verbose=True)
        return 0

    if args.reset:
        reset_port(args.reset)
        return 0

    port = args.port
    if args.auto and not port:
        port = guess_port()
        if not port:
            print("[!] Auto-detect fehlgeschlagen.")
            print("\nVerfügbare Ports:")
            list_serial_ports(verbose=True)
            return 1
        print(f"[✓] Auto-selected: {port}")

    if not port:
        print("[!] Bitte --port angeben (oder --auto / --list)")
        return 1

    record(
        port=port,
        baud=args.baud,
        outfile=args.outfile,
        duration=args.duration,
        echo=args.echo,
        dtr=bool(args.dtr),
        rts=bool(args.rts),
        handshake_timeout=args.handshake_timeout
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())