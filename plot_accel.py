"""
Dipendenze:
    pip install pyserial matplotlib

Uso tipico (auto-detect porta):
    python plot_accel.py

Specificare la porta manualmente:
    python plot_accel.py --port COM5

Salvare anche su CSV:
    python plot_accel.py --csv dati.csv

Elencare le porte disponibili:
    python plot_accel.py --list
"""

import argparse
import csv
import re
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ---------------------------------------------------------------------------
# Utility: rileva la porta dello ST-Link
# ---------------------------------------------------------------------------
def find_stlink_port():
    """Cerca tra le porte seriali una che assomigli a uno ST-Link."""
    candidates = []
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        manuf = (port.manufacturer or "").lower()
        hwid = (port.hwid or "").lower()
        # ST-Link si presenta come "STMicroelectronics STLink Virtual COM Port"
        # oppure HWID contiene VID:PID 0483:374B (o simili)
        if ("stlink" in desc or "st-link" in desc
                or "stmicroelectronics" in manuf
                or "0483:374" in hwid):
            candidates.append(port.device)
    if candidates:
        return candidates[0]
    return None


def list_ports():
    print("Porte seriali disponibili:")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (nessuna porta trovata)")
        return
    for p in ports:
        print(f"  {p.device:8s}  {p.description}")
        if p.hwid:
            print(f"            hwid: {p.hwid}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Plot real-time dell'accelerometro via ST-Link COM port.")
    parser.add_argument("--port", default=None,
                        help="Porta seriale (es. COM5). Auto-detect se omesso.")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--window", type=int, default=300,
                        help="N. campioni mostrati a video (default: 300)")
    parser.add_argument("--ylim", type=int, default=2200,
                        help="Limite asse Y in mg (default: 2200, FS=2g+margine)")
    parser.add_argument("--csv", default=None,
                        help="Se specificato, salva i dati in questo file CSV.")
    parser.add_argument("--list", action="store_true",
                        help="Elenca le porte seriali e termina.")
    parser.add_argument("--quiet", action="store_true",
                        help="Non stampa le righe di log della firmware.")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return 0

    # --- Apertura porta ---
    port = args.port or find_stlink_port()
    if not port:
        print("[ERR] Porta ST-Link non trovata.")
        print("      Usa --list per vedere le porte disponibili,")
        print("      poi rilancia con --port COMx")
        return 1

    print(f"[OK] Apertura {port} @ {args.baud} baud, 8N1")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"[ERR] Impossibile aprire {port}: {e}")
        return 1

    # --- Buffer dati (rolling) ---
    N = args.window
    idx_buf = deque(maxlen=N)
    x_buf = deque(maxlen=N)
    y_buf = deque(maxlen=N)
    z_buf = deque(maxlen=N)

    state = {
        "sample_idx": 0,
        "t_start": time.time(),
    }

    # --- CSV opzionale ---
    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["sample", "time_s", "x_mg", "y_mg", "z_mg"])
        print(f"[OK] Salvataggio CSV su {args.csv}")

    # --- Setup plot ---
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(11, 6))
    fig.canvas.manager.set_window_title(
        f"SensorTile.box PRO  -  Accelerometro  -  {port}")

    line_x, = ax.plot([], [], color="#ff5555", linewidth=1.4, label="X")
    line_y, = ax.plot([], [], color="#55ff55", linewidth=1.4, label="Y")
    line_z, = ax.plot([], [], color="#5599ff", linewidth=1.4, label="Z")

    ax.set_xlabel("Sample #")
    ax.set_ylabel("Accel [mg]")
    ax.set_title("LSM6DSV16X  -  streaming BLE BlueST V1  -  FS = 2g, ODR = 60 Hz")
    ax.set_ylim(-args.ylim, args.ylim)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")

    # Linee orizzontali di riferimento: 0, +/-1g
    for y_ref, lbl in [(0, "0"), (1000, "+1g"), (-1000, "-1g")]:
        ax.axhline(y_ref, color="#666", linewidth=0.5, linestyle="--", alpha=0.6)

    # Box di stato live
    status_text = ax.text(
        0.99, 0.97, "", transform=ax.transAxes, ha="right", va="top",
        fontfamily="monospace", fontsize=9,
        bbox=dict(boxstyle="round,pad=0.4",
                  facecolor="#222a36", edgecolor="#555"))

    # Regex per le righe dati: "X,Y,Z" (interi, anche negativi)
    data_pat = re.compile(r"^(-?\d+),(-?\d+),(-?\d+)\s*$")

    # --- Loop di update ---
    def update(_frame):
        # Leggi tutte le righe disponibili (non bloccante)
        max_lines = 100
        while max_lines > 0 and ser.in_waiting > 0:
            try:
                raw = ser.readline()
            except serial.SerialException:
                break
            if not raw:
                break
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            max_lines -= 1
            if not line:
                continue

            m = data_pat.match(line)
            if m:
                x = int(m.group(1))
                y = int(m.group(2))
                z = int(m.group(3))
                t = time.time() - state["t_start"]
                state["sample_idx"] += 1
                idx_buf.append(state["sample_idx"])
                x_buf.append(x)
                y_buf.append(y)
                z_buf.append(z)
                if csv_writer:
                    csv_writer.writerow([state["sample_idx"], f"{t:.4f}", x, y, z])
            else:
                # Riga di log della firmware (es. [BOOT], [CONN], ...)
                if not args.quiet:
                    print(f"[FW] {line}")

        # Aggiorna plot
        if idx_buf:
            line_x.set_data(idx_buf, x_buf)
            line_y.set_data(idx_buf, y_buf)
            line_z.set_data(idx_buf, z_buf)
            # finestra scorrevole
            x_max = idx_buf[-1]
            x_min = max(0, x_max - N)
            ax.set_xlim(x_min, x_max + 1)

            # statistiche live
            elapsed = time.time() - state["t_start"]
            rate = state["sample_idx"] / elapsed if elapsed > 0 else 0.0
            x_cur, y_cur, z_cur = x_buf[-1], y_buf[-1], z_buf[-1]
            mag = (x_cur**2 + y_cur**2 + z_cur**2) ** 0.5
            status_text.set_text(
                f"X   = {x_cur:+6d} mg\n"
                f"Y   = {y_cur:+6d} mg\n"
                f"Z   = {z_cur:+6d} mg\n"
                f"|a| = {mag:6.0f} mg\n"
                f"---\n"
                f"samples = {state['sample_idx']:>6d}\n"
                f"rate    = {rate:5.1f} Hz"
            )

        return line_x, line_y, line_z, status_text

    ani = animation.FuncAnimation(
        fig, update, interval=30, blit=False, cache_frame_data=False)

    print("[OK] Pronto. Chiudi la finestra del grafico per terminare.")
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        if csv_file:
            csv_file.close()
            print(f"[OK] CSV salvato in {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
