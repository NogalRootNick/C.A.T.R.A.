import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import os
import threading
import sys
import serial
import serial.tools.list_ports
import re
import time
from PIL import Image, ImageTk
import numpy as np
import random

# Define los patrones de expresiones regulares
x_pattern = re.compile(r"LIDAR COORD: X=([-+]?\d+\.?\d*)")
y_pattern = re.compile(r"Y=([-+]?\d+\.?\d*)")
temp_pattern = re.compile(r"Temp:\s*([-+]?\d+\.?\d*)")
hum_pattern = re.compile(r"Hum:\s*([-+]?\d+\.?\d*)")
pres_pattern = re.compile(r"Pres:\s*([-+]?\d+\.?\d*)")
map_complete_pattern = re.compile(r"MAP_COMPLETE\s+(.+)")

# --- Serial Port Configuration ---
SERIAL_PORT = 'AUTO'
MANUAL_FALLBACK_PORT = 'COM3'
BAUD_RATE = 9600
MAX_POINTS_LIDAR = 500
MAX_POINTS_SCATTER = 10 * 1000  # Increased for a more detailed map
DATA_TIMEOUT_SECONDS = 2.0

# Data collections
x_data_scatter = deque(maxlen=MAX_POINTS_SCATTER)
y_data_scatter = deque(maxlen=MAX_POINTS_SCATTER)
x_data_stats = deque(maxlen=MAX_POINTS_LIDAR)
y_data_stats = deque(maxlen=MAX_POINTS_LIDAR)
temp_data = deque(maxlen=200)
hum_data = deque(maxlen=200)
pres_data = deque(maxlen=200)

plotting_active = True
last_x_val = None
last_y_val = None
serial_status_var = None
last_data_timestamp = time.time()
data_source = "serial"  # New: 'serial' or 'simulation'

# Matplotlib objects
fig_scatter, ax_scatter, line_scatter = None, None, None
fig_stats, ax_stats, line_stats = None, None, None
fig_sensors, ax_temp, ax_hum, ax_pres = None, None, None, None
line_temp, line_hum, line_pres = None, None, None
serial_connection = None
current_displayed_map_path = None

# Paleta de colores en hexadecimal para el nuevo diseño
BG_COLOR_DARKEST = "#1f1f1f"
PANEL_COLOR = "#2d2d2d"
TEXT_COLOR = "#ffffff"
MUTED_TEXT_COLOR = "#b0b0b0"
ACCENT_BLUE = "#3498db"
ACCENT_ORANGE = "#e67e22"
GRID_COLOR = "#444444"
BUTTON_BG_COLOR = "#3a3a3a"
BUTTON_FG_COLOR = "#ffffff"
LIDAR_POINT_COLOR = "#00ffff" # Cian para los puntos del LIDAR
TEMP_LINE_COLOR = "#5dade2" # Azul claro
HUM_LINE_COLOR = "#f5b041"  # Naranja
PRES_LINE_COLOR = "#2ecc71" # Verde

CAPS = "catra_maps"
MAP_PREVIEW_PATH = os.path.join(CAPS, "last_completed_map.png")

if not os.path.exists(CAPS):
    try:
        os.makedirs(CAPS)
        print(f"Created directory: {CAPS}")
    except OSError as e:
        print(f"Error creating directory {CAPS}: {e}")
        messagebox.showerror("Error de Directorio", f"No se pudo crear la carpeta para mapas '{CAPS}':\n{e}\n"
                                                      "Asegúrese de tener permisos de escritura.")

def configure_plot_style(ax, title, xlabel, ylabel):
    ax.set_facecolor(PANEL_COLOR)
    ax.tick_params(axis='x', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax.tick_params(axis='y', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax.xaxis.label.set_color(MUTED_TEXT_COLOR)
    ax.yaxis.label.set_color(MUTED_TEXT_COLOR)
    ax.title.set_color(TEXT_COLOR)
    ax.spines['left'].set_color(GRID_COLOR)
    ax.spines['bottom'].set_color(GRID_COLOR)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_linewidth(0.5)
    ax.spines['bottom'].set_linewidth(0.5)
    ax.grid(True, linestyle=':', alpha=0.5, color=GRID_COLOR, linewidth=0.5)
    ax.set_title(title, fontsize=10, pad=5, loc='left')
    ax.set_xlabel(xlabel, fontsize=8)
    ax.set_ylabel(ylabel, fontsize=8)

def setup_stats_plot_config():
    global fig_stats, ax_stats, line_stats
    fig_stats = plt.Figure(figsize=(3, 3), dpi=100)  # Tamaño reducido del gráfico
    ax_stats = fig_stats.add_subplot(111, polar=True)
    fig_stats.patch.set_facecolor(PANEL_COLOR)
    ax_stats.set_facecolor(PANEL_COLOR)

    ax_stats.set_theta_zero_location('N')
    ax_stats.set_theta_direction(-1)
    ax_stats.set_ylim(0, 1000)  # Example range for stats plot
    ax_stats.set_rticks(np.arange(0, 1001, 200))
    ax_stats.set_rlabel_position(-22.5)
    ax_stats.tick_params(axis='both', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_stats.grid(color=GRID_COLOR, linestyle=':', linewidth=0.5, alpha=0.5)
    ax_stats.spines['polar'].set_color(GRID_COLOR)
    ax_stats.set_title("LIDAR Position", fontsize=12, pad=10, color=TEXT_COLOR)

    line_stats, = ax_stats.plot([], [], 'o', color=LIDAR_POINT_COLOR, markersize=3, alpha=0.7)

def setup_scatter_plot_config():
    global fig_scatter, ax_scatter, line_scatter
    fig_scatter = plt.Figure(figsize=(8, 8), dpi=100)
    ax_scatter = fig_scatter.add_subplot(111)
    fig_scatter.patch.set_facecolor(PANEL_COLOR)
    configure_plot_style(ax_scatter, "Real-Time Scatter Map", "X (mm)", "Y (mm)")
    ax_scatter.set_aspect('equal', adjustable='box')
    ax_scatter.set_xlim(-1500, 1500)
    ax_scatter.set_ylim(-1500, 1500)
    line_scatter, = ax_scatter.plot([], [], 'o', color=ACCENT_BLUE, markersize=2, alpha=0.5)

def setup_sensor_plots():
    global fig_sensors, ax_temp, ax_hum, ax_pres, line_temp, line_hum, line_pres
    fig_sensors, (ax_temp, ax_hum, ax_pres) = plt.subplots(3, 1, figsize=(4, 6), dpi=100)
    fig_sensors.patch.set_facecolor(PANEL_COLOR)
    
    # Temperature Plot
    ax_temp.set_facecolor(PANEL_COLOR)
    ax_temp.tick_params(axis='x', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_temp.tick_params(axis='y', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_temp.title.set_color(TEXT_COLOR)
    ax_temp.grid(True, linestyle=':', alpha=0.5, color=GRID_COLOR)
    ax_temp.set_title("Temperature °C", loc='left', color=TEXT_COLOR)
    line_temp, = ax_temp.plot([], [], color=TEMP_LINE_COLOR)

    # Humidity Plot
    ax_hum.set_facecolor(PANEL_COLOR)
    ax_hum.tick_params(axis='x', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_hum.tick_params(axis='y', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_hum.title.set_color(TEXT_COLOR)
    ax_hum.grid(True, linestyle=':', alpha=0.5, color=GRID_COLOR)
    ax_hum.set_title("Humidity (%)", loc='left', color=TEXT_COLOR)
    line_hum, = ax_hum.plot([], [], color=HUM_LINE_COLOR)

    # Pressure Plot
    ax_pres.set_facecolor(PANEL_COLOR)
    ax_pres.tick_params(axis='x', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_pres.tick_params(axis='y', colors=MUTED_TEXT_COLOR, labelsize=8)
    ax_pres.title.set_color(TEXT_COLOR)
    ax_pres.grid(True, linestyle=':', alpha=0.5, color=GRID_COLOR)
    ax_pres.set_title("Atmospheric Pressure (kPa)", loc='left', color=TEXT_COLOR)
    line_pres, = ax_pres.plot([], [], color=PRES_LINE_COLOR)

    fig_sensors.tight_layout(pad=1.5)

def display_map_preview():
    global map_preview_label
    if os.path.exists(MAP_PREVIEW_PATH):
        try:
            img = Image.open(MAP_PREVIEW_PATH)
            w, h = map_preview_label.winfo_width() - 2, map_preview_label.winfo_height() - 2
            if w > 0 and h > 0:
                img.thumbnail((w, h), Image.Resampling.LANCZOS)
                img_tk = ImageTk.PhotoImage(img)
                map_preview_label.config(image=img_tk, text="")
                map_preview_label.image = img_tk
            else:
                map_preview_label.config(text="Error al cargar el mapa: Tamaño de widget inválido.")
                map_preview_label.image = None
        except Exception as e:
            map_preview_label.config(text=f"Error al cargar el mapa:\n{e}")
            map_preview_label.image = None
    else:
        map_preview_label.config(text="No hay mapa previo guardado.")
        map_preview_label.image = None

def update_gui_elements():
    global plotting_active
    if plotting_active:
        # Update LIDAR plots
        if line_scatter and x_data_scatter and y_data_scatter:
            line_scatter.set_data(list(x_data_scatter), list(y_data_scatter))
            ax_scatter.relim()
            ax_scatter.autoscale_view()
            fig_scatter.canvas.draw_idle()

        if line_stats and x_data_stats and y_data_stats:
            r_vals = np.sqrt(np.array(x_data_stats)**2 + np.array(y_data_stats)**2)
            theta_vals = np.arctan2(y_data_stats, x_data_stats)
            line_stats.set_data(theta_vals, r_vals)
            ax_stats.relim()
            ax_stats.autoscale_view()
            fig_stats.canvas.draw_idle()

        # Update sensor plots
        if line_temp:
            line_temp.set_data(range(len(temp_data)), temp_data)
            ax_temp.set_xlim(0, max(20, len(temp_data)))
            ax_temp.relim()
            ax_temp.autoscale_view(True, True, True)
            fig_sensors.canvas.draw_idle()
        
        if line_hum:
            line_hum.set_data(range(len(hum_data)), hum_data)
            ax_hum.set_xlim(0, max(20, len(hum_data)))
            ax_hum.relim()
            ax_hum.autoscale_view(True, True, True)
            fig_sensors.canvas.draw_idle()

        if line_pres:
            line_pres.set_data(range(len(pres_data)), pres_data)
            ax_pres.set_xlim(0, max(20, len(pres_data)))
            ax_pres.relim()
            ax_pres.autoscale_view(True, True, True)
            fig_sensors.canvas.draw_idle()

        root.after(50, update_gui_elements)

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    keywords = ["arduino", "usb-serial", "ch340", "ft232r", "usbmodem", "cp210x", "serial"]
    for p in ports:
        if any(keyword in p.description.lower() for keyword in keywords) or \
           any(keyword in p.hwid.lower() for keyword in keywords):
            return p.device
    return None

def setup_serial_connection():
    global serial_connection, data_source
    if serial_connection and serial_connection.is_open:
        serial_status_var.set(f"Estado: Conectado a {serial_connection.port}")
        data_source = "serial"
        return True
    
    port_to_use = SERIAL_PORT
    if port_to_use == 'AUTO':
        port_to_use = find_arduino_port() or MANUAL_FALLBACK_PORT
    
    if not port_to_use:
        serial_status_var.set("Estado: Error (No hay puerto válido)")
        return False
        
    serial_status_var.set(f"Estado: Intentando conectar a {port_to_use}...")
    try:
        serial_connection = serial.Serial(port_to_use, BAUD_RATE, timeout=0.5)
        serial_status_var.set(f"Estado: Conectado a {port_to_use}")
        time.sleep(2)
        serial_connection.reset_input_buffer()
        data_source = "serial"
        return True
    except serial.SerialException as e:
        serial_status_var.set(f"Estado: Desconectado ({e})")
        messagebox.showerror("Error de Conexión Serial", f"No se pudo conectar al puerto serial:\n{e}")
        return False

def receive_from_serial():
    global plotting_active, serial_connection, last_data_timestamp, data_source
    if not setup_serial_connection():
        data_source = "simulation"
        simulate_data()
        serial_status_var.set("Estado: Sin Conexión Serial - Modo Simulación")
        return
        
    while plotting_active and data_source == "serial":
        try:
            if serial_connection and serial_connection.is_open:
                if serial_connection.in_waiting > 0:
                    line_bytes = serial_connection.readline()
                    if line_bytes:
                        line = line_bytes.decode('utf-8').strip()
                        match_x = x_pattern.search(line)
                        match_y = y_pattern.search(line)
                        match_temp = temp_pattern.search(line)
                        match_hum = hum_pattern.search(line)
                        match_pres = pres_pattern.search(line)
                        
                        if match_x and match_y:
                            x = float(match_x.group(1))
                            y = float(match_y.group(1))
                            root.after(0, lambda: (x_data_scatter.append(x), y_data_scatter.append(y), 
                                                   x_data_stats.append(x), y_data_stats.append(y)))
                        if match_temp:
                            temp = float(match_temp.group(1))
                            root.after(0, lambda: temp_data.append(temp))
                        if match_hum:
                            hum = float(match_hum.group(1))
                            root.after(0, lambda: hum_data.append(hum))
                        if match_pres:
                            pres = float(match_pres.group(1))
                            root.after(0, lambda: pres_data.append(pres))
                            
                        last_data_timestamp = time.time()
                time.sleep(0.01)
            else:
                time.sleep(1)
                setup_serial_connection()
        except serial.SerialException:
            plotting_active = False
            break
        except Exception:
            plotting_active = False
            break

def simulate_data():
    """Simulates wireless data reception for all plots."""
    global plotting_active, last_data_timestamp, data_source
    if not plotting_active or data_source != "simulation":
        return

    # Generate random data for all plots
    x_sim = random.uniform(-1000, 1000)
    y_sim = random.uniform(-1000, 1000)
    temp_sim = random.uniform(20, 30)
    hum_sim = random.uniform(50, 70)
    pres_sim = random.uniform(90, 110)
    
    # Add simulated data to the queues
    root.after(0, lambda: (x_data_scatter.append(x_sim), y_data_scatter.append(y_sim),
                           x_data_stats.append(x_sim), y_data_stats.append(y_sim),
                           temp_data.append(temp_sim), hum_data.append(hum_sim),
                           pres_data.append(pres_sim)))
    last_data_timestamp = time.time()
    
    # Schedule the next simulation call
    root.after(50, simulate_data)


def on_closing():
    global plotting_active, serial_connection
    plotting_active = False
    if 'data_thread' in globals() and data_thread.is_alive():
        data_thread.join(timeout=2)
    if serial_connection and serial_connection.is_open:
        try:
            serial_connection.close()
        except Exception:
            pass
    root.destroy()
    plt.close('all')
    sys.exit(0)

def capture_and_clear_map_data():
    global x_data_scatter, y_data_scatter
    fig_scatter.savefig(MAP_PREVIEW_PATH, facecolor=fig_scatter.get_facecolor(), bbox_inches='tight', pad_inches=0.1)
    messagebox.showinfo("Captura de Mapa", "Se ha guardado un mapa de los puntos actuales.")
    display_map_preview()
    x_data_scatter.clear()
    y_data_scatter.clear()
    root.after(0, update_gui_elements)

def check_data_flow():
    global last_data_timestamp, serial_status_var, data_source
    if plotting_active:
        current_time = time.time()
        # Only check timeout if we are in serial mode
        if data_source == "serial" and (current_time - last_data_timestamp) > DATA_TIMEOUT_SECONDS:
            if x_data_scatter or y_data_scatter:
                x_data_scatter.clear()
                y_data_scatter.clear()
                root.after(0, update_gui_elements)
                serial_status_var.set("Estado: Conectado (Sin Datos)")
        root.after(200, check_data_flow)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Monitor LIDAR de Robot C.A.T.R.A.")
    root.geometry("1400x700")
    root.minsize(1000, 600)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.configure(bg=BG_COLOR_DARKEST)

    style = ttk.Style()
    style.theme_use('clam')
    style.configure("TFrame", background=BG_COLOR_DARKEST)
    style.configure("TLabel", background=BG_COLOR_DARKEST, foreground=TEXT_COLOR)
    style.configure("Panel.TFrame", background=PANEL_COLOR, borderwidth=1, relief="flat")
    style.configure("PanelTitle.TLabel", background=PANEL_COLOR, foreground=TEXT_COLOR, font=('Segoe UI', 14, 'bold'))
    style.configure("Button.TButton", background=BUTTON_BG_COLOR, foreground=BUTTON_FG_COLOR, font=('Segoe UI', 10, 'bold'), borderwidth=0, relief="flat")
    style.configure("Status.TLabel", background=BG_COLOR_DARKEST, foreground='#2ECC71', font=('Segoe UI', 12, 'bold'))
    style.configure("Coord.TLabel", background=PANEL_COLOR, foreground=TEXT_COLOR, font=('Consolas', 18), padding=(5, 5))

    main_frame = ttk.Frame(root, style="TFrame", padding=10)
    main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Configure grid for horizontal layout
    main_frame.grid_columnconfigure(0, weight=1, minsize=600)
    main_frame.grid_columnconfigure(1, weight=1, minsize=400)
    main_frame.grid_rowconfigure(0, weight=1)

    # Left Panel: LIDAR plots
    lidar_panel_left = ttk.Frame(main_frame, style="Panel.TFrame", padding=10)
    lidar_panel_left.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
    lidar_panel_left.grid_rowconfigure(0, weight=1)
    lidar_panel_left.grid_rowconfigure(1, weight=1)
    lidar_panel_left.grid_columnconfigure(0, weight=1)

    # Top-left: Real-time LIDAR plot
    lidar_stats_frame = ttk.Frame(lidar_panel_left, style="Panel.TFrame", padding=10)
    lidar_stats_frame.grid(row=0, column=0, sticky="nsew", pady=5)
    ttk.Label(lidar_stats_frame, text="Real Time", style="PanelTitle.TLabel").pack(anchor="nw")
    setup_stats_plot_config()
    canvas_stats = FigureCanvasTkAgg(fig_stats, master=lidar_stats_frame)
    canvas_stats.get_tk_widget().pack(fill=tk.BOTH, expand=True, anchor='center', pady=(10, 0))

    # Bottom-left: Completed Map
    map_panel_left = ttk.Frame(lidar_panel_left, style="Panel.TFrame", padding=10)
    map_panel_left.grid(row=1, column=0, sticky="nsew", pady=5)
    ttk.Label(map_panel_left, text="Completed Map", style="PanelTitle.TLabel").pack(anchor="nw")
    map_preview_label = ttk.Label(map_panel_left, background=PANEL_COLOR, anchor="center",
                                  text="No hay mapa previo guardado.", foreground=MUTED_TEXT_COLOR,
                                  font=('Segoe UI', 12, 'italic'), relief="solid", borderwidth=1)
    map_preview_label.pack(fill=tk.BOTH, expand=True, anchor='center', padx=10, pady=10)

    # Right Panel: Sensor plots
    sensor_panel_right = ttk.Frame(main_frame, style="Panel.TFrame", padding=10)
    sensor_panel_right.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
    ttk.Label(sensor_panel_right, text="Real Time", style="PanelTitle.TLabel").pack(anchor="nw")
    
    setup_sensor_plots()
    canvas_sensors = FigureCanvasTkAgg(fig_sensors, master=sensor_panel_right)
    canvas_sensors.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(10, 0))
    
    # Bottom Control Panel
    bottom_control_panel = ttk.Frame(root, style="TFrame", padding=10)
    bottom_control_panel.pack(side=tk.BOTTOM, fill=tk.X)
    
    serial_status_var = tk.StringVar(value="Estado: Desconectado")
    status_label_widget = ttk.Label(bottom_control_panel, textvariable=serial_status_var, style="Status.TLabel")
    status_label_widget.pack(side=tk.LEFT)
    
    capture_button = ttk.Button(bottom_control_panel, text="Capturar y Limpiar", command=capture_and_clear_map_data, style="Button.TButton")
    capture_button.pack(side=tk.RIGHT)
    
    data_thread = threading.Thread(target=receive_from_serial, daemon=True)
    data_thread.start()

    root.after(50, update_gui_elements)
    root.after(200, check_data_flow)
    root.after(300, display_map_preview)

    root.mainloop()
