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
import json
import numpy as np
from PIL import Image, ImageTk
import random

# Define los patrones de expresiones regulares
x_pattern = re.compile(r"LIDAR COORD: X=([-+]?\d+\.?\d*)")
y_pattern = re.compile(r"Y=([-+]?\d+\.?\d*)")
temp_pattern = re.compile(r"Temp:\s*([-+]?\d+\.?\d*)")
hum_pattern = re.compile(r"Hum:\s*([-+]?\d+\.?\d*)")
pres_pattern = re.compile(r"Pres:\s*([-+]?\d+\.?\d*)")
raw_pattern = re.compile(r"RAW:\s*(\d+)")
map_complete_pattern = re.compile(r"MAP_COMPLETE\s+(.+)")

# --- Serial Port Configuration ---
SERIAL_PORT = 'AUTO'
MANUAL_FALLBACK_PORT = 'COM3'
BAUD_RATE = 9600
MAX_POINTS_SCATTER = 3000
DATA_TIMEOUT_SECONDS = 3.0

# Data collections
x_data_scatter = deque(maxlen=MAX_POINTS_SCATTER)
y_data_scatter = deque(maxlen=MAX_POINTS_SCATTER)
temp_data = deque(maxlen=200)
hum_data = deque(maxlen=200)
pres_data = deque(maxlen=200)
raw_data = deque(maxlen=200)
plotting_active = True
last_data_timestamp = time.time()
data_source = "serial"

# Matplotlib objects
fig_scatter, ax_scatter, line_scatter = None, None, None
fig_sensors, ax_temp, ax_hum, ax_pres = None, None, None, None
line_temp, line_hum, line_pres = None, None, None
serial_connection = None

# TKinter variable objects for live data
temp_var = None
hum_var = None
pres_var = None
raw_var = None

# Paleta de colores inspirada en VS Code Dark Modern
BG_COLOR_DARKEST = "#1E1E1E"
PANEL_COLOR = "#252526"
TEXT_COLOR = "#D4D4D4"
MUTED_TEXT_COLOR = "#FFFFFF"
ACCENT_BLUE = "#007ACC"
ACCENT_ORANGE = "#CE9178"
GRID_COLOR = "#FFFFFF"
BUTTON_BG_COLOR = "#0E639C"
BUTTON_FG_COLOR = "#FFFFFF"
LIDAR_POINT_COLOR = "#FFFFFF"
TEMP_LINE_COLOR = "#D16969"
HUM_LINE_COLOR = "#FFD700"
PRES_LINE_COLOR = "#6A9955"
CAPS = "catra_maps"

if not os.path.exists(CAPS):
    try:
        os.makedirs(CAPS)
    except OSError as e:
        messagebox.showerror("Error de Directorio", f"No se pudo crear la carpeta para mapas '{CAPS}':\n{e}")

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
    ax.grid(True, linestyle='-', alpha=0.6, color=GRID_COLOR, linewidth=1)
    ax.set_title(title, fontsize=12, pad=8, loc='left', weight='bold')
    ax.set_xlabel(xlabel, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)

def setup_scatter_plot_config():
    global fig_scatter, ax_scatter, line_scatter
    fig_scatter = plt.Figure(figsize=(8, 8), dpi=100)
    ax_scatter = fig_scatter.add_subplot(111)
    fig_scatter.patch.set_facecolor(PANEL_COLOR)
    configure_plot_style(ax_scatter, "", "X (mm)", "Y (mm)")
    ax_scatter.set_aspect('equal', adjustable='box')
    ax_scatter.set_xlim(-2000, 2000)
    ax_scatter.set_ylim(-2000, 2000)
    line_scatter, = ax_scatter.plot([], [], 'o', color=LIDAR_POINT_COLOR, markersize=2.5, alpha=0.7)
    ax_scatter.set_title("MAPA EN TIEMPO REAL", loc='left', color=TEXT_COLOR, fontsize=12, weight='bold')

def set_map_scale(size):
    if ax_scatter:
        ax_scatter.set_xlim(-size, size)
        ax_scatter.set_ylim(-size, size)
        fig_scatter.canvas.draw_idle()

def setup_sensor_plots():
    global fig_sensors, ax_temp, ax_hum, ax_pres, line_temp, line_hum, line_pres
    fig_sensors, (ax_temp, ax_hum, ax_pres) = plt.subplots(3, 1, figsize=(5, 9), dpi=100, sharex=True)
    fig_sensors.patch.set_facecolor(PANEL_COLOR)

    for ax, title, color in zip([ax_temp, ax_hum, ax_pres], ["Temperatura (C)", "Humedad (%)", "Presión (hPa)"], [TEMP_LINE_COLOR, HUM_LINE_COLOR, PRES_LINE_COLOR]):
        ax.set_facecolor(PANEL_COLOR)
        ax.tick_params(axis='y', colors=MUTED_TEXT_COLOR, labelsize=9)
        ax.title.set_color(TEXT_COLOR)
        ax.grid(True, linestyle='--', alpha=0.5, color=GRID_COLOR, linewidth=0.8)
        ax.set_title(title, loc='left', color=TEXT_COLOR, fontsize=10, weight='bold')
        ax.spines['left'].set_color(GRID_COLOR)
        ax.spines['bottom'].set_color(GRID_COLOR)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)

    ax_temp.set_ylabel("Temperatura (C)", fontsize=9, color=MUTED_TEXT_COLOR)
    ax_hum.set_ylabel("Humedad (%)", fontsize=9, color=MUTED_TEXT_COLOR)
    ax_pres.set_ylabel("Presión (hPa)", fontsize=9, color=MUTED_TEXT_COLOR)
    ax_pres.set_xlabel("Valores del Tiempo", fontsize=9, color=MUTED_TEXT_COLOR)
    ax_pres.set_xlim(0, 20)
    ax_pres.set_xticks(np.arange(0, 21, 5))
    ax_pres.set_xticklabels(np.arange(0, 21, 5), color=MUTED_TEXT_COLOR, fontsize=9)
    ax_temp.tick_params(axis='x', colors=MUTED_TEXT_COLOR, labelsize=9)
    line_temp, = ax_temp.plot([], [], color=TEMP_LINE_COLOR, linewidth=2)
    line_hum, = ax_hum.plot([], [], color=HUM_LINE_COLOR, linewidth=2)
    line_pres, = ax_pres.plot([], [], color=PRES_LINE_COLOR, linewidth=2)
    fig_sensors.tight_layout(pad=2.0, h_pad=0.5, w_pad=0.5, rect=[0, 0, 1, 1])

def update_gui_elements():
    global plotting_active
    if not plotting_active:
        return
    if line_scatter and x_data_scatter and y_data_scatter:
        line_scatter.set_data(list(x_data_scatter), list(y_data_scatter))
        ax_scatter.relim()
        ax_scatter.autoscale_view()
        fig_scatter.canvas.draw_idle()

    for line, data in zip([line_temp, line_hum, line_pres], [temp_data, hum_data, pres_data]):
        if line:
            line.set_data(range(len(data)), data)
            ax = line.axes
            ax.set_xlim(0, max(20, len(data)))
            ax.relim()
            ax.autoscale_view(True, True, True)
    if fig_sensors:
        fig_sensors.canvas.draw_idle()

    if temp_data: temp_var.set(f"{temp_data[-1]:.1f}C")
    if hum_data: hum_var.set(f"{hum_data[-1]:.1f}%")
    if pres_data: pres_var.set(f"{pres_data[-1]:.2f} hPa")
    if raw_data: raw_var.set(f"{raw_data[-1]}")
    root.after(200, update_gui_elements)

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    keywords = ["arduino", "usb-serial", "ch340", "ft232r", "usbmodem", "cp210x"]
    for p in ports:
        desc = p.description.lower()
        hwid = p.hwid.lower()
        if any(keyword in desc for keyword in keywords) or any(keyword in hwid for keyword in keywords):
            print(f"Arduino found: {p.device} ({p.description})")
            return p.device
    print("No Arduino found automatically.")
    return None

def setup_serial_connection():
    global serial_connection, data_source
    if serial_connection and serial_connection.is_open:
        return True

    port_to_use = SERIAL_PORT
    if port_to_use == 'AUTO':
        port_to_use = find_arduino_port()

    if not port_to_use:
        return False

    try:
        serial_connection = serial.Serial(port_to_use, BAUD_RATE, timeout=1.0)
        time.sleep(1.5)
        serial_connection.reset_input_buffer()
        data_source = "serial"
        return True
    except serial.SerialException as e:
        return False

def receive_from_serial():
    global plotting_active, serial_connection, last_data_timestamp, data_source
    
    while plotting_active:
        if not (serial_connection and serial_connection.is_open):
            if not setup_serial_connection():
                time.sleep(5)
                continue

        data_source = "serial"
        try:
            if serial_connection.in_waiting > 0:
                line = serial_connection.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                last_data_timestamp = time.time()
                
                map_match = map_complete_pattern.match(line)
                if map_match:
                    map_name = map_match.group(1).strip()
                    root.after(0, _capture_and_clear_map_data_logic, map_name)
                    continue

                x_match = x_pattern.search(line)
                y_match = y_pattern.search(line)
                temp_match = temp_pattern.search(line)
                hum_match = hum_pattern.search(line)
                pres_match = pres_pattern.search(line)
                raw_match = raw_pattern.search(line)

                if x_match and y_match:
                    try:
                        x = float(x_match.group(1))
                        y = float(y_match.group(1))
                        root.after(0, lambda x=x, y=y: (x_data_scatter.append(x), y_data_scatter.append(y)))
                    except (ValueError, IndexError):
                        pass

                if temp_match:
                    try:
                        temp = float(temp_match.group(1))
                        root.after(0, lambda t=temp: temp_data.append(t))
                    except (ValueError, IndexError):
                        pass

                if hum_match:
                    try:
                        hum = float(hum_match.group(1))
                        root.after(0, lambda h=hum: hum_data.append(h))
                    except (ValueError, IndexError):
                        pass

                if pres_match:
                    try:
                        pres = float(pres_match.group(1))
                        root.after(0, lambda p=pres: pres_data.append(p))
                    except (ValueError, IndexError):
                        pass
                
                if raw_match:
                    try:
                        raw = int(raw_match.group(1))
                        root.after(0, lambda r=raw: raw_data.append(r))
                    except (ValueError, IndexError):
                        pass
            else:
                time.sleep(0.01)
        except (serial.SerialException, OSError) as e:
            print(f"Serial connection error: {e}")
            if serial_connection and serial_connection.is_open:
                serial_connection.close()
            serial_connection = None
            time.sleep(2)
        except Exception as e:
            print(f"An unexpected error occurred in receive_from_serial: {e}")
            plotting_active = False
            break

def on_closing():
    global plotting_active, serial_connection
    plotting_active = False
    if 'data_thread' in globals() and data_thread.is_alive():
        data_thread.join(timeout=1)
    if serial_connection and serial_connection.is_open:
        serial_connection.close()
    root.destroy()
    plt.close('all')
    sys.exit(0)

def _capture_and_clear_map_data_logic(unique_map_filename=None):
    if not x_data_scatter:
        messagebox.showwarning("Mapa Vacio", "No hay puntos para guardar.")
        return

    if unique_map_filename is None:
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        unique_map_filename = f"map_{timestamp}.json"
    else:
        if not unique_map_filename.lower().endswith('.json'):
            unique_map_filename += '.json'

    unique_map_path = os.path.join(CAPS, unique_map_filename)

    avg_temp = float(np.mean(temp_data)) if temp_data else None
    avg_hum = float(np.mean(hum_data)) if hum_data else None
    avg_pres = float(np.mean(pres_data)) if pres_data else None

    map_data = {
        "timestamp": time.time(),
        "lidar_points": list(zip(list(x_data_scatter), list(y_data_scatter))),
        "sensor_data": {
            "temperature": avg_temp,
            "humidity": avg_hum,
            "pressure": avg_pres
        }
    }

    try:
        with open(unique_map_path, 'w') as f:
            json.dump(map_data, f, indent=4)
        
        messagebox.showinfo("Captura de Mapa", f"Datos del mapa guardados como {unique_map_filename}")
        x_data_scatter.clear()
        y_data_scatter.clear()
        temp_data.clear()
        hum_data.clear()
        pres_data.clear()

    except Exception as e:
        messagebox.showerror("Error al Guardar", f"No se pudo guardar el archivo de datos del mapa:\n{e}")

def capture_and_clear_map_data():
    _capture_and_clear_map_data_logic()

def check_data_flow():
    if plotting_active and data_source == "serial":
        if (time.time() - last_data_timestamp) > DATA_TIMEOUT_SECONDS:
            if serial_connection and serial_connection.is_open:
                root.title(f'Interfaz C.A.T.R.A. | Conectado a {serial_connection.port} (Sin Datos)')
    root.after(1000, check_data_flow)

def update_connection_status():
    global serial_connection
    title_text = 'Interfaz C.A.T.R.A. | Desconectado'
    if serial_connection and serial_connection.is_open:
        title_text = f'Interfaz C.A.T.R.A. | Conectado a {serial_connection.port}'
    root.title(title_text)
    root.after(1000, update_connection_status)

if __name__ == "__main__":
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.configure(bg=BG_COLOR_DARKEST)

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)

    style = ttk.Style()
    style.theme_use('clam')
    style.configure("TFrame", background=BG_COLOR_DARKEST)
    style.configure("Panel.TFrame", background=PANEL_COLOR)
    style.configure("TLabel", background=BG_COLOR_DARKEST, foreground=TEXT_COLOR)
    style.configure("Panel.TLabel", background=PANEL_COLOR, foreground=TEXT_COLOR)
    style.configure("Title.TLabel", background=PANEL_COLOR, foreground=TEXT_COLOR, font=('Segoe UI', 14, 'bold'))
    style.configure("Value.TLabel", background=PANEL_COLOR, foreground=ACCENT_BLUE, font=('Consolas', 28, 'bold'))
    style.configure("Unit.TLabel", background=PANEL_COLOR, foreground=MUTED_TEXT_COLOR, font=('Segoe UI', 12))
    style.configure("TButton", font=('Segoe UI', 10, 'bold'), foreground=BUTTON_FG_COLOR, background=BUTTON_BG_COLOR, borderwidth=0)
    style.map("TButton", background=[('active', ACCENT_ORANGE)])
    style.configure("Scale.TButton", font=('Segoe UI', 10, 'bold'), foreground=BUTTON_FG_COLOR, background=ACCENT_BLUE, borderwidth=0)
    style.map("Scale.TButton", background=[('active', ACCENT_ORANGE)])
    style.configure("Emoji.TButton", font=('Segoe UI Emoji', 16), padding=5)
    style.configure("Gallery.TButton", font=('Segoe UI', 10), foreground=TEXT_COLOR, background=PANEL_COLOR, borderwidth=0)
    style.map("Gallery.TButton", background=[('active', ACCENT_BLUE)])

    main_view_container = ttk.Frame(root, style="TFrame", padding=10)
    main_view_container.grid(row=0, column=0, sticky="nsew")
    main_view_container.grid_rowconfigure(0, weight=1)
    main_view_container.grid_columnconfigure(0, weight=1)

    monitor_frame = ttk.Frame(main_view_container, style="TFrame")
    monitor_frame.grid_columnconfigure(0, weight=1)
    monitor_frame.grid_columnconfigure(1, weight=1)
    monitor_frame.grid_rowconfigure(0, weight=1)
    monitor_frame.grid_rowconfigure(1, weight=0)
    monitor_frame.grid_rowconfigure(2, weight=0)

    scatter_panel = ttk.Frame(monitor_frame, style="Panel.TFrame")
    scatter_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
    setup_scatter_plot_config()
    canvas_scatter = FigureCanvasTkAgg(fig_scatter, master=scatter_panel)
    canvas_scatter.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    scale_buttons_frame = ttk.Frame(monitor_frame, style="Panel.TFrame")
    scale_buttons_frame.grid(row=1, column=0, sticky="ew", padx=(0, 5), pady=(5, 5))
    scale_buttons_frame.grid_columnconfigure(0, weight=1)
    scale_buttons_frame.grid_columnconfigure(1, weight=0)
    scale_buttons_frame.grid_columnconfigure(2, weight=0)
    scale_buttons_frame.grid_columnconfigure(3, weight=0)
    scale_buttons_frame.grid_columnconfigure(4, weight=0)
    scale_buttons_frame.grid_columnconfigure(5, weight=1)

    ttk.Label(scale_buttons_frame, text="Escala:", style="Unit.TLabel").grid(row=0, column=1, padx=(0, 5))
    ttk.Button(scale_buttons_frame, text="250x250", command=lambda: set_map_scale(250), style="Scale.TButton").grid(row=0, column=2, padx=5)
    ttk.Button(scale_buttons_frame, text="1000x1000", command=lambda: set_map_scale(1000), style="Scale.TButton").grid(row=0, column=3, padx=5)
    ttk.Button(scale_buttons_frame, text="1500x1500", command=lambda: set_map_scale(1500), style="Scale.TButton").grid(row=0, column=4, padx=5)

    sensor_panel = ttk.Frame(monitor_frame, style="Panel.TFrame")
    sensor_panel.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0), pady=(0, 5))
    setup_sensor_plots()
    canvas_sensors = FigureCanvasTkAgg(fig_sensors, master=sensor_panel)
    canvas_sensors.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    info_panel = ttk.Frame(monitor_frame, style="Panel.TFrame")
    info_panel.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(5, 0))
    info_panel.grid_columnconfigure(0, weight=1)
    info_panel.grid_columnconfigure(1, weight=0)

    climate_frame = ttk.Frame(info_panel, style="Panel.TFrame")
    climate_frame.grid(row=0, column=0, sticky="nsew")

    temp_var, hum_var = tk.StringVar(value="--.-C"), tk.StringVar(value="--.-%")
    pres_var, raw_var = tk.StringVar(value="---.-- hPa"), tk.StringVar(value="----")

    for i, (var, label) in enumerate([(temp_var, "Temperatura"), (hum_var, "Humedad"), (pres_var, "Presión"), (raw_var, "RAW")]):
        climate_frame.grid_columnconfigure(i, weight=1)
        f = ttk.Frame(climate_frame, style="Panel.TFrame")
        f.grid(row=0, column=i, sticky="ew", padx=10, pady=10)
        ttk.Label(f, text=label, style="Unit.TLabel", anchor="center").pack()
        ttk.Label(f, textvariable=var, style="Value.TLabel", anchor="center").pack()

    actions_panel = ttk.Frame(info_panel, style="Panel.TFrame")
    actions_panel.grid(row=0, column=1, sticky="e", padx=10, pady=5)

    broom_button = ttk.Button(actions_panel, text="🧹", command=capture_and_clear_map_data, style="Emoji.TButton")
    broom_button.pack(side=tk.RIGHT, padx=5)

    gallery_button = ttk.Button(actions_panel, text="🗺️", command=lambda: show_view("gallery"), style="Emoji.TButton")
    gallery_button.pack(side=tk.RIGHT, padx=5)

    gallery_frame = ttk.Frame(main_view_container, style="TFrame")
    gallery_frame.grid_columnconfigure(0, weight=1)
    gallery_frame.grid_columnconfigure(1, weight=3)
    gallery_frame.grid_rowconfigure(0, weight=1)

    map_list_panel = ttk.Frame(gallery_frame, style="Panel.TFrame")
    map_list_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
    map_list_panel.grid_rowconfigure(2, weight=1)
    map_list_panel.grid_columnconfigure(0, weight=1)
    ttk.Button(map_list_panel, text="< Volver al Monitor", command=lambda: show_view("monitor"), style="TButton").grid(row=0, column=0, sticky="ew", padx=10, pady=10)
    ttk.Label(map_list_panel, text="Mapas Guardados", style="Title.TLabel").grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 5))
    map_list_canvas = tk.Canvas(map_list_panel, background=PANEL_COLOR, highlightthickness=0)
    map_list_scrollbar = ttk.Scrollbar(map_list_panel, orient="vertical", command=map_list_canvas.yview)
    map_list_scrollable_frame = ttk.Frame(map_list_canvas, style="Panel.TFrame")
    map_list_scrollable_frame.bind("<Configure>", lambda e: map_list_canvas.configure(scrollregion=map_list_canvas.bbox("all")))
    map_list_canvas.create_window((0, 0), window=map_list_scrollable_frame, anchor="nw")
    map_list_canvas.configure(yscrollcommand=map_list_scrollbar.set)
    map_list_canvas.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)
    map_list_scrollbar.grid(row=2, column=1, sticky="ns", pady=5)

    gallery_display_panel = ttk.Frame(gallery_frame, style="Panel.TFrame")
    gallery_display_panel.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 5))
    gallery_display_panel.grid_rowconfigure(0, weight=1)
    gallery_display_panel.grid_rowconfigure(1, weight=0)
    gallery_display_panel.grid_rowconfigure(2, weight=0)
    gallery_display_panel.grid_columnconfigure(0, weight=1)

    fig_gallery, ax_gallery = plt.subplots(figsize=(8, 8), dpi=100)
    fig_gallery.patch.set_facecolor(PANEL_COLOR)
    configure_plot_style(ax_gallery, "Mapa Seleccionado", "X (mm)", "Y (mm)")
    ax_gallery.set_aspect('equal', adjustable='box')
    line_gallery, = ax_gallery.plot([], [], 'o', color=LIDAR_POINT_COLOR, markersize=2.5, alpha=0.7)
    
    canvas_gallery = FigureCanvasTkAgg(fig_gallery, master=gallery_display_panel)
    canvas_gallery.get_tk_widget().grid(row=0, column=0, sticky="nsew", pady=5)

    gallery_scale_buttons_frame = ttk.Frame(gallery_display_panel, style="Panel.TFrame")
    gallery_scale_buttons_frame.grid(row=1, column=0, sticky="ew", pady=(10, 5))
    gallery_scale_buttons_frame.grid_columnconfigure(0, weight=1)
    gallery_scale_buttons_frame.grid_columnconfigure(1, weight=0)
    gallery_scale_buttons_frame.grid_columnconfigure(2, weight=0)
    gallery_scale_buttons_frame.grid_columnconfigure(3, weight=0)
    gallery_scale_buttons_frame.grid_columnconfigure(4, weight=0)
    gallery_scale_buttons_frame.grid_columnconfigure(5, weight=1)

    def set_gallery_map_scale(size):
        ax_gallery.set_xlim(-size, size)
        ax_gallery.set_ylim(-size, size)
        canvas_gallery.draw_idle()

    ttk.Label(gallery_scale_buttons_frame, text="Escala:", style="Unit.TLabel").grid(row=0, column=1, padx=(0, 5))
    ttk.Button(gallery_scale_buttons_frame, text="250x250", command=lambda: set_gallery_map_scale(250), style="Scale.TButton").grid(row=0, column=2, padx=5)
    ttk.Button(gallery_scale_buttons_frame, text="1000x1000", command=lambda: set_gallery_map_scale(1000), style="Scale.TButton").grid(row=0, column=3, padx=5)
    ttk.Button(gallery_scale_buttons_frame, text="1500x1500", command=lambda: set_gallery_map_scale(1500), style="Scale.TButton").grid(row=0, column=4, padx=5)

    ttk.Separator(gallery_display_panel, orient="horizontal").grid(row=2, column=0, sticky="ew", padx=10, pady=10)

    gallery_sensor_data_frame = ttk.Frame(gallery_display_panel, style="Panel.TFrame")
    gallery_sensor_data_frame.grid(row=3, column=0, sticky="ew", pady=(5, 10))

    gallery_temp_var = tk.StringVar(value="--.- C")
    gallery_hum_var = tk.StringVar(value="--.- %")
    gallery_pres_var = tk.StringVar(value="---.-- hPa")
    
    gallery_sensor_labels = ["Temperatura", "Humedad", "Presión"]
    gallery_sensor_vars = [gallery_temp_var, gallery_hum_var, gallery_pres_var]

    for i, (var, label) in enumerate(zip(gallery_sensor_vars, gallery_sensor_labels)):
        gallery_sensor_data_frame.grid_columnconfigure(i, weight=1)
        f = ttk.Frame(gallery_sensor_data_frame, style="Panel.TFrame")
        f.grid(row=0, column=i, sticky="ew", padx=10, pady=5)
        ttk.Label(f, text=label, style="Unit.TLabel", anchor="center").pack()
        ttk.Label(f, textvariable=var, style="Value.TLabel", anchor="center").pack()

    current_view_frame = None

    def show_view(view_name):
        global current_view_frame
        if current_view_frame: current_view_frame.grid_remove()
        if view_name == "monitor":
            current_view_frame = monitor_frame
            monitor_frame.grid(row=0, column=0, sticky="nsew")
        elif view_name == "gallery":
            current_view_frame = gallery_frame
            gallery_frame.grid(row=0, column=0, sticky="nsew")
            populate_map_list()

    def populate_map_list():
        for widget in map_list_scrollable_frame.winfo_children():
            widget.destroy()

        maps = [f for f in os.listdir(CAPS) if f.endswith(".json")]
        if not maps:
            ttk.Label(map_list_scrollable_frame, text="No hay mapas guardados.", style="Panel.TLabel").pack(pady=10)
            return
        for map_file in sorted(maps, reverse=True):
            map_path = os.path.join(CAPS, map_file)
            btn = ttk.Button(map_list_scrollable_frame, text=map_file, style="TButton",
                             command=lambda p=map_path: display_dynamic_map(p))
            btn.pack(fill="x", padx=5, pady=2)

    def display_dynamic_map(file_path):
        try:
            with open(file_path, 'r') as f:
                map_data = json.load(f)
            
            points = map_data.get("lidar_points", [])
            if not points:
                line_gallery.set_data([], [])
                ax_gallery.set_title("Mapa Vacio", loc='left')
            else:
                x_coords, y_coords = zip(*points)
                line_gallery.set_data(x_coords, y_coords)
                ax_gallery.set_title(os.path.basename(file_path), loc='left')
                ax_gallery.relim()
                ax_gallery.autoscale_view()

            sensor_data = map_data.get("sensor_data", {})
            temp = sensor_data.get('temperature')
            hum = sensor_data.get('humidity')
            pres = sensor_data.get('pressure')

            gallery_temp_var.set(f"{temp:.1f} C" if temp is not None else "--.- C")
            gallery_hum_var.set(f"{hum:.1f} %" if hum is not None else "--.- %")
            gallery_pres_var.set(f"{pres:.2f} hPa" if pres is not None else "---.-- hPa")

            canvas_gallery.draw_idle()

        except Exception as e:
            messagebox.showerror("Error al Cargar Mapa", f"No se pudo cargar el archivo de datos:\n{e}")

    data_thread = threading.Thread(target=receive_from_serial, daemon=True)
    data_thread.start()

    show_view("monitor")
    root.after(200, update_gui_elements)
    root.after(1000, check_data_flow)
    update_connection_status()

    root.mainloop()
    on_closing()
    plt.close('all')
    sys.exit(0)