import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.animation as animation
from collections import deque
import re
from PIL import Image, ImageTk, ImageDraw
import numpy as np
import os
import random

ESP32_IP = '192.168.137.23'
ESP32_PORT = 8888

MAX_POINTS = 500

x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)

receiving_data = True

first_data_received_flag = False

last_x_val = None
last_y_val = None

fig = None
ax = None
line = None

top_panel = None
bottom_panel = None

WHITE_BACKGROUND_COLOR = "#FFFFFF"

CAPS = "plots_catra"

if not os.path.exists(CAPS):
    os.makedirs(CAPS)

map_configurations = [
    {
        "name": "Mapa Área Cercana (0-200mm)",
        "xlim": (-250, 250),
        "ylim": (-250, 250),
        "title": "Monitor LIDAR: Área Cercana (0-200mm)",
        "point_color": '#1E90FF'
    },
    {
        "name": "Mapa Área General (-1500-1500mm)",
        "xlim": (-1500, 1500),
        "ylim": (-1500, 1500),
        "title": "Monitor LIDAR: Área General (-1500-1500mm)",
        "point_color": '#FF4500'
    },
    {
        "name": "Mapa Cuadrante Superior (0-1000mm)",
        "xlim": (0, 1000),
        "ylim": (0, 1000),
        "title": "Monitor LIDAR: Cuadrante Superior (0-1000mm)",
        "point_color": '#32CD32'
    }
]
current_map_index = 0

def setup_matplotlib_plot_config():
    global fig, ax, line, current_map_index

    config = map_configurations[current_map_index]

    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    ax.tick_params(axis='x', colors='black', labelsize=10)
    ax.tick_params(axis='y', colors='black', labelsize=10)
    ax.xaxis.label.set_color('black')
    ax.yaxis.label.set_color('black')
    ax.title.set_color('black')

    ax.spines['left'].set_color('black')
    ax.spines['bottom'].set_color('black')
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_linewidth(1.5)
    ax.spines['bottom'].set_linewidth(1.5)

    ax.grid(True, linestyle=':', alpha=0.8, color='black', linewidth=0.7)

    ax.set_xlim(config["xlim"][0], config["xlim"][1])
    ax.set_ylim(config["ylim"][0], config["ylim"][1])
    ax.set_aspect('equal', adjustable='box')

    ax.set_title(config["title"], fontsize=18, color='black', pad=15)
    ax.set_xlabel('Coordenada X (mm)', fontsize=12, color='black')
    ax.set_ylabel('Coordenada Y (mm)', fontsize=12, color='black')

    line.set_color(config["point_color"])
    line.set_marker('o')
    line.set_markersize(8)
    line.set_linestyle('None')
    line.set_alpha(0.9)

def update_plot(_):
    if receiving_data and line:
        line.set_data(list(x_data), list(y_data))
        fig.canvas.draw_idle()

        if x_data and y_data:
            last_x_val.set(f"X: {x_data[-1]:.2f} mm")
            last_y_val.set(f"Y: {y_data[-1]:.2f} mm")
        else:
            last_x_val.set("X: ---")
            last_y_val.set("Y: ---")

def receive_from_esp32():
    global receiving_data, first_data_received_flag
    print(f"Intentando conectar al ESP32 (Estación Base) en {ESP32_IP}:{ESP32_PORT}...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ESP32_IP, ESP32_PORT))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Conectado al ESP32 (Estación Base).")

            buffer_size = 4096
            buffer = ""

            while receiving_data:
                try:
                    chunk = s.recv(buffer_size).decode('utf-8')
                    if not chunk:
                        print("Conexión con ESP32 cerrada inesperadamente.")
                        break

                    buffer += chunk

                    while "\n" in buffer:
                        line_data, buffer = buffer.split('\n', 1)
                        data_to_process = line_data.strip()

                        if data_to_process.startswith("LIDAR_DATA_TCP:"):
                            parts_str = data_to_process[len("LIDAR_DATA_TCP:"):].strip()
                            parts = parts_str.split(',')
                            parsed_data = {}
                            for part in parts:
                                if ':' in part:
                                    key, value = part.split(':', 1)
                                    parsed_data[key.strip()] = value.strip()

                            try:
                                coord_x = float(parsed_data.get('X', '0.0'))
                                coord_y = float(parsed_data.get('Y', '0.0'))

                                if not first_data_received_flag:
                                    x_data.clear()
                                    y_data.clear()
                                    first_data_received_flag = True
                                    print("Mapa reiniciado después de la primera coordenada válida.")

                                x_data.append(coord_x)
                                y_data.append(coord_y)

                            except ValueError as e:
                                print(f"Error al parsear datos numéricos: {e} - Datos RAW: {data_to_process}")
                            except KeyError as e:
                                print(f"Error: clave esperada no encontrada: {e} - Datos RAW: {data_to_process}")
                        else:
                            print(f"Mensaje no reconocido: {data_to_process}")

                except socket.error as e:
                    print(f"Error de socket durante la recepción: {e}")
                    break
                except Exception as e:
                    print(f"Error inesperado al recibir datos: {e}")
                    break

    except ConnectionRefusedError:
        print("Conexión rechazada. Asegúrate de que el ESP32 (Estación Base) esté encendido, conectado al Wi-Fi, y la IP sea correcta.")
        if last_x_val:
            last_x_val.set("X: ERROR")
            last_y_val.set("Y: ERROR")
    except socket.timeout:
        print("Tiempo de espera agotado al intentar conectar.")
        if last_x_val:
            last_x_val.set("X: TIMEOUT")
            last_y_val.set("Y: TIMEOUT")
    except Exception as e:
        print(f"Error al intentar conectar: {e}")
        if last_x_val:
            last_x_val.set("X: ERROR")
            last_y_val.set("Y: ERROR")
    finally:
        receiving_data = False
        print("Hilo de recepción de datos TCP finalizado.")

def on_closing():
    global receiving_data
    receiving_data = False
    if data_thread.is_alive():
        data_thread.join(timeout=1)
    root.destroy()
    plt.close('all')

def place_panels_on_canvas():
    global top_panel, bottom_panel

    canvas_width = root.winfo_width()
    canvas_height = root.winfo_height()

    if canvas_width < 100 or canvas_height < 100:
        root.after(100, place_panels_on_canvas)
        return

    margin_top = 10
    margin_sides = 10
    margin_bottom = 50

    available_width = canvas_width - 2 * margin_sides
    available_height = canvas_height - margin_top - margin_bottom

    if available_width < 0: available_width = 0
    if available_height < 0: available_height = 0

    bottom_panel_height = 50
    top_panel_height = available_height - bottom_panel_height

    top_panel.place(x=margin_sides, y=margin_top,
                    width=available_width, height=top_panel_height)
    top_panel.grid_rowconfigure(0, weight=1)
    top_panel.grid_columnconfigure(0, weight=1)

    bottom_panel.place(x=margin_sides, y=margin_top + top_panel_height,
                        width=available_width, height=bottom_panel_height)

    bottom_panel.grid_columnconfigure(0, weight=0)
    bottom_panel.grid_columnconfigure(1, weight=0)
    bottom_panel.grid_columnconfigure(2, weight=1)

    for i in range(len(map_configurations)):
        bottom_panel.grid_columnconfigure(3 + i, weight=0)

    bottom_panel.grid_columnconfigure(3 + len(map_configurations), weight=0)

    bottom_panel.grid_rowconfigure(0, weight=1)

def switch_map_config(map_index):
    global current_map_index, first_data_received_flag
    if map_index < 0 or map_index >= len(map_configurations):
        print(f"Error: Índice de mapa {map_index} fuera de rango.")
        return

    current_map_index = map_index
    print(f"Cambiando a mapa: {map_configurations[current_map_index]['name']}")

    x_data.clear()
    y_data.clear()
    first_data_received_flag = False

    setup_matplotlib_plot_config()
    update_plot(None)

def take_screenshot_and_clean_map():
    global x_data, y_data, first_data_received_flag

    timestamp = int(time.time())
    random_id = random.randint(1000, 9999)
    filename = os.path.join(CAPS, f"catra_map_{timestamp}_{random_id}.png")

    try:
        fig.savefig(filename, dpi=300, bbox_inches='tight', facecolor=fig.get_facecolor())
        print(f"Captura de pantalla guardada en: {filename}")
    except Exception as e:
        print(f"Error al guardar la captura de pantalla: {e}")

    x_data.clear()
    y_data.clear()
    first_data_received_flag = False
    print("Mapa limpiado.")
    update_plot(None)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Monitor LIDAR de Robot C.A.T.R.A.")
    root.geometry("1200x900")
    root.minsize(800, 600)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.configure(bg=WHITE_BACKGROUND_COLOR)

    style = ttk.Style()
    style.theme_use('clam')

    style.configure("TFrame", background=WHITE_BACKGROUND_COLOR, foreground="black")
    style.configure("TLabel", background=WHITE_BACKGROUND_COLOR, foreground="black", font=('Segoe UI', 11))
    style.configure("TButton", background="#DDDDDD", foreground="black", font=('Segoe UI', 10, 'bold'),
                    focuscolor="#CCCCCC", borderwidth=0, relief="flat")
    style.map("TButton", background=[('active', '#EEEEEE'), ('pressed', '#BBBBBB')])

    style.configure("DataPanel.TFrame", background=WHITE_BACKGROUND_COLOR, relief="flat", borderwidth=0)

    style.configure("CoordLabels.TLabel",
                    background=WHITE_BACKGROUND_COLOR,
                    foreground="black",
                    font=('Consolas', 18, 'bold'),
                    padding=(5, 5))

    top_panel = tk.Frame(root, bg="white", relief="flat", bd=0)
    bottom_panel = ttk.Frame(root, style="DataPanel.TFrame")

    fig, ax = plt.subplots(figsize=(8, 6), dpi=100)
    line, = ax.plot([], [], 'o')

    setup_matplotlib_plot_config()

    canvas_matplotlib = FigureCanvasTkAgg(fig, master=top_panel)
    canvas_widget = canvas_matplotlib.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=0, pady=0)

    last_x_val = tk.StringVar(root, value="X: ---")
    last_y_val = tk.StringVar(root, value="Y: ---")

    ttk.Label(bottom_panel, textvariable=last_x_val, style="CoordLabels.TLabel", anchor="w").grid(row=0, column=0, sticky="w", padx=(20, 5), pady=5)
    ttk.Label(bottom_panel, textvariable=last_y_val, style="CoordLabels.TLabel", anchor="w").grid(row=0, column=1, sticky="w", padx=5, pady=5)

    map_button_frame = ttk.Frame(bottom_panel, style="DataPanel.TFrame")
    map_button_frame.grid(row=0, column=2, sticky="w", padx=(10, 20), pady=5)

    for i, config in enumerate(map_configurations):
        ttk.Button(map_button_frame, text=config["name"], command=lambda idx=i: switch_map_config(idx)).pack(side=tk.LEFT, padx=5)

    clean_button_frame = ttk.Frame(bottom_panel, style="DataPanel.TFrame")
    clean_button_frame.grid(row=0, column=3, sticky="e", padx=(10, 20), pady=5)
    ttk.Button(clean_button_frame, text="Limpiar Mapa y Capturar", command=take_screenshot_and_clean_map).pack(side=tk.RIGHT, padx=5)

    data_thread = threading.Thread(target=receive_from_esp32, daemon=True)
    data_thread.start()

    ani = animation.FuncAnimation(fig, update_plot, interval=10, cache_frame_data=False)

    root.bind("<Configure>", lambda event: place_panels_on_canvas())
    root.after(100, place_panels_on_canvas)

    root.mainloop()

    print("\nCerrando la aplicación...")
    if data_thread.is_alive():
        data_thread.join(timeout=1)
    print("Programa Python terminado.")
