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

ESP32_IP = '192.168.137.97'
ESP32_PORT = 8888

MAX_POINTS = 100
x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)

receiving_data = True

last_x_val = None
last_y_val = None

fig = None
ax = None
line = None

top_panel = None
bottom_panel = None

WHITE_BACKGROUND_COLOR = "#FFFFFF"


def setup_matplotlib_plot_config():
    """
    Configura los elementos visuales del gráfico Matplotlib,
    especificando colores para el fondo, ejes, cuadrícula y puntos.
    """
    global fig, ax, line

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
    ax.spines['top'].set_visible(False)r
    ax.spines['left'].set_linewidth(1.5)
    ax.spines['bottom'].set_linewidth(1.5)

    ax.grid(True, linestyle=':', alpha=0.8, color='black', linewidth=0.7)

    ax.set_xlim(-100, 200)
    ax.set_ylim(-100, 200)
    ax.set_aspect('equal', adjustable='box')

    #ax.set_title('Mapa de Puntos LIDAR', fontsize=18, color='black', pad=15) # Título como en la imagen de referencia
    #ax.set_xlabel('Coordenada X (mm)', fontsize=12, color='black') # Etiqueta X como en la imagen de referencia
    #ax.set_ylabel('Coordenada Y (mm)', fontsize=12, color='black') # Etiqueta Y como en la imagen de referencia (no visible en la imagen, pero implícita)

    line.set_color('#1E90FF')
    line.set_marker('o')
    line.set_markersize(8)
    line.set_linestyle('None')
    line.set_alpha(0.9)

    ax.legend(facecolor='white', edgecolor='#333333', labelcolor='black', framealpha=0.8, fontsize=10, loc='upper right').get_frame().set_boxstyle("round,pad=0.5")


def update_plot(_):
    """
    Función de callback para Matplotlib FuncAnimation.
    Actualiza los datos del gráfico en tiempo real desde las colas globales
    y refresca la vista de los datos de X e Y en la GUI.
    """
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
    """
    Gestiona la conexión TCP con el ESP32, recibe datos y los procesa.
    Actualiza las variables de Tkinter para mostrar en la GUI y las colas de datos para el gráfico.
    """
    global receiving_data
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
        print("Tiempo de espera agotado al intentar conectar (esto no debería ocurrir si settimeout se eliminó de s.recv()).")
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
    """Maneja el evento de cierre de la ventana Tkinter, deteniendo los hilos."""
    global receiving_data
    receiving_data = False
    if data_thread.is_alive():
        data_thread.join(timeout=1)
    root.destroy()
    plt.close('all')


def place_panels_on_canvas():
    """
    Posiciona y redimensiona los paneles del gráfico y los datos sobre la ventana principal.
    Se asegura de mantener el margen deseado.
    """
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
    bottom_panel.grid_rowconfigure(0, weight=1)


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
                    font=('Consolas', 18, 'bold')
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
    
    bottom_panel.grid_columnconfigure(0, weight=0) 
    bottom_panel.grid_columnconfigure(1, weight=0)
    bottom_panel.grid_columnconfigure(2, weight=1)
    bottom_panel.grid_rowconfigure(0, weight=1)

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
