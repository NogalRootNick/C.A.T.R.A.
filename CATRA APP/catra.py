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

# --- Configuración del Cliente TCP ---
ESP32_IP = '192.168.137.97' # ¡IMPORTANTE! Reemplaza con la IP real de tu ESP32 (Estación Base).
ESP32_PORT = 8888

# --- Configuración para la Visualización (Matplotlib) ---
MAX_POINTS = 100 # Aumentado para ver más historial si los datos llegan rápido
x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)

# Variable de control del hilo de recepción
receiving_data = True

# Variables para almacenar los últimos valores mostrados en la GUI
last_x_val = None # Re-introducing Tkinter StringVar
last_y_val = None # Re-introducing Tkinter StringVar

# Variables globales para el gráfico de Matplotlib
fig = None
ax = None
line = None

# Global references for panels
top_panel = None # Referencia al panel superior (gráfico)
bottom_panel = None # Referencia al panel inferior (datos/info)

# Definir el color BLANCO para el fondo general
WHITE_BACKGROUND_COLOR = "#FFFFFF" # Blanco puro


# --- Función para Configurar el Gráfico Matplotlib ---
def setup_matplotlib_plot_config():
    """
    Configura los elementos visuales del gráfico Matplotlib,
    especificando colores para el fondo, ejes, cuadrícula y puntos.
    """
    global fig, ax, line

    # El fondo de la figura (área total del gráfico, incluyendo márgenes) será blanco.
    fig.patch.set_facecolor('white')
    
    # El fondo del área de trazado (donde se dibujan los puntos y los ejes)
    ax.set_facecolor('white')

    # Colores de los elementos del eje para contraste con fondo blanco
    ax.tick_params(axis='x', colors='black', labelsize=10) # Etiquetas de los ticks
    ax.tick_params(axis='y', colors='black', labelsize=10)
    ax.xaxis.label.set_color('black') # Etiqueta del eje X
    ax.yaxis.label.set_color('black') # Etiqueta del eje Y
    ax.title.set_color('black')        # Título del gráfico

    # Configuración de los bordes de los ejes (spines)
    ax.spines['left'].set_color('black')   # Borde izquierdo
    ax.spines['bottom'].set_color('black') # Borde inferior
    ax.spines['right'].set_visible(False)  # Oculta el borde derecho
    ax.spines['top'].set_visible(False)    # Oculta el borde superior
    ax.spines['left'].set_linewidth(1.5)   # Grosor del borde izquierdo
    ax.spines['bottom'].set_linewidth(1.5) # Grosor del borde inferior

    # Cuadrícula: Color negro y estilo punteado/transparente
    ax.grid(True, linestyle=':', alpha=0.8, color='black', linewidth=0.7) # Cuadrícula más fina y punteada

    # Límites fijos para los ejes X e Y para un espacio de mapeo consistente
    ax.set_xlim(-100, 200) # De -1500mm a 1500mm como en la imagen de referencia
    ax.set_ylim(-100, 200) # De -1500mm a 1500mm como en la imagen de referencia
    ax.set_aspect('equal', adjustable='box') # Mantiene la misma escala en ambos ejes (proporciones reales)

    # Título principal del gráfico
    #ax.set_title('Mapa de Puntos LIDAR', fontsize=18, color='black', pad=15) # Título como en la imagen de referencia
    #ax.set_xlabel('Coordenada X (mm)', fontsize=12, color='black') # Etiqueta X como en la imagen de referencia
    #ax.set_ylabel('Coordenada Y (mm)', fontsize=12, color='black') # Etiqueta Y como en la imagen de referencia (no visible en la imagen, pero implícita)

    # Configuración de los puntos del Lidar
    line.set_color('#1E90FF')       # Color de los puntos: Azul Doder (un azul vibrante)
    line.set_marker('o')           # Tipo de marcador (círculo)
    line.set_markersize(8)         # Tamaño de los puntos
    line.set_linestyle('None')     # Crucial: NO CONECTAR LOS PUNTOS con una línea
    line.set_alpha(0.9)            # Ligera transparencia para puntos superpuestos

    # Leyenda: Personalizada para integrar con el diseño
    ax.legend(facecolor='white', edgecolor='#333333', labelcolor='black', framealpha=0.8, fontsize=10, loc='upper right').get_frame().set_boxstyle("round,pad=0.5")


# --- Función para Actualizar el Gráfico ---
def update_plot(_):
    """
    Función de callback para Matplotlib FuncAnimation.
    Actualiza los datos del gráfico en tiempo real desde las colas globales
    y refresca la vista de los datos de X e Y en la GUI.
    """
    if receiving_data and line:
        line.set_data(list(x_data), list(y_data))
        fig.canvas.draw_idle() # Matplotlib maneja las actualizaciones de forma eficiente
        
        # Actualizar las etiquetas de X e Y en la GUI con los últimos valores
        if x_data and y_data:
            last_x_val.set(f"X: {x_data[-1]:.2f} mm")
            last_y_val.set(f"Y: {y_data[-1]:.2f} mm")
        else:
            last_x_val.set("X: ---")
            last_y_val.set("Y: ---")


# --- Función para el Manejo de la Conexión TCP y Recepción de Datos ---
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
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # Deshabilita el algoritmo de Nagle (reduce latencia)
            print("Conectado al ESP32 (Estación Base).")

            buffer_size = 4096 # Aumentado para manejar posibles ráfagas de datos más grandes
            buffer = "" 

            while receiving_data:
                try:
                    chunk = s.recv(buffer_size).decode('utf-8')
                    if not chunk: # Si el chunk está vacío, la conexión se ha cerrado
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


# --- Funciones de la GUI ---
def on_closing():
    """Maneja el evento de cierre de la ventana Tkinter, deteniendo los hilos."""
    global receiving_data
    receiving_data = False
    if data_thread.is_alive():
        data_thread.join(timeout=1) # Espera un poco a que el hilo termine
    root.destroy()
    plt.close('all') # Cierra todas las figuras de Matplotlib para liberar memoria


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

    # Márgenes: pequeño arriba, grande abajo, lados ajustados para la gráfica
    margin_top = 10  # Margen superior pequeño
    margin_sides = 10 # Margen lateral, reducido para que el gráfico ocupe más espacio
    margin_bottom = 50 # Margen inferior para la barra de datos

    available_width = canvas_width - 2 * margin_sides
    available_height = canvas_height - margin_top - margin_bottom

    if available_width < 0: available_width = 0
    if available_height < 0: available_height = 0

    # Altura del panel inferior (para X: Y: )
    bottom_panel_height = 50 # Altura fija para el panel de datos
    top_panel_height = available_height - bottom_panel_height

    # Posicionar top_panel (gráfico)
    top_panel.place(x=margin_sides, y=margin_top, 
                    width=available_width, height=top_panel_height)
    top_panel.grid_rowconfigure(0, weight=1)
    top_panel.grid_columnconfigure(0, weight=1)

    # Posicionar bottom_panel (datos)
    bottom_panel.place(x=margin_sides, y=margin_top + top_panel_height,
                       width=available_width, height=bottom_panel_height)
    
    # Configurar el grid layout dentro de bottom_panel para las etiquetas de datos X, Y
    # Colocamos las etiquetas en la esquina inferior izquierda
    bottom_panel.grid_columnconfigure(0, weight=0) # X Label (no expande)
    bottom_panel.grid_columnconfigure(1, weight=0) # Y Label (no expande)
    bottom_panel.grid_columnconfigure(2, weight=1) # Columna de relleno para empujar a la izquierda
    bottom_panel.grid_rowconfigure(0, weight=1) # Centrar verticalmente en la fila


# --- Código Principal de la Aplicación (Bloque de Ejecución) ---
if __name__ == "__main__":
    # --- 1. Configuración Inicial de la Ventana Tkinter ---
    root = tk.Tk()
    root.title("Monitor LIDAR de Robot C.A.T.R.A.")
    root.geometry("1200x900")
    root.minsize(800, 600)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.configure(bg=WHITE_BACKGROUND_COLOR) # Establece el color de fondo de la ventana principal a blanco

    # --- 2. Configuración de Estilos TTK (Themed Tkinter) ---
    style = ttk.Style()
    style.theme_use('clam')

    # Estilos Base (ajustados para el nuevo fondo blanco)
    style.configure("TFrame", background=WHITE_BACKGROUND_COLOR, foreground="black") 
    style.configure("TLabel", background=WHITE_BACKGROUND_COLOR, foreground="black", font=('Segoe UI', 11))
    style.configure("TButton", background="#DDDDDD", foreground="black", font=('Segoe UI', 10, 'bold'),
                    focuscolor="#CCCCCC", borderwidth=0, relief="flat")
    style.map("TButton", background=[('active', '#EEEEEE'), ('pressed', '#BBBBBB')])

    # Estilos Específicos para Paneles y Etiquetas
    # El panel inferior y sus etiquetas también serán blancos.
    style.configure("DataPanel.TFrame", background=WHITE_BACKGROUND_COLOR, relief="flat", borderwidth=0)
    
    # Estilo para las etiquetas de coordenadas X e Y:
    style.configure("CoordLabels.TLabel", 
                    background=WHITE_BACKGROUND_COLOR, 
                    foreground="black", # Texto negro para contraste con el blanco del panel inferior
                    font=('Consolas', 18, 'bold'), # Tamaño de fuente ajustado
                    padding=(5, 5)) # Padding ajustado

    # --- 3. Crear los Paneles Secundarios (Gráfico y Datos/Info) ---
    # top_panel y bottom_panel tendrán un fondo BLANCO.
    top_panel = tk.Frame(root, bg="white", relief="flat", bd=0) 
    bottom_panel = ttk.Frame(root, style="DataPanel.TFrame")

    # --- 4. Configuración Inicial de la Figura y Ejes de Matplotlib ---
    fig, ax = plt.subplots(figsize=(8, 6), dpi=100)
    line, = ax.plot([], [], 'o') 
    setup_matplotlib_plot_config()

    # --- 5. Incrustar el Gráfico de Matplotlib en el top_panel ---
    canvas_matplotlib = FigureCanvasTkAgg(fig, master=top_panel)
    canvas_widget = canvas_matplotlib.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=0, pady=0)

    # --- 6. Inicializar las Variables Tkinter para los Datos ---
    last_x_val = tk.StringVar(root, value="X: ---")
    last_y_val = tk.StringVar(root, value="Y: ---")

    # --- 7. Crear y Posicionar las Etiquetas de Datos en el bottom_panel ---
    # Colocar las etiquetas en la esquina inferior izquierda
    ttk.Label(bottom_panel, textvariable=last_x_val, style="CoordLabels.TLabel", anchor="w").grid(row=0, column=0, sticky="w", padx=(20, 5), pady=5)
    ttk.Label(bottom_panel, textvariable=last_y_val, style="CoordLabels.TLabel", anchor="w").grid(row=0, column=1, sticky="w", padx=5, pady=5)
    
    # Esto asegura que las etiquetas permanezcan a la izquierda y el espacio restante se tome por la columna 2
    bottom_panel.grid_columnconfigure(0, weight=0) 
    bottom_panel.grid_columnconfigure(1, weight=0)
    bottom_panel.grid_columnconfigure(2, weight=1) # Columna de relleno
    bottom_panel.grid_rowconfigure(0, weight=1)

    # --- 8. Iniciar el Hilo para la Recepción de Datos TCP ---
    data_thread = threading.Thread(target=receive_from_esp32, daemon=True)
    data_thread.start()

    # --- 9. Configurar la Animación de Matplotlib ---
    ani = animation.FuncAnimation(fig, update_plot, interval=10, cache_frame_data=False) 

    # --- 10. Iniciar el Posicionamiento de los Paneles (Inicial y Redimensionamiento) ---
    root.bind("<Configure>", lambda event: place_panels_on_canvas())
    root.after(100, place_panels_on_canvas)

    # --- 11. Iniciar el Bucle Principal de Tkinter ---
    root.mainloop()

    # --- 12. Código de Limpieza al Cerrar la Aplicación ---
    print("\nCerrando la aplicación...")
    if data_thread.is_alive():
        data_thread.join(timeout=1)
    print("Programa Python terminado.")