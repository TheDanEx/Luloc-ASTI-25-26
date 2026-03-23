import skimage as ski
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
import numpy as np
from picamera2 import Picamera2
from pprint import pprint
# ---------------- Configuración ----------------
W, H = 820, 616              # resolución de captura
OUT_W, OUT_H = 820, 616         # resolución de procesamiento (sube FPS)
JPEG_QUALITY = 50
BOUNDARY = b"frame"


# ---------------- Cámara ----------------
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"format": "BGR888", "size": (W, H)}
    )
)
picam2.start()
image = picam2.capture_array()


fig, axs = plt.subplots(1, 1, layout="constrained")
axs.imshow(image)
axs.set_axis_off()
descr_puntos = ["superior izquierdo", "inferior izquierdo", "inferior derecho","superior derecho"]
puntos = []
nPunto = 0
def on_click(event):
    global nPunto
    if event.button is MouseButton.LEFT and event.xdata is not None and event.ydata is not None:
        x = round(event.xdata)
        y = round(event.ydata)
        print(f'Punto seleccionado: [{x}, {y}]')

        # Dibujar punto
        axs.plot(x, y, 'or', markersize=8)

        # Dibujar línea con el punto anterior
        if len(puntos) > 0:
            x_prev, y_prev = puntos[-1]
            axs.plot([x_prev, x], [y_prev, y], 'r-', linewidth=2)

        puntos.append([x, y])
        fig.canvas.draw_idle()

        nPunto += 1

        if nPunto == 4:
            # opcional: cerrar figura (último con primero)
            x0, y0 = puntos[0]
            axs.plot([x, x0], [y, y0], 'r-', linewidth=2)
            fig.canvas.draw_idle()

            plt.disconnect(binding_id)
            plt.close()
        else:
            print(f"Introduzca el punto {descr_puntos[nPunto]}...")

binding_id = plt.connect('button_press_event', on_click)
print(f"Introduzca el punto {descr_puntos[nPunto]}...")
plt.show()

print("Puntos leídos:")
src = np.array(puntos)
pprint(src)