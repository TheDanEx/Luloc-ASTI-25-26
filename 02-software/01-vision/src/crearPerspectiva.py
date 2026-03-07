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

######################################
# Selección de puntos con el ratón   #
# No nos interesa. Se puede ignorar  #
######################################
fig, axs = plt.subplots(1, 1, layout="constrained")
axs.imshow(image)
axs.set_axis_off()
descr_puntos = ["superior izquierdo", "superior derecho", "inferior izquierdo", "inferior derecho"]
puntos = []
nPunto = 0
def on_click(event):
    global nPunto
    if event.button is MouseButton.LEFT:
        x = round(event.xdata)
        y = round(event.ydata)
        print(f'Punto seleccionado: [{x}, {y}]')
        axs.plot(x, y, '.r')
        puntos.append([x,y])
        plt.show()
        nPunto += 1
        if nPunto==4:
            plt.disconnect(binding_id)
            plt.close()
        else:
            print(f"Introduzca el punto {descr_puntos[nPunto]}...")

binding_id = plt.connect('button_press_event', on_click)
print(f"Introduzca el punto {descr_puntos[nPunto]}...")
plt.show()
######################################

print("Puntos leídos:")
src = np.array(puntos)
pprint(src)

