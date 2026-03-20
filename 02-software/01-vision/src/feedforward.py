import os

from flask import Flask, Response
import numpy as np
import imageio.v3 as iio
import cv2
from skimage.transform import resize,ProjectiveTransform, warp
from picamera2 import Picamera2
from PIL import Image, ImageDraw, ImageFont
import paho.mqtt.client as mqtt

# ---------------- Configuración ----------------
W, H = 820, 616              # resolución de captura
OUT_W, OUT_H = 820, 616         # resolución de procesamiento (sube FPS)
JPEG_QUALITY = 50
BOUNDARY = b"frame"
MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_TOPIC_CURVATURA = os.getenv("MQTT_TOPIC_CURVATURA", "robot/vision/curvature")

app = Flask(__name__)

try:
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="vision_feedforward")
except AttributeError:
    mqtt_client = mqtt.Client(client_id="vision_feedforward")
mqtt_client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=30)
mqtt_client.loop_start()

# ---------------- Cámara ----------------
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"format": "BGR888", "size": (W, H)}
    )
)
picam2.start()

def frames():
    while True:
        frame = picam2.capture_array()  # (H,W,3) BGR uint8

        # ---- Gris rápido (suficiente para visión) ----
        gray = frame[:, :, 0]
        
        # ---- A PARTIR DE AQUÍ: skimage ----
        # Ejemplo:
        # mask = und < 60
        # und = (mask.astype(np.uint8) * 255)
        # Elegimos un umbral para "negro"
        threshold = 60   # ajustable
        # Máscara: Matriz de True y false. Donde es mas pequeño de 30, ponemos true, donde no, false.
        mask = gray < threshold
        # creamos una nueva matriz del mismo tamaño, pero todo a ceros
        resultado = np.zeros_like(gray)
        # en la matriz de ceros, ponemos a 255 los pixeles que estan a True en la mascara
        resultado[mask] = 255

        resultado  = np.repeat(resultado[:, :, None], 3, axis=2)
    
        img = Image.fromarray(resultado)
        draw = ImageDraw.Draw(img)

        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            size=40   
        )
        
        #puntos obtenidos mediante el script crearPerspectiva
        puntos = [[293, 301],
       [532, 281],
       [171, 579],
       [726, 513]]

        pts1 = np.float32(puntos)
        pts2 = np.float32([[0,0],[OUT_H,0],[0,OUT_W],[OUT_H,OUT_W]])

        tform = ProjectiveTransform()
        tform.estimate(pts1, pts2)

        new_img = warp(
            resultado,
            inverse_map=tform.inverse,
            output_shape=(820, 626),
            preserve_range=True
        ).astype(np.uint8)

        final_img = Image.fromarray(new_img)
        draw2 = ImageDraw.Draw(final_img)

        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            size=20   
        )
        
        shape = new_img.shape
        n_partes_image = 10
        tam_parte = shape[0]//n_partes_image
        fin_parte = tam_parte
        origen_image = 0
        r = 4  # radio del punto
        puntos_linea = []
        for i in range(n_partes_image):
            cx, cy = blobDetector(new_img[origen_image:fin_parte,:, 0])
            #print(f'{i} -> [{origen_image} : {fin_parte}]')
            cx_i = int(cx)
            cy_i = int(cy+origen_image)
            puntos_linea.append((cx_i,cy_i))
            draw2.ellipse(
                [(cx_i - r, cy_i - r), (cx_i + r, cy_i + r)],
                fill=(255, 0, 0)   # rojo
            )
            origen_image = fin_parte
            fin_parte+=tam_parte
        
        theta_prev = None
        curvatura = 0

        for i in range(1, len(puntos_linea)):
            x_last, y_last = puntos_linea[i-1]
            x_current, y_current = puntos_linea[i]

            theta = np.arctan2(y_current - y_last,
                            x_current - x_last)

            if theta_prev is not None:
                dtheta = theta - theta_prev

                # normalizar a [-pi, pi]
                dtheta = (dtheta + np.pi) % (2*np.pi) - np.pi

                weight = len(puntos_linea) - i
                curvatura += abs(dtheta) * weight

            theta_prev = theta

        curvatura_feedforward = curvatura / len(puntos_linea)

        mqtt_client.publish(
            MQTT_TOPIC_CURVATURA,
            f"{curvatura_feedforward:.6f}",
            qos=0,
            retain=False,
        )

        text = f"curvatura= {curvatura_feedforward:.6f}"
        draw2.text((20, 50), text, fill=(255, 0, 0),font=font)


        jpg = iio.imwrite("<bytes>", final_img, extension=".jpg", quality=JPEG_QUALITY)

        yield (
            b"--" + BOUNDARY + b"\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" +
            jpg + b"\r\n"
        )

def blobDetector(mask):
    mask = mask.astype(np.uint8)

    m00 = mask.sum()
    if m00 == 0:
        return -1, -1

    ys, xs = np.indices(mask.shape)

    m10 = (xs * mask).sum()
    m01 = (ys * mask).sum()

    cx = m10 / m00
    cy = m01 / m00

    return cx, cy

@app.get("/")
def index():
    return "<img src='/stream' style='max-width:100%;height:auto'/>"

@app.get("/stream")
def stream():
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
