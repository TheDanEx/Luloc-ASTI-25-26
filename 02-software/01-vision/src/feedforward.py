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
        threshold = 130   # ajustable
        gray = frame[:, :, 0]
        green = frame[:, :, 1]
        blue = frame[:, :, 0]
        red = frame[:, :, 2]
        mask = (blue < threshold) & (green < threshold) & (red < threshold)
        resultado = np.zeros_like(green)
        resultado[mask] = 255

        resultado  = np.repeat(resultado[:, :, None], 3, axis=2)
    
        img = Image.fromarray(resultado)
        draw = ImageDraw.Draw(img)

        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            size=40   
        )
        
        #puntos obtenidos mediante el script crearPerspectiva
        puntos = [[314, 298],
       [270, 459],
       [568, 464],
       [515, 292]]

        pts1 = np.float32(puntos)
        # pts2 = np.float32([[0,0],[OUT_H,0],[0,OUT_W],[OUT_H,OUT_W]])
        pts2 = np.float32([[0,0],[0,OUT_W],[OUT_H,OUT_W],[OUT_H,0]])
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
        n_partes_image = 17
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
            if cx_i == -1:
                puntos_linea.append((-1, -1))
            else:
                puntos_linea.append((cx_i,cy_i))
                draw2.ellipse(
                    [(cx_i - r, cy_i - r), (cx_i + r, cy_i + r)],
                    fill=(255, 0, 0)   # rojo
                )
            origen_image = fin_parte
            fin_parte+=tam_parte
        
        total_deviation = 0
        total_weight = 0

        for i in range(1, len(puntos_linea)):
            x_last, y_last = puntos_linea[i-1]
            x_current, y_current = puntos_linea[i]
            
            # Peso decreciente para puntos más lejanos al robot (look-ahead)
            # i=1 (lejos) -> weight ~ 0.94, i=16 (cerca) -> weight ~ 0.06
            weight = (len(puntos_linea) - i) / len(puntos_linea)
            total_weight += weight

            if x_last == -1 or x_current == -1:
                # Pérdida de línea: penalización máxima (PI/2)
                total_deviation += (np.pi / 2) * weight
            elif x_current <= 1 or x_current >= OUT_W - 1:
                # Fuera de bordes: penalización máxima
                total_deviation += (np.pi / 2) * weight
            else:
                # Vector entre segmentos
                dx = x_current - x_last
                dy = y_current - y_last
                
                theta = np.arctan2(dy, dx)
                # "Adelante" es pi/2 (vertical hacia abajo en la imagen procesada de arriba a abajo)
                deviation = abs(theta - np.pi / 2)
                total_deviation += deviation * weight

        # Promedio ponderado de la desviación (rango 0 a PI/2)
        avg_deviation = total_deviation / total_weight if total_weight > 0 else (np.pi / 2)

        # Multiplicador 0-2:
        # - Desviación 0 (recta) -> 2.0
        # - Desviación PI/4 (90 grados) -> 1.0
        # - Desviación PI/2 (pérdida/perpendicular) -> 0.0
        speed_multiplier = 2.0 * (1.0 - (avg_deviation / (np.pi / 2)))
        curvatura_feedforward = max(0.0, min(2.0, speed_multiplier))

        mqtt_client.publish(
            MQTT_TOPIC_CURVATURA,
            f"{curvatura_feedforward:.6f}",
            qos=0,
            retain=False,
        )

        text = f"speed_mult= {curvatura_feedforward:.6f}"
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
