import threading
from flask import Flask, Response
import imageio.v3 as iio
from picamera2 import Picamera2

# ---------------- Configuración ----------------
W, H = 820, 616
JPEG_QUALITY = 50
BOUNDARY = b"frame"

# ---------------- Flask ----------------
app = Flask(__name__)

# ---------------- Cámara ----------------
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"format": "BGR888", "size": (W, H)}
    )
)
picam2.start()

camera_lock = threading.Lock()


def capture_jpeg():
    """
    Captura una imagen de la cámara y la devuelve en JPEG.
    La cámara entrega BGR, así que convertimos a RGB.
    """
    with camera_lock:
        frame_bgr = picam2.capture_array()

  
    jpg = iio.imwrite("<bytes>", frame_bgr, extension=".jpg", quality=JPEG_QUALITY)
    return jpg


def frames():
    """
    Generador MJPEG para navegador.
    """
    while True:
        jpg = capture_jpeg()
        yield (
            b"--" + BOUNDARY + b"\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" +
            jpg + b"\r\n"
        )


@app.get("/")
def index():
    return """
    <html>
      <body style="margin:0;background:#111;color:white;font-family:sans-serif">
        <div style="padding:10px">
          <h3>Camera server</h3>
          <p><a href="/snapshot" style="color:#7cf">Snapshot</a></p>
          <p><a href="/stream" style="color:#7cf">MJPEG stream</a></p>
        </div>
        <img src="/stream" style="max-width:100%;height:auto;display:block;margin:auto"/>
      </body>
    </html>
    """


@app.get("/snapshot")
def snapshot():
    """
    Endpoint para Unity: devuelve una imagen JPG normal.
    """
    jpg = capture_jpeg()
    return Response(jpg, mimetype="image/jpeg")


@app.get("/stream")
def stream():
    """
    Endpoint MJPEG para navegador.
    """
    return Response(
        frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8080, threaded=True)
    finally:
        picam2.stop()