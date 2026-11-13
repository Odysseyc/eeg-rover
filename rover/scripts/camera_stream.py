#!/usr/bin/env python3
"""Live view of the rover's camera in a web browser.

Run on the Jetson, then open  http://<jetson-ip>:8081  on the laptop (any browser).

  USB camera:                 /usr/bin/python3 camera_stream.py
  Ribbon (CSI) camera:        /usr/bin/python3 camera_stream.py --csi
  Pick a USB device:          /usr/bin/python3 camera_stream.py --device 2
  Test without a camera:      /usr/bin/python3 camera_stream.py --test

Uses /usr/bin/python3 because JetPack's OpenCV (with GStreamer, needed for --csi) is installed there.
Streams MJPEG over HTTP: simple and works everywhere; expect roughly 0.1-0.3 s of delay on Wi-Fi.
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PAGE = b"""<!doctype html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover camera</title><style>
html,body{margin:0;height:100%;background:#111;color:#ddd;font-family:system-ui,sans-serif}
body{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px}
img{max-width:100vw;max-height:92vh;object-fit:contain}
</style></head><body><img src="/stream" alt="Rover camera"><div>Rover camera</div></body></html>"""


class Camera:
    """Grabs frames in a background thread and keeps the latest JPEG."""

    def __init__(self, args):
        self.args, self.jpeg, self.lock = args, None, threading.Lock()
        self.cap = None if args.test else self.open()
        threading.Thread(target=self.loop, daemon=True).start()

    def open(self):
        a = self.args
        if a.csi:
            pipeline = (f'nvarguscamerasrc sensor-id={a.sensor_id} ! '
                        f'video/x-raw(memory:NVMM),width={a.width},height={a.height},framerate={a.fps}/1 ! '
                        f'nvvidconv flip-method={a.flip} ! video/x-raw,format=BGRx ! videoconvert ! '
                        f'video/x-raw,format=BGR ! appsink drop=true max-buffers=1')
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
            cap.set(cv2.CAP_PROP_FPS, a.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            raise SystemExit('Could not open the camera. Check `ls /dev/video*`; for a ribbon camera use --csi.')
        return cap

    def loop(self):
        params = [cv2.IMWRITE_JPEG_QUALITY, self.args.quality]
        t0 = time.monotonic()
        while True:
            if self.cap is None:  # --test: moving color bars so you can check the page and delay
                t = time.monotonic() - t0
                frame = np.zeros((self.args.height, self.args.width, 3), np.uint8)
                frame[:, :, 0] = (np.arange(self.args.width) * 255 // self.args.width)[None, :]
                x = int((t * 200) % self.args.width)
                frame[:, max(0, x - 10):x + 10] = 255
                cv2.putText(frame, f'{t:7.2f} s', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
                time.sleep(1.0 / self.args.fps)
                ok = True
            else:
                ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode('.jpg', frame, params)
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()

    def latest(self):
        with self.lock:
            return self.jpeg


def make_handler(cam, fps):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # keep the terminal quiet
            pass

        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                last = None
                try:
                    while True:
                        jpg = cam.latest()
                        if jpg is not None and jpg is not last:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                             + f'Content-Length: {len(jpg)}\r\n\r\n'.encode() + jpg + b'\r\n')
                            last = jpg
                        time.sleep(0.5 / fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # browser tab closed
            else:
                self.send_error(404)
    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8081)
    ap.add_argument('--device', type=int, default=0, help='USB camera index (/dev/videoN)')
    ap.add_argument('--csi', action='store_true', help='ribbon camera on the Jetson CAM connector')
    ap.add_argument('--sensor-id', type=int, default=0, help='CSI connector: 0 = CAM0, 1 = CAM1')
    ap.add_argument('--flip', type=int, default=0, help='CSI only: 2 = rotate 180 degrees')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--quality', type=int, default=70, help='JPEG quality; lower = less Wi-Fi load')
    ap.add_argument('--test', action='store_true', help='no camera: stream a moving test pattern')
    args = ap.parse_args()

    cam = Camera(args)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(cam, args.fps))
    print(f'Streaming on http://<this-jetson-ip>:{args.port}  (Ctrl+C to stop)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
