import os 
import cv2 
import time
import json
from tempfile import NamedTemporaryFile
from ultralytics import YOLO
from fastapi import Response, Request
from nicegui import app, run, ui
import numpy as np
from PIL import Image
import base64
from functools import cache


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]
    
    
def convert(frame: np.ndarray) -> bytes:
    return Image.fromarray(frame)


class ImageSeg(metaclass=Singleton):
    def __init__(self):
        self.model = YOLO("yolo11n-seg.pt")
        self.cam   = cv2.VideoCapture(2)
        black_1px = 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAAAXNSR0IArs4c6QAAAA1JREFUGFdjYGBg+A8AAQQBAHAgZQsAAAAASUVORK5CYII='
        self.placeholder = base64.b64decode(black_1px.encode('ascii'))
    @cache
    async def __call__(self, t):
        if not self.cam.isOpened():
            return {}, self.placeholder
        ret, frame = await run.io_bound(self.cam.read)
        results = self.model(frame)
        result  = results[0].plot()
        jpeg = await run.cpu_bound(convert, result)
        return {
            results[0].names[idx]: segment.shape[0]
            for segment, idx in zip(
                results[0].masks.xyn, 
                [int(box.cls.cpu().numpy()[0]) for box in results[0].boxes]
            )
        }, jpeg
    def __del__(self):
        self.cam.release()
        cv2.destroyAllWindows()

@app.get('/video/frame')
async def grab_video_frame(request: Request, t=None) -> Response:
    _, jpeg = await AI()(t)
    print(t)
    return Response(content=jpeg, media_type='image/jpeg')



@ui.page("/")
def index():
    class Main:
        @ui.refreshable
        def chart(self, data=[]):
            ui.highchart({
                        'title': False,
                        'chart': {'type': 'solidgauge'},
                        'yAxis': {
                            'min': 0,
                            'max': 1,
                        },
                        'series': [
                            {'data': data},
                        ],
                    }, extras=['solid-gauge']).classes('w-full h-64')
        async def loop(self):
            info, jpeg = await AI()(time.time())
            self.video.set_source(jpeg)
            self.chart.refresh([*info.values()])
        def __init__(self):
            with ui.header(elevated=True):
                ui.label("NiceGUI.4.Bots")
            with ui.grid(columns=2, rows=2).classes("h-full w-full"):
                with ui.card().classes("h-full w-full"):
                    self.video = ui.interactive_image().classes('w-full h-full')
                with ui.card().classes("h-full w-full"):
                    self.chart()
                with ui.card().classes("h-full w-full"):
                    pass
                with ui.card().classes("h-full w-full"):
                    pass
            ui.timer(interval=0.1, callback=self.loop)
    Main()

ui.run(
    host="0.0.0.0",
    port=os.environ.get('port', 8080),
    title="NiceGUI.4.Bots",
    favicon="🤖"
)
