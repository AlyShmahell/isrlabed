import os
import json
import asyncio
import zmq
import zmq.asyncio
import logging 
import argparse
from datetime import datetime
from nicegui import app, ui, Client, background_tasks
logger = logging.getLogger("nicegui")
logger.setLevel(logging.DEBUG)
import plotly.graph_objects as go
import numpy as np
from scipy.spatial.transform import Rotation as R

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]
    
class Args(metaclass=Singleton):
    def __new__(self):
        argparser = argparse.ArgumentParser()
        argparser.add_argument("--gui-port", default=int(os.environ.get("gui-port", 8080)), type=int)
        argparser.add_argument("--zmq-port", default=int(os.environ.get("zmq-port", 5555)), type=int)
        argparser.add_argument("--gui-host", default=str(os.environ.get("gui-host", '0.0.0.0')), type=str)
        argparser.add_argument("--zmq-host", default=str(os.environ.get("zmq-host", '0.0.0.0')), type=str)
        args = argparser.parse_args()
        print(f"args: {args}")
        return args


@ui.page("/")
def index(client: Client):
    class Main:
        @property
        def connected(self):
            return 'wifi' if bool(self.events) else 'wifi_off'
        def __init__(self):
            args = Args()
            self.sensors = {}
            context = zmq.asyncio.Context()
            self.socket = context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.connect(f'tcp://{args.zmq_host}:{args.zmq_port}')
            self.socket.subscribe(b'')
            self.poller = zmq.asyncio.Poller()
            self.poller.register(self.socket, zmq.POLLIN)
            with ui.header(elevated=True).classes("justify-between bg-black"):
                ui.label("NiceGUI.4.Bots")
                def change_status_icon(name):
                    if name == 'wifi':
                        statuc_icon.style("color: green")
                    else:
                        statuc_icon.style("color: red")
                    return name
                statuc_icon = ui.icon('wifi_off', size='xl').bind_name_from(self, 'connected', change_status_icon)
            with ui.grid(columns=2, rows=2).classes("h-full w-full") as self.context:
                with ui.card().classes("h-full w-full"):
                    self.line_select = ui.select(
                        options=[
                            "android.sensor.accelerometer",
                            "android.sensor.orientation",
                            "android.sensor.gyroscope",
                            "android.sensor.gravity",
                            "android.sensor.linear_acceleration"
                        ],
                        label='sensors'
                    )
                    self.line_plot = ui.line_plot(n=3, limit=100, figsize=(10, 4))
                with ui.card().classes("h-full w-full"):
                    with ui.grid(columns=2, rows=2):
                        for name, chartype, minmax in zip(
                            [
                                "android.sensor.light",
                                "android.sensor.game_rotation_vector",
                                "android.sensor.geomagnetic_rotation_vector",
                                "android.sensor.rotation_vector",
                                "android.sensor.magnetic_field"
                            ],
                            [
                                "solidgauge",
                                "bar",
                                "bar",
                                "bar",
                                "bar",
                            ],
                            [
                                [0,20],
                                [0,.01],
                                [0,1],
                                [0,1],
                                [0,1],
                                [-90, 90]
                            ]
                        ):
                            self.sensors[name] = ui.highchart({
                                'title': False,
                                'chart': {'type': chartype},
                                'yAxis': {
                                    'min': minmax[0],
                                    'max': minmax[1],
                                },
                                'series': [
                                    {'name': name, 'data': []},
                                ],
                            }, extras=['solid-gauge']).classes('w-full')
                with ui.card().classes("h-full w-full"):
                    self.map = ui.leaflet(center=(0,0))
                    self.marker = self.map.marker(latlng=self.map.center)
                with ui.card().classes("h-full w-full"):
                    pass
            with ui.footer(elevated=True).classes("w-full justify-center bg-black"):
                ui.label("Copyrights 2024 © Aly Shmahell")
            client.on_connect(background_tasks.create(self()))
        async def __call__(self):
            while not app.is_stopped:
                self.events = await self.poller.poll()
                if self.socket in dict(self.events):
                    try:
                        data = await self.socket.recv()
                        topic, data = data.decode().split(" ")
                        data = json.loads(data)
                        assert isinstance(data, dict)
                    except Exception as e:
                        logger.exception(e)
                        continue
                    if topic == "gps":
                        center = (int(data.get("latitude", 0)), int(data.get("longitude", 0)))
                        self.map.set_center(center)
                        self.marker.move(*center)
                        continue
                    values = data.get("values", [])
                    name   = data.get("type") 
                    if  name == self.line_select.value and len(values) == 3:
                        self.line_plot.push(
                            [datetime.now()], 
                            [[value] for value in values]
                        )
                    if name in [
                        "android.sensor.light",
                        "android.sensor.game_rotation_vector",
                        "android.sensor.geomagnetic_rotation_vector",
                        "android.sensor.rotation_vector",
                        "android.sensor.magnetic_field"
                    ]:
                        await self.update_charts(name, values)
        async def update_charts(self, name, values):
            if not any(x['name']==name for x in self.sensors[name].options['series']):
                self.sensors[name].options['series'] += [
                    {
                        'name': name,
                        'data': values
                    }
                ]
            else:
                for idx, x in enumerate(self.sensors[name].options['series']):
                    if x['name']==name:
                        self.sensors[name].options['series'][idx]['data'] = values
            self.sensors[name].update()
            await asyncio.sleep(.01)

    Main()

if __name__ in ['__main__', '__mp_main__']:
    args = Args()
    ui.run(
        host=args.gui_host,
        port=args.gui_port,
        title="NiceGUI.4.Bots",
        favicon="🤖"
    )