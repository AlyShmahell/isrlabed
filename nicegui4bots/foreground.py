import os
import json
import zmq
import zmq.asyncio
import asyncio
import logging 
import argparse
import datetime
from   nicegui import app, ui, Client, background_tasks
from nicegui.helpers import is_coroutine_function
logger = logging.getLogger("nicegui")
logger.setLevel(logging.DEBUG)
from utils import Singleton

class Args(metaclass=Singleton):
    def __new__(self):
        argparser = argparse.ArgumentParser()
        argparser.add_argument("--gui-port", default=int(os.environ.get("gui-port", 8080)), type=int)
        argparser.add_argument("--gui-host", default=str(os.environ.get("gui-host", '0.0.0.0')), type=str)
        argparser.add_argument("--zmq-port", default=int(os.environ.get("zmq-port", 5555)), type=int)
        argparser.add_argument("--zmq-host", default=str(os.environ.get("zmq-host", '0.0.0.0')), type=str)
        args = argparser.parse_args()
        print(f"{__name__} args: {args}")
        return args
    
class CompoundMap(ui.element):
    def __init__(self, tag = None, *, _client = None, center):
        super().__init__(tag, _client=_client)
        with self:
            self.map = ui.leaflet(center).classes("w-full h-full")
            self.marker = self.map.marker(latlng=center)
    def move(self, center):
        self.map.set_center(center)
        self.marker.move(*center)


@ui.page("/")
def index(client: Client):
    class Main:
        @property
        def connected(self):
            return 'wifi' if bool(self.events) else 'wifi_off'
        def switch(self, topic):
            self.body.clear()
            with self.body:
                with ui.card().classes("w-full h-full justify-center items-center"):
                    ui.label(topic)
                    if topic == "gps":
                        self.elem = CompoundMap(center=(0,0)).classes("w-full h-full")
                    else:
                        if self.options[topic]['len'] == 3:
                            self.elem = ui.line_plot(n=3, limit=100, figsize=(10, 4))
                        else:
                            self.elem = ui.highchart({
                                        'title': False,
                                        'chart': {'type': 'bar'},
                                        'series': [
                                            {'name': topic, 'data': []},
                                        ],
                                    }, extras=['solid-gauge'])
                            if self.options[topic]['len'] == 1:
                                self.elem.options['chart']['type'] = 'solidgauge'
                            ui.notify(self.elem.options)
        async def process(self, topic, data):
            if topic not in self.selection.options:
                self.selection.options += [topic]
                self.selection.update()
            if topic == "gps":
                self.options[topic] = {
                    'center': (int(data.get("latitude", 0)), int(data.get("longitude", 0)))
                }
            else:
                values = data.get("values", [])
                self.options[topic] = {
                    'values': values,
                    'len': len(values),
                    'min': min(min(values), self.options.get(topic, {}).get('min', min(values))),
                    'max': max(max(values), self.options.get(topic, {}).get('max', max(values)))
                }
            if topic != self.selection.value:
                return
            with self.body:
                if topic == "gps":
                    self.elem.move(self.options[topic]['center'])
                else:
                    if self.options[topic]['len'] == 3:
                        print(topic, self.options[topic]['values'])
                        self.elem.push(
                                [datetime.datetime.now()], 
                                [[value] for value in self.options[topic]['values']]
                            )
                    else:
                        self.elem.options['series'] = [{
                            "name": topic,
                            "data": self.options[topic]['values']
                        }]
                        self.elem.options['yAxis'] = {
                            'min': self.options[topic]['min'],
                            'max': self.options[topic]['max']
                        }                    
                        self.elem.update()
        def __init__(self):
            args = Args()
            self.elem = None
            self.options = {}
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
            with ui.element("div").classes("h-[80vh] w-full"):
                self.selection = ui.select(
                        options=[],
                        label='sensors',
                        on_change=lambda e: self.switch(e.value)
                    ).classes("w-full")
                self.body = ui.element("div").classes("h-full w-full")
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
                        await self.process(topic, data)
                    except Exception as e:
                        logger.exception(e)
                        continue

    Main()

if __name__ in ['__main__', '__mp_main__']:
    args = Args()
    ui.run(
        host=args.gui_host,
        port=args.gui_port,
        title="NiceGUI.4.Bots",
        favicon="🤖"
    )