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
    
async def refresh_maybe(refreshable: ui.refreshable, container:ui.element, *args, **kwargs):
    new = True
    for target in refreshable.targets:
        if target.instance == refreshable.instance:
            new = False
    with container:
        container.clear()
        if new:
            if is_coroutine_function(refreshable):
                await refreshable(*args, **kwargs)
            else:
                refreshable(*args, **kwargs)
        else:
            if is_coroutine_function(refreshable):
                await refreshable.refresh(*args, **kwargs)
            else:
                refreshable.refresh(*args, **kwargs)

@ui.page("/")
def index(client: Client):
    class Main:
        @property
        def connected(self):
            return 'wifi' if bool(self.events) else 'wifi_off'
        async def update(self, topic, data):
            if topic == "gps":
                center = (int(data.get("latitude", 0)), int(data.get("longitude", 0)))
                self.elem['map'].set_center(center)
                self.elem['marker'].move(*center)
            else:
                values = data.get("values", [])
                if topic in self.minmax:
                    tmpmin = self.minmax[topic]['min']
                    tmpmax = self.minmax[topic]['max']
                else:
                    tmpmin = 0
                    tmpmax = 100
                self.minmax[topic] = {
                    'min': min(min(values), tmpmin),
                    'max': max(max(values), tmpmax)
                }
                if len(values) == 3:
                    self.elem.push(
                            [datetime.datetime.now()], 
                            [[value] for value in values]
                        )
                else:
                    self.elem.options['series'] = [{
                        "name": topic,
                        "data": values
                    }]
                    self.elem.options['yAxis'] = {
                        'min': self.minmax[topic]['min'],
                        'max': self.minmax[topic]['max']
                    }
                    self.elem.update()
        @ui.refreshable
        def draw(self, topic, data):
            with self.body:
                ui.notify(topic)
                with ui.card():
                    ui.label(topic)
                    if topic == "gps":
                        self.elem = {
                            'map': ui.leaflet(center=(0,0)),
                        }
                        self.elem['marker'] = self.elem['map'].marker(latlng=self.elem['map'].center)
                    else:
                        values = data.get("values", [])
                        if len(values) == 3:
                            self.elem = ui.line_plot(n=3, limit=100, figsize=(10, 4))
                        else:
                            self.elem = ui.highchart({
                                        'title': False,
                                        'chart': {'type': 'bar'},
                                        'series': [
                                            {'name': topic, 'data': []},
                                        ],
                                    }, extras=['solid-gauge'])
                            if len(values) == 1:
                                self.elem.options['chart']['type'] = 'solidgaugue'
        async def draw_or_update(self, topic, data):
            if topic not in self.selection.options:
                self.selection.options += [topic]
                self.selection.update()
            if topic != self.selection.value:
                return
            if topic == self.selected:
                await self.update(topic, data)
            else:
                await refresh_maybe(self.draw, self.body, topic, data)
                await asyncio.sleep(.1)
                self.selected = topic
                await asyncio.sleep(.1)
                await self.update(topic, data)
        def __init__(self):
            args = Args()
            self.selected = ""
            self.elem = None
            self.minmax = {}
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
            self.selection = ui.select(
                    options=[],
                    label='sensors'
                ).classes("w-full")
            self.body = ui.grid(rows=2).classes("h-full w-full")
                
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
                        await self.draw_or_update(topic, data)
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