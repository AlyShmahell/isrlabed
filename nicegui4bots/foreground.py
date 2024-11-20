import os
import json
import zmq
import zmq.asyncio
import asyncio
import logging 
import argparse
import datetime
from   nicegui import app, ui, Client, background_tasks
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
            self.map = ui.leaflet(center).classes("w-full h-[60vh]")
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
        def __init__(self):
            args = Args()
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
            with ui.element("div").classes("h-full w-full"):
                self.tabs       = ui.tabs()
                self.topic2tab  = {}
                self.elems      = {}
                self.tab_panels =  ui.tab_panels(self.tabs).classes('w-full')
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
                        values = data.get("values", [])
                        if topic not in self.topic2tab:
                            with self.tabs:
                                self.topic2tab[topic] = ui.tab(topic)
                            with self.tab_panels:
                                with ui.tab_panel(self.topic2tab[topic]):
                                    with ui.card().classes("w-full h-full justify-center items-center"):
                                        ui.label(topic)
                                        if topic == "gps":
                                            self.elems[topic] = CompoundMap(center=(0,0)).classes("w-full h-full")
                                        else:
                                            if len(values) == 3:
                                                self.elems[topic] = ui.line_plot(n=3, limit=100, figsize=(10, 4))
                                            else:
                                                self.elems[topic] = ui.highchart({
                                                            'title': False,
                                                            'chart': {'type': 'bar'},
                                                            'series': [
                                                                {'name': topic, 'data': []},
                                                            ],
                                                        }, extras=['solid-gauge'])
                                                if len(values) == 1:
                                                    self.elems[topic].options['chart']['type'] = 'solidgauge'
                                                ui.notify(self.elems[topic].options) 
                        if topic != self.tabs.value:
                            continue
                        if topic == "gps":
                            self.elems[topic].move((int(data.get("latitude", 0)), int(data.get("longitude", 0))))
                        else:
                            if len(values) == 3:
                                self.elems[topic].push(
                                        [datetime.datetime.now()], 
                                        [[value] for value in values]
                                    )
                            else:
                                if not any(x['name']==topic for x in self.elems[topic].options['series']):
                                    self.elems[topic].options['series'] += [{
                                        "name": topic,
                                        "data": values
                                    }]
                                else:
                                    for idx, x in enumerate(self.elems[topic].options['series']):
                                        if x['name']==topic:
                                            self.elems[topic].options['series'][idx]['data'] = values
                                self.elems[topic].options['yAxis'] = {
                                    'min': min(min(values), self.elems[topic].options.get('yAxis', {}).get('min', min(values))),
                                    'max': max(max(values), self.elems[topic].options.get('yAxis', {}).get('max', max(values)))
                                }                    
                                self.elems[topic].update()
                                await asyncio.sleep(.1)
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