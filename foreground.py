import os
import json
import zmq
import zmq.asyncio
import logging 
import argparse
from datetime import datetime
from nicegui import app, ui, Client, background_tasks
logger = logging.getLogger("nicegui")

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]
    
class ZMQ(metaclass=Singleton):
    def __init__(self, host='', port=''):
        context = zmq.asyncio.Context()
        self.socket = context.socket(zmq.PULL)
        self.socket.connect(f'tcp://{host}:{port}')
        self.poller = zmq.asyncio.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

@ui.page("/")
def index(client: Client):
    class Main:
        @property
        def connected(self):
            return 'wifi' if bool(self.events) else 'wifi_off'
        def __init__(self):
            self.zmqcon = ZMQ()
            with ui.header(elevated=True).classes("justify-between bg-black"):
                ui.label("NiceGUI.4.Bots")
                def change_status_icon(name):
                    if name == 'wifi':
                        statuc_icon.style("color: green")
                    else:
                        statuc_icon.style("color: red")
                    return name
                statuc_icon = ui.icon('wifi_off', size='xl').bind_name_from(self, 'connected', change_status_icon)

            with ui.grid(columns=2, rows=2).classes("h-full w-full"):
                with ui.card().classes("h-full w-full"):

                    self.line_plot = ui.line_plot(n=3, limit=100, figsize=(10, 4))
                with ui.card().classes("h-full w-full"):
                    pass
                with ui.card().classes("h-full w-full"):
                    pass
                with ui.card().classes("h-full w-full"):
                    pass
            with ui.footer(elevated=True).classes("w-full justify-center bg-black"):
                ui.label("Copyrights 2024 © Aly Shmahell")
            client.on_connect(background_tasks.create(self()))
        async def __call__(self):
            while not app.is_stopped:
                self.events = await self.zmqcon.poller.poll()
                if self.zmqcon.socket in dict(self.events):
                    try:
                        data = json.loads(await self.zmqcon.socket.recv())
                        assert isinstance(data, dict)
                    except Exception as e:
                        logger.exception(e)
                        continue
                    match data.get("name"):
                        case "linear_acceleration":
                            values = data.get("values", [])
                            if values:
                                self.line_plot.push(
                                    [datetime.now()], 
                                    [[value] for value in values]
                                )
    Main()

if __name__ in ['__main__', '__mp_main__']:
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--gui-port", default=int(os.environ.get("gui-port", 8080)), type=int)
    argparser.add_argument("--zmq-port", default=int(os.environ.get("zmq-port", 5555)), type=int)
    argparser.add_argument("--gui-host", default=str(os.environ.get("gui-host", '0.0.0.0')), type=str)
    argparser.add_argument("--zmq-host", default=str(os.environ.get("zmq-host", '0.0.0.0')), type=str)
    args = argparser.parse_args()
    ZMQ(
        host=args.zmq_host,
        port=args.zmq_port
    )
    ui.run(
        host=args.gui_host,
        port=args.gui_port,
        title="NiceGUI.4.Bots",
        favicon="🤖"
    )