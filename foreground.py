import os
import json
import zmq
import zmq.asyncio
import logging 
import argparse
from datetime import datetime
from nicegui import app, ui, Client, background_tasks
logger = logging.getLogger("nicegui")
import plotly.graph_objects as go
import numpy as np
from scipy.spatial.transform import Rotation as R

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
        @ui.refreshable
        def orientation(self, degree):
            # Define a rotation for demonstration (e.g., 45 degrees around the Z-axis)
            rotation = R.from_euler('z', degree, degrees=True)

            # Standard unit vectors
            i_hat = np.array([1, 0, 0])  # X-axis
            j_hat = np.array([0, 1, 0])  # Y-axis
            k_hat = np.array([0, 0, 1])  # Z-axis

            # Rotate the unit vectors to get the new orientation
            u_x = rotation.apply(i_hat)  # New X-axis after rotation
            u_y = rotation.apply(j_hat)  # New Y-axis after rotation
            u_z = rotation.apply(k_hat)  # New Z-axis after rotation

            # Define vectors in dictionary for easier plotting
            vectors = {
                "X-axis": {"start": [0, 0, 0], "end": u_x},
                "Y-axis": {"start": [0, 0, 0], "end": u_y},
                "Z-axis": {"start": [0, 0, 0], "end": u_z},
            }

            # Create the 3D plot
            fig = go.Figure()

            # Add each oriented unit vector as an arrow (quiver plot)
            for name, vector in vectors.items():
                fig.add_trace(
                    go.Scatter3d(
                        x=[vector["start"][0], vector["end"][0]],
                        y=[vector["start"][1], vector["end"][1]],
                        z=[vector["start"][2], vector["end"][2]],
                        marker=dict(size=2),
                        line=dict(width=5),
                        name=name
                    )
                )
            fig.update_traces()
            # Customize layout for visualization
            fig.update_layout(
                scene=dict(
                    xaxis=dict(nticks=4, range=[-1, 1.5], title="X-axis"),
                    yaxis=dict(nticks=4, range=[-1, 1.5], title="Y-axis"),
                    zaxis=dict(nticks=4, range=[-1, 1.5], title="Z-axis"),
                    aspectratio=dict(x=1, y=1, z=1),
                ),
                title="Orientation"
            )

            ui.plotly(fig).classes("h-full w-full")
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
            try:
                with open("names.json", "r") as f:
                    names = json.load(f)
                    assert isinstance(names, list)
            except:
                names = []
            with ui.grid(columns=2, rows=2).classes("h-full w-full") as self.context:
                with ui.card().classes("h-full w-full"):
                    self.line_select = ui.select(
                        options=names,
                        label='sensors'
                    )
                    self.line_plot = ui.line_plot(n=3, limit=100, figsize=(10, 4))
                with ui.card().classes("h-full w-full"):
                    self.orientation(0)
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
                        case self.line_select.value:
                            values = data.get("values", [])
                            if len(values) == 3:
                                self.line_plot.push(
                                    [datetime.now()], 
                                    [[value] for value in values]
                                )
                            else:
                                with self.context:
                                    ui.notify(f"cannot plot {len(values)} lines", type='negative')
                        case 'orientation':
                            values = data.get("values", [])
                            self.orientation.refresh(np.linalg.norm(values))
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