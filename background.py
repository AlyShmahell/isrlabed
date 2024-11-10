import os
import rel
import zmq
import zmq.asyncio
import json
import asyncio
import argparse
import websocket

class Pusher:
    def on_message(self, ws, message, sensor):
        message = json.loads(message)
        message['name'] = sensor
        message = json.dumps(message)
        async def send(message):
            await self.socket.send(str(message).encode('ascii'))
            await asyncio.sleep(0.01)
        async def create_tasks():
            await asyncio.wait(
                [asyncio.create_task(send(message))]
            )
            print(sensor, message)
        self.loop.run_until_complete(create_tasks())
    def __init__(self, args):
        context = zmq.asyncio.Context()
        self.socket = context.socket(zmq.PUSH)
        self.socket.bind(f'tcp://{args.zmq_host}:{args.zmq_port}')
        self.loop = asyncio.get_event_loop()
        websocket.enableTrace(True)
        try:
            with open("names.json", "r") as f:
                names = json.load(f)
                assert isinstance(names, list)
        except:
            names = []
        for sensor in names:
            ws = websocket.WebSocketApp(
                f"ws://{args.wss_host}:{args.wss_port}/sensor/connect?type=android.sensor.{sensor}",
                on_open=lambda ws: None,
                on_close=lambda ws, *args: print(*args),
                on_error=lambda ws, err: print(err),
                on_message=lambda ws, message, sensor=sensor: self.on_message(ws, message, sensor)
            )
            ws.run_forever(dispatcher=rel, reconnect=5)
        rel.signal(2, rel.abort)
        rel.dispatch()

if __name__ in ['__main__', '__mp_main__']:
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--wss-port", default=int(os.environ.get("wss-port", 8081)), type=int)
    argparser.add_argument("--zmq-port", default=int(os.environ.get("zmq-port", 5555)), type=int)
    argparser.add_argument("--wss-host", default=str(os.environ.get("wss-host", '192.168.1.171')), type=str)
    argparser.add_argument("--zmq-host", default=str(os.environ.get("zmq-host", '0.0.0.0')), type=str)
    args = argparser.parse_args()
    Pusher(args)