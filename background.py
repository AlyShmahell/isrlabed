import os
import rel
import zmq
import json
import time
import argparse
import websocket

class Publisher:
    def on_sensor(self, ws, message):
        self.socket.send(str(f"sensor {message}").encode('ascii'))
    def on_gps(self, ws, message):
        print(message)
        self.socket.send(str(f"gps {message}").encode('ascii'))
    def __init__(self, args):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f'tcp://{args.zmq_host}:{args.zmq_port}')
        websocket.enableTrace(True)
        try:
            with open("names.json", "r") as f:
                names = json.load(f)
                assert isinstance(names, list)
        except:
            names = []
        types = f'{names}'.replace(" ", '').replace("'", '"')
        url = f'ws://{args.wss_host}:{args.wss_port}/sensors/connect?types={types}'
        ws = websocket.WebSocketApp(
            url,
            on_open=lambda ws: None,
            on_close=lambda ws, *args: print(*args),
            on_error=lambda ws, err: print(err),
            on_message=lambda ws, message: self.on_sensor(ws, message)
        )
        ws.run_forever(dispatcher=rel, reconnect=5)
        url = f'ws://{args.wss_host}:{args.wss_port}/gps'
        ws = websocket.WebSocketApp(
            url,
            on_open=lambda ws: None,
            on_close=lambda ws, *args: print(*args),
            on_error=lambda ws, err: print(err),
            on_message=lambda ws, message: self.on_gps(ws, message)
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
    Publisher(args)