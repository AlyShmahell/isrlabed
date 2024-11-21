import os
import rel
import zmq
import json
import requests
import argparse
import websocket

class Publisher:
    def send(self, topic, message):
        if topic == 'sensor':
            topic = json.loads(message)['type']
        self.socket.send(str(f"{topic} {message}").encode('ascii'))
    def __init__(self, args):
        with requests.Session() as session:
            with session.get(f"http://{args.http_host}:{args.http_port}/sensors") as resp:
                try:
                    if not resp.ok:
                        raise Exception(resp.status_code)
                    sensors = json.loads(resp.content)
                    assert isinstance(sensors, list)
                    types = [sensor["type"] for sensor in sensors]
                    types = f'{types}'.replace(" ", '').replace("'", '"')
                    print(types)
                except Exception as e:
                    exit(e)
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(f'tcp://{args.zmq_host}:{args.zmq_port}')
        websocket.enableTrace(True)
        url = f'ws://{args.wss_host}:{args.wss_port}/sensors/connect?types={types}'
        ws = websocket.WebSocketApp(
            url,
            on_open=lambda ws: None,
            on_close=lambda ws, *args: print(*args),
            on_error=lambda ws, err: print(err),
            on_message=lambda ws, message: self.send('sensor', message)
        )
        ws.run_forever(dispatcher=rel, reconnect=5)
        url = f'ws://{args.wss_host}:{args.wss_port}/gps'
        ws = websocket.WebSocketApp(
            url,
            on_open=lambda ws: None,
            on_close=lambda ws, *args: print(*args),
            on_error=lambda ws, err: print(err),
            on_message=lambda ws, message: self.send('gps', message)
        )
        ws.run_forever(dispatcher=rel, reconnect=5)
        rel.signal(2, rel.abort)
        rel.dispatch()
        
if __name__ in ['__main__', '__mp_main__']:
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--zmq-port", default=int(os.environ.get("zmq-port", 5555)), type=int)
    argparser.add_argument("--zmq-host", default=str(os.environ.get("zmq-host", '0.0.0.0')), type=str)
    argparser.add_argument("--wss-port", default=int(os.environ.get("wss-port", 8082)), type=int)
    argparser.add_argument("--wss-host", default=str(os.environ.get("wss-host", '192.168.1.171')), type=str)
    argparser.add_argument("--http-port", default=int(os.environ.get("http-port", 9090)), type=int)
    argparser.add_argument("--http-host", default=str(os.environ.get("http-host", '192.168.1.171')), type=str)
    args = argparser.parse_args()
    Publisher(args)