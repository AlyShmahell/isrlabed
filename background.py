import rel
import zmq
import zmq.asyncio
import json
import asyncio
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
    def __init__(self):
        context = zmq.asyncio.Context()
        self.socket = context.socket(zmq.PUSH)
        self.socket.bind('tcp://localhost:5555')
        self.loop = asyncio.get_event_loop()
        websocket.enableTrace(True)
        for sensor in [
            'accelerometer',
            'gyroscope',
            'linear_acceleration'
        ]:
            ws = websocket.WebSocketApp(
                f"ws://192.168.1.171:8081/sensor/connect?type=android.sensor.{sensor}",
                on_open=lambda ws: None,
                on_close=lambda ws, *args: print(*args),
                on_error=lambda ws, err: print(err),
                on_message=lambda ws, message, sensor=sensor: self.on_message(ws, message, sensor)
            )
            ws.run_forever(dispatcher=rel, reconnect=5)
        rel.signal(2, rel.abort)
        rel.dispatch()

if __name__=='__main__':
    Pusher()