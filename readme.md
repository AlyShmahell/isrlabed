# NiceGUI4bots

A 100% Python graphical user interface to visualize real-time data streams from a generic WebSocket & HTTP source.

## Demonstrator

Make your smartphone as the WebSocket & HTTP source with th opensource [SensorServer](https://f-droid.org/packages/github.umer0586.sensorserver) App by [umer0586](https://github.com/umer0586) available from F-Droid app store:

     https://f-droid.org/packages/github.umer0586.sensorserver

in the hamburger tab activate the http server and go to settings then activate the discoverable option, also activate the option to use the hotspot if you are on a hotspot.
## Installation
```sh
pip install -r requirements.txt
```

## Running
```
python nicegui4bots/background.py --wss-host <your_local_ip> --http-host <your_local_ip>
python nicegui4bots/foreground.py
```
