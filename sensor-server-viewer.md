# sensor-server-viewer

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
python sensor-server-viewer/background.py \
     --wss-host <your_sensor_source_ip> \
     --http-host <your_sensor_source_ip> \
     --wss-port <your_sensor_source_wss_port> \
     --http-port <your_sensor_source_http_port>
python sensor-server-viewer/foreground.py
```

- **your_sensor_source_ip** can be found inside the SensorServer app in the hamburger menu.
- **your_sensor_source_http_port** can also be found inside the SensorServer app in the hamburger menu.
- **your_sensor_source_wss_port** can on the other hand be found in the **server** section of the main screen of the Sensor Server app once you click **Start**.
