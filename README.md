# Air quality sensor to MQTT

The firmware I use to push measurements from my air quality sensors (BME680 and SDS011) to my MQTT broker.

Note that this uses the proprietary BME680 library. I do have concerns about it but after inspecting the
alternative libraries I concluded that, at this moment, the manufacturer seems to be the one having the
best API and providing the most accurate readings.

One could also argue that the SDS011, even though the protocol is very simple, runs its own proprietary
firmware, it's just 4 wires apart as opposed to running as part of the MCU firmware.

This code includes a library for the SDS011 that I've written myself, since the existing libraries that 
I could find are quite terrible. All functionality documented in the datasheet is exposed.

## Flashing

This software uses PlatformIO, so to flash you can run

```
pio run --target upload
```

after changing the UART parameters in `platformio.ini`. ArduinoOTA is also implemented.

## Remote logging

```bash
mosquitto_sub -h MQTT_BROKER -t homie/air-sensor/general/log -N
```