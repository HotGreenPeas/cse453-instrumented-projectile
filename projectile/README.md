# setup

## anchor
upload ```anchor_display.ino``` to a ESP32 UWB Pro with Display

## projectile
upload ```smart_shell.ino``` to a ESP32 UWB Pro

## data receiver 
upload ```hc12_arduino.ino``` to an Arduino

## libraries and dependencies
download ```libraries.zip``` and unzip the contents into the Arduino libraries folder

# future work
- attempt to reduce loop to ~100 ms, currently ~120 ms
- test max distance
- test battery
- cannon will automatically detect when the projectile is loaded
- calibrate sensors
- calculate time of flight
- find orientation of projectile??
- make sense of acceleration and display understandable data
- test fire
- automatic power shutoff in case of high temps
- validate sensor reading, reduce noise, reject invalid data
- communication handshake
- indicator leds
- calculate ground distance
- bearing of projectile??
