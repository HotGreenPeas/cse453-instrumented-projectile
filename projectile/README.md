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
| task                                                           | effort | impact |     priority     | status |
|----------------------------------------------------------------|--------|--------|------------------|--------|
| attempt to reduce loop to ~100 ms, currently ~120 ms           | high   | low    | :sleeping:       | listed |
| cannon will automatically detect when the projectile is loaded | med    | med    | :coffee:         | listed |
| calibrate sensors                                              | med    | med    | :coffee:         | listed |
| calculate time of flight                                       | high   | high   | :star:           | listed |
| find orientation of projectile                                 | high   | low    | :coffee:         | listed |
| make sense of acceleration and display understandable data     | med    | med    |  :star:          | listed |
| automatic power shutoff in case of high temps                  | med    | low    | :sleeping:       | listed |
| validate sensor reading, reduce noise, reject invalid data     | med    | high   | :rotating_light: | listed |
| communication handshake                                        | med    | low    |  :star:          | listed |
| indicator leds                                                 | med    | low    | :sleeping:       | listed |
| calculate ground distance                                      | low    | high   | :sleeping:       | listed |
| bearing of projectile                                          | high   | low    | :coffee:         | listed |
| test max distance                                              | med    | high   | :fire:           | listed |
| test battery                                                   | high   | high   | :rotating_light: | listed |
| test fire                                                      | high   | high   |  :fire:          | listed |
| characterize power draw                                        | ???    | high   |  :fire:          | listed |
| redundant sensors                                              | med    | low    |  :sleeping:      | listed |
| test wind effect on pressure                                   | med    | low    |  :coffee:        | listed |
| test sensor and controller reliability                         | low    | med    |  :star:          | done  |
| reduce esp32 uwb power consumption                             | med    | med    |  :star:          | listed  |
| find new batteries                                             | med    | high   | :rotating_light: | listed  |
| sensor config                                                  | med    | med    | :star:           | listed  |
| state diagram                                                  | med    | med    | :star:           | listed  |


# broken parts
1. ~~bme280, cause of death: extended usage??~~ faulty wiring/breadboard
