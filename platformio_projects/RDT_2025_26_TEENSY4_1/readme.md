# project usage:

flags in config are mostly for testing. could move thos to test folder somehow probably. change their boolean values to enable/disable features like ramping and command timeout.

### if youre only testing:
0. connect through usb to the i2c **parent**, power child on by connecting the 3S battery
1. ensure i2c wiring is correct (sda to sda, scl to scl, gnd to gnd)
2. if code is already uploaded to both mcus just open serial monitor with `pio device monitor`
3. if you need to upload code, use the commands below. you can open two serial monitors at once to see both sides of the i2c communication and join them in vscode to see them together, or do it one at a time (in that case upload the main code first)

#### to upload main code:

```bash
pio run -t upload -d platformio_projects/RDT_2025_26_TEENSY4_1
```
or
```bash
pio run -t upload -d platformio_projects/RDT_2025_26_TEENSY4_1 --upload-port COM3
```
(upload port is optional if you just have one mcu plugged in)

#### upload test file:

```bash
pio run -t upload -d platformio_projects/i2c_parent --upload-port COM4
```
(replace COM3/4 with your actual ports - you can see which ones are connected in port menu under you terminal in vscode)

sometimes this will act up, then disconnect the other usb from your laptop while uploading, then reconnect after upload is done and it should work fine.


## corresponding test file:

testing project for parent side i2c is [/platformio_projects/i2c_parent](./../platformio_projects/i2c_parent) - lets you send loco commands via serial (WASD in serial monitor)

## opening serial monitor(s):
```bash
pio device monitor -p COM3
```
and in another terminal:
```bash
pio device monitor -p COM4
```

you can join two terminals to see both sides at once by selecting two with ctrl and rightclicking then selecting "join terminals"

in parent terminal, control direction with WASD, speed with E and Q, and stop with X or space.
