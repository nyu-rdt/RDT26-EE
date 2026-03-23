testing project for parent side i2c is [/general_testing\ALL_i2c_can\i2c_parent_CAN](./../general_testing/ALL_i2c_can/i2c_parent_CAN) - lets you send loco commands via serial (just wasd in serial monitor)

flags in config are mostly for testing. could move thos to test folder somehow probably.

to upload main code to teensy and parent test code to another, run

pio run -t upload -e RDT_2025_26_TEENSY4_1 --upload-port COM3
pio run -t upload -e i2c_parent_CAN --upload-port COM4

sometimes this will act up, then disconnect the other usb from your laptop while uploading, then reconnect after upload is done and it should work fine.

(replace COM3/4 with your actual ports - you can see which ones are connected in port menu under you terminal in vscode)

then to monitor:

pio device monitor -p COM3
pio device monitor -p COM4

you can join two terminals to see both sides at once by selecting two with ctrl and rightclicking "join terminals"
