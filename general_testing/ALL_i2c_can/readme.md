# all the platformio projects in this folder are for testing the i2c and can bus communication between the teensy and another teensy (to be replaced by jetson send same command strucutre)

## Currently used:

The most complete child project (that is/will be integrated into full system) is [/i2c_can_child_tony](./i2c_can_child_tony)

The project used to test it is [/i2c_parent_CAN](./i2c_parent_CAN) but obviosuly that should be handled by the software team and their python-based code on the jetson

These only have the implementation for locomotion control but they form a foundation for the rest of the system and have stubs for excav and depo. the actual project is in [/platformio_projects\RDT_2025_26_TEENSY4_1](../../platformio_projects/RDT_2025_26_TEENSY4_1)