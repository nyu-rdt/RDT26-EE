# RDT26-EE
NYU Robotic Design Team Electrical Engineering subsystem repo

## CI Status
[![Main Project Compile](https://github.com/nyu-rdt/RDT26-EE/actions/workflows/platformio-main.yml/badge.svg?branch=main&event=push)](https://github.com/nyu-rdt/RDT26-EE/actions/workflows/platformio-main.yml)
[![i2c_parent_CAN Compile](https://github.com/nyu-rdt/RDT26-EE/actions/workflows/platformio-i2c-parent.yml/badge.svg?branch=main&event=push)](https://github.com/nyu-rdt/RDT26-EE/actions/workflows/platformio-i2c-parent.yml)
[![Impacted Projects Compile](https://github.com/nyu-rdt/RDT26-EE/actions/workflows/platformio-all.yml/badge.svg?branch=main&event=push)](https://github.com/nyu-rdt/RDT26-EE/actions/workflows/platformio-all.yml)

*(Third badge compiles only PlatformIO project directories touched by the push/PR)*

For help with C/C++ or using git and github, refer to training presentations in the NYU RDT 2025 - 2026 / Electrical Engineering / EE Trainings google drive

## where is the robot code?

Embedded code for robot control is / will be in [/platformio_projects](./platformio_projects). code from last years robot is in [/platformio_archive](./platformio_archive) (useful to review!) check readmes to identify the right project to use.

Review docs in [/docs](./docs) for coding style, testing, and contributing guidelines.

## where are the PCBs?

all PCBs, schematics etc. are in the Google Drive or in [/HARDWARE](./HARDWARE) folder

## new files
pls try and keep new files organized in the right folders - e.g. tests in /tests, docs in /docs, hardware files in /HARDWARE etc.

when testing things try and use the platformio test framework instead of making throwaway files.

If you do need to make such files (really want to use arduino IDE or something) please place them in [/general_testing](./general_testing) so they don't clutter the main repo.
