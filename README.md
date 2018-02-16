# MotorDaemon SAM3x8e (WIP)
A feedback loop motor deamon with odometer support for the Atmel Cortex M3 SAM3x8e
This has the UDOO QUAD as the main target behind this project.

This daemon consist of multiple threads to handle odometry, motors, actuators and a piloting interface.
This is made to be flashed and launched from MotorDaemon with interrupt-control.

This is a platformio-based project, to flash it use bossac.

This project is part of INTechOS (https://github.com/discord-intech/meta-intechos)
