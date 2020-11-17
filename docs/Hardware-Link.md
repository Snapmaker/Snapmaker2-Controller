# Snapmaker2-Controller Hardware Link 

This is the hardware link of Snapmaker2-Controller, it shows how different modules communicate with each other.

![Hardware-Link](https://user-images.githubusercontent.com/3749551/99341821-9053a000-28c5-11eb-8437-5dc9dea185ce.png)

### Controller <-> Luban

- USB to UART: Communication between Controller and Luban is original Marlin supported Gcode.

### Controller <-> HMI

- UART: Communication with HMI using SSTP (Snapmaker simple transformation protocol)
- USB 2.0: Provides the link to read/write USB stick plugged in

### Controller <-> 3D printing module

- Wired link: stepper signals (enable, direction, step signals)
- CAN bus:
    - indentify and configure module
    - upgrade module
    - read probe sensor
    - read filament sensor
    - read/write nozzle temperature
    - set FAN1/FAN2 speed
    - set PID parameters

### Controller <-> Laser module

- Wired link: laser power (remapped from step signal)
- CAN bus:
    - indentify and configure module
    - upgrade module
    - read/write laser focus data
    - Set FAN speed
- UART bus (remapped from enable and direction signals):
    - upgrade camera module
    - get bluetooth MAC address
    - get/set bluetooth name

### Controller <-> CNC module

- CAN bus:
    - indentify and configure module
    - upgrade module
    - set spindle speed

### Controller <-> Linear modules

- Wired link: stepper signals (enable, direction, step signals)
- CAN bus:
    - indentify and configure module
        - e.g. distinguish X, Y, Z linear modules
    - upgrade module
    - get the lead
    - get the length
    - get endstop status

### Controller <-> Rotary module

- Wired link: stepper signals (enable, direction, step signals)
- CAN bus:
    - indentify and configure module
    - upgrade module

### Controller <-> Enclosure add-on

- CAN bus:
    - indentify and configure module
    - upgrade module
    - get door status (open / closed)
    - set light bar brightness
    - set FAN speed

### Controller <-> Power Module

- 24V provider
- Wired link: Power-Loss signal

### HMI <-> Snapmaker Luban

- Wi-Fi API
    - file transfer
    - get machine status
    - G-code commands
    - remote printing control
    - ...

### HMI <-> Laser camera

- Bluetooth
    - LED control
    - take picture
    - file transfer
