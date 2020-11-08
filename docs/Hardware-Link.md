# Snapmaker2-Controller Hardware Link 

This is the hardware link of Snapmaker2-Controller, it shows how different modules communicate with each other.

![Hardware-Link](https://user-images.githubusercontent.com/3749551/98462552-0d10bb00-21f0-11eb-84d1-36e81ff4741e.png)

### Controller <-> Luban

- USB to UART: Communication between Controller and Luban is original Marlin supported Gcode.

### Controller <-> HMI

- UART: Communication with HMI using SSTP (Snapmaker simple transformation protocol)
- USB 2.0: Provides the link to read/write USB stick plugged in

### Controller <-> 3D printing module

- Wired link: stepper signals (enable, direction, step signals)
- CAN bus:
    - upgrade module
    - read probe sensor
    - read filament sensor
    - read/write nozzle temperature
    - set FAN1/FAN2 speed
    - set PID parameters

### Controller <-> Laser module

- Wired link: laser power (remapped from step signal)
- CAN bus:
    - upgrad module
    - read/write laser focus data
    - Set FAN speed
- UART bus (remapped from enable and direction signals):
    - upgrade camera module
    - get bluetooth MAC address
    - get/set bluetooth name

### Controller <-> CNC module

- CAN bus:
    - upgrade module
    - set spindle speed

### Controller <-> Linear modules

- Wired link: stepper signals (enable, direction, step signals)
- CAN bus:
    - upgrade module
    - get the lead
    - get the length
    - get endstop status

### Controller <-> Enclosure add-on

- CAN bus:
    - upgrade module
    - get door status (open / closed)
    - set light bar brightness
    - set FAN speed

### HMI <-> Snapmaker Luban

- Wi-Fi API
    - file transfer
    - get machine status
    - G-code commands
    - remote printing control
    - ...

### HMI <-> Laser camera

- Bluetooth
    - take picture
    - file transfer
