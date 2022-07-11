# Snapmaker 2.0 Controller Firmware

Snapmaker2-Controller is the firmware for Snapmaker 2.0 3-in-1 3D Printers. It's based on the popular [Marlin firmware](http://marlinfw.org/) with optimized FreeRTOS support.

## Documentation

Snapmaker2-Controller builds on the codebase of Marlin 2.0, we also added some new features:

- Build on top of FreeRTOS.
- Support 3 different function modules: 3D Printing Module, Laser Module, CNC Module.
- Support add-ons like *Rotary Module* and *Enclosure Module*.
- CAN based communication system.
- Snapmaker specific functions like Auto-leveling, Laser Auto-Focus, Power-Loss Recovery etc.
- HMI (Touch Screen) communication over SSTP.

To read more documentations about **Snapmaker2-Controller**, you can checkout the [Snapmaker2-Controller Overview](https://github.com/Snapmaker/Snapmaker2-Controller/blob/main/docs/Overview.md) or [Hardware Link](https://github.com/Snapmaker/Snapmaker2-Controller/blob/main/docs/Hardware-Link.md) for further reading.

## Feedback & Contribution

- To submit a bug or feature request, [file an issue](https://github.com/Snapmaker/Snapmaker2-Controller/issues/new) in github issues.
- To contribute some code, make sure you have read and followed our guidelines for [contributing](https://github.com/Snapmaker/Snapmaker2-Controller/blob/master/CONTRIBUTING.md).

## Development

### Setup Development Environment

As of recommended in Marlin's development settings, we use **Visual Studio Code** and **PlatformIO IDE** to develop Snapmaker2-Controller. 

- Follow [Setting up Visual Studio Code](https://code.visualstudio.com/docs/setup/setup-overview) to install and setup **VSCode**.

- Follow the [guide](https://platformio.org/install/ide?install=vscode) to install PlatformIO extension in **VSCode**.
- Clone [Snapmaker2-Controller repo](https://github.com/Snapmaker/Snapmaker2-Controller) using Git to your local folder.

```shell
> git clone git@github.com:Snapmaker/Snapmaker2-Controller.git
```

- Open downloaded repo in **VSCode**
  - Use the **Open Folder…** command in the **VSCode** **File** menu
  - Then choose top folder of **Snapmaker2-Controller** in your location
- After opening the source code in **VSCode**, you will see these icons at the bottom status bar，it also indicates PlatformIO has been installed successfully.

![VSCode with PlatformIO](https://user-images.githubusercontent.com/3749551/98325327-814d3200-2029-11eb-9dd8-df9bee2dcbad.png)

### Ensure your changes are loaded

- The machine will not load new firmware if the version string remains the same
- You must update [Marlin/src/inc/Version.h](https://github.com/Snapmaker/Snapmaker2-Controller/blob/main/Marlin/src/inc/Version.h) to change the `SHORT_BUILD_VERSION` or your changes will not be loaded when flashing the firmware.

### Compile the code

- To compile the code, you have two ways:
  - click the **build** icon in status bar
  - click the **terminal** icon to open terminal, then type command ***pio run***

NOTE: if you build the source for first time, PlatformIO will download the relative libraries and toochains. It may take a few minutes.

- After PlatformIO finishing the build, you will get two images:
  - `(PROJECT FOLDER)/.pioenvs/GD32F105/firmware.bin`: image to be programmed into main controller
  - `(PROJECT FOLDER)/.pioenvs/GD32F105/firmware.elf`: image used to debug the firmware (in online debug tools like Ozone) 

- To clean previous build, just click the **clean** icon, or type command ***pio run -t clean*** in the terminal.

### Program compiled firmware to main controller

#### With PlatformIO CLI

After building, type below command in VSCode terminal

```
> pio run -t pack
```

Then you will get below firmwares in the folder `(PROJECT FOLDER)/release`:

- `firmware.bin`: raw binary of firmware.
- `firmware.elf`: firmware with debug information.
- `SM2_MC_APP_{xxx such as V4.0.0}_{xxx such as 20201222}.bin`: minor image of module, can be used to generate major image
- `Snapmaker_{xxx: version such as V4.0.0}_{xxx: date such as 20201222}.bin`: major image which can be used to upgrade modules with USB stick

Finally, copy the major image to your USB stick and upgrade your machine follow the instructions in [How to update Firmware](https://forum.snapmaker.com/t/snapmaker-2-0-firmware-updates-and-downloads/5443/10) section.

#### With Luban

You need to install [Luban](https://github.com/Snapmaker/Luban) to package the compiled firmware.

First, Open **Settings** -> **Firmware Tool** in Luban, upload the compiled `firmware.bin`, click **Compile and Export**. You will get a file with name like `Snapmaker2_V3.2.0_20201117.bin`, this is the packaged update file to be programmed.

Then, Update your firmware via USB follow the instructions in [How to update Firmware](https://forum.snapmaker.com/t/snapmaker-2-0-firmware-updates-and-downloads/5443/10) section.

## License

Snapmaker2-Controller is released under terms of the GPL-3.0 License.

Terms of the license can be found in the LICENSE file or at https://www.gnu.org/licenses/gpl-3.0.en.html.
