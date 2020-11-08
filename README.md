# Snapmaker 2.0 Controller Firmware

Snapmaker2-Controller is the firmware for Snapmaker 2.0 3-in-1 3D Printers. It's based on the popular [Marlin firmware](http://marlinfw.org/) with optimized FreeRTOS support.



## Features of Snapmaker2-Controller

- Build on top of FreeRTOS.
- Rotary Module add-on supports.



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

### Compile the code

- To compile the code, you have two ways:
  - click the **build** icon in status bar
  - click the **terminal** icon to open terminal, then type command ***pio run***

NOTE: if you build the source for first time, PlatformIO will download the relative libraries and toochains. It may take a few minutes.

- After PlatformIO finishing the build, you will get two images
  - (Top)/.pioenvs/GD32F105/firmware.bin: image to be programmed into main controller
  - (Top)/.pioenvs/GD32F105/firmware.elf: image to be used in Ozone(online debug tool) 

- To clean previous build, just click the **clean** icon, or type command ***pio run -t clean*** in the terminal



### Program compiled firmware to main controller

TBD



## License

Snapmaker2-Controller is released under terms of the GPL-3.0 License.

Terms of the license can be found in the LICENSE file or at https://www.gnu.org/licenses/gpl-3.0.en.html.
