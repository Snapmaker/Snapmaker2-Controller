# Snapmaker2-Controller Overview 

Snapmaker2-Controller is firmware designed for the Snapmaker 2.0 extendible modular system - a 3-in-1 modular 3D Printer.

Snapmaker2-Controller is developed by Team Snapmaker. While it was forked from the popular Marlin firmware, it inherits the original functions from Marlin as below.

![MarlinFrimware](https://user-images.githubusercontent.com/3749551/98461469-47c22580-21e7-11eb-8c28-5eb4895fa740.png)

- **G-code parser and executor**: Parse G-code coming from Snapmaker Luban software or HMI (the Touch Screen), and execute parsed G-code.
- **Motion control**: Plan G-code motions into motion blocks one by one, according to motion blocks planned, drive steppers in extruder(s) and linear modules; It also manages a coordinator system to support absolute/relative motion switching, work origin setting, etc.
- **Thermal Manager**: Control and monitor the heated bed heating with PID algorithm. (nozzle temperature is controlled by the 3D printing module itself)
- **Bed leveling**: Compensation Z offsets of movements using leveling data.

On top of Marlin firmware, Snapamker2-Controller has added functions that comes with Snapmaker 2.0, as demonstrated in the figure below.

![Snapmaker-Controller](https://user-images.githubusercontent.com/3749551/98461651-f31faa00-21e8-11eb-966a-c6cc08841d1f.png)

### **FreeRTOS**

During the development of Snapmaker2-Controller, we had a hard time dealing with race condition issues, such as handling of HMI requests, asynchronious module events, combined with the handling of original Marlin bussiness logic, in parallel. We bring FreeRTOS into Marlin to solve the problem, it also help us to separate complex logic in discrete tasks.

### **HMI event handler** 

Snapmaker2-Controller connects HMI (the Touch Screen) thru UART, it forwards received requests to the **HMI event handler** that running in a separate RTOS task.

### **Bed Level Service**

Provides API for auto leveling and manual leveling.

### **Upgrade Service**

Provides API for firmware upgrade requests and deals with upgrade of modules.

### **System Service**

Manages the state of system and provides API for state requests like

- Start a job
- Pause a job
- Stop a job
- Finish a job
- Recover a unfinished job
- Heartbeat to ensure connections

### **Quick Stop Serivce**

Handling of quick pause, stop, power-loss of steppers.

### **Power Loss Recovery Service**

Works with **System Service** and **Quick Stop Service** to provide the Power-Loss function in Snapmaker 2.0, it saves working environment data during Power-Loss and brings back working environment when recovering.

### **Module Manager**

In Snapmaker 2.0 modular system, each of the modules owns a MCU itself and connects to the main controller through *CAN bus*.

Each module can manage its own state without directly control of main controller. This enables our modules being very flexible and extendable.
For example, the 3D printing module controls its nozzle temperature by it own implemented PID temperature control algorithm, main controller doesn't need to take care of the complex 

While there are some exceptions, control of steppers, laser power and heated bed temperature is still in the hand of main controller.

The **Module Manager** communicates with all other modules, including:

- Discovery and configuration modules
- Upgrade of modules
- Communication Framework
- Module specific business logic


## Project Structure

Most of Snapmaker specific source files are located in the directory `{ROOT of repository}/snapmaker`, alongside with the `Marlin` folder.

Through the file structure, you can easily navigate to the functions you are looking for.

```
Snapmaker2-Controller
├── Marlin
├── snapmaker
   ├── lib
   │   └── GD32F1  // maple hal and other libraries such as FreeRTOS
   ├── scripts     // extra scripts for platformIO
   └── src         // source files developed by snapmaker team
       ├── common  // some common headers and APIs
       ├── gcode   // costumized Gcode
       ├── hmi     // HMI event handler mentioned above
       ├── module  // modules manager mentioned above
       ├── service // services such as system, upgrade, etc.
       └── utils   // some utilities
```
