# CoreLPC for RepRapFirmware

This is an experimental port of DC42's [CoreNG](https://github.com/dc42/CoreNG) for LPC1768/LPC1769 based boards that is required build the [LPC Port of RepRapFirmware v2 RTOS](https://github.com/sdavi/RepRapFirmware/tree/v2-dev-lpc) (forked from DC42s [RepRapFirmware Version 2 (RTOS)](https://github.com/dc42/RepRapFirmware/tree/v2-dev)). This project also contains [FreeRTOS](https://www.freertos.org/) and [FreeRTOS+TCP](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/index.html) (with additions to work with RepRapFirmware and a ported 17xx networking driver based on the FreeRTOS+TCP 18xx driver).


## LPC Port of RepRapFirmware
This page also describes the [LPC Port of RepRapFirmware v2 RTOS](https://github.com/sdavi/RepRapFirmware/tree/v2-dev-lpc) that is used with this project to compile RepRapFirmware for LPC boards.

There are two configurations of the firmware:
* Non-Networking build that includes LCD support; and
* One with ethernet networking support built in and No LCD support.  *Note: Networking builds should not be used on a board that does not have a PHY chip installed.*

**These ports are experimental and are likely to contain bugs - Use at your own risk - All configuration/settings etc should be checked in the source code to ensure they are correct**

*Note: This firmware does not show up as a mass storage device when connected to a computer. Physical access to the internal sdcard is generally required in order to revert back or update.*


### Main Differences to [DC42's RepRapFirmware Version 2 (RTOS)](https://github.com/dc42/RepRapFirmware/tree/v2-dev):
* A maximum of 4 files can be open at a time.
* External interrupts (i.e., fan rpm etc) are limited to 3 and need to be defined at compile time to help reduce memory usage.
* Hardware Timers may be used generate 3 PWM frequencies: 10Hz, 50Hz and 250Hz by default, to provide PWM to non-PWM pins or support multiple PWM frequencies to run at the same time. This is in addition to the Hardware PWM which is configured to generally run at 250Hz.
* Users that wish to use a spare pin for servo probes will need to either update the header file and recompile or use a HW pin which may be running at a higher frequency than 50Hz if one of the other HW-PWM pins is running.
* Specific to Networking Builds:
  * Unable to run M503 or any other command that will output a lot of text to the console over the web interface due to memory constraints - this may cause the web interface to become unresponsive.
  * Only 1 connection to the web interface at a time.
  * No Ftp and Telnet interfaces
* Configuration:
  * GCode [M350](https://duet3d.dozuki.com/Wiki/Gcode#Section_M350_Set_microstepping_mode) - Microstepping for boards included in this port is done via hardware and thus M350 is not required. You may include it in your config.g if you like, but the command has no effect on the operation of the firmware.
  * Some drivers may require specifying the timing information as they require longer pulse timings than the configured default. Timing information for stepper drivers can be added using [M569](https://duet3d.dozuki.com/Wiki/Gcode#Section_M569_Set_motor_driver_direction_enable_polarity_and_step_pulse_timing). Timing information can usually be found in the stepper driver data sheets.    
  * Networking builds have the following restrictions when using auto-calibration:
    * Maximum number of probe points of 121; and
    * Delta maximum calibration points of 16
* Endstop Connections - Many boards provide endstop connections for both Max and Min, however, this firmware only supports 1 endstop per axis. For those boards, the [X,Y,Z] endstop pins are configured to use the [X,Y,Z]-MAX headers (regardless of where the endstops are physically located on the machine) and the Z-Probe is connected to the Z-MIN header. The X-MIN and Y-MIN are unused in this port and can be used as spare pins. The function of the endstop is set in config.g (see RepRapFirmware documentation) to configure the endstop physically located at the min or max on the axis.

## Compiling RepRapFirmware for LPC

Clone this repository, along with the following:
* [RRFLibraries](https://github.com/sdavi/RRFLibraries)
* [LPC port of RepRapFirmware v2 RTOS](https://github.com/sdavi/RepRapFirmware/tree/v2-dev-lpc)

The ARM toolchain needs to be installed to build the firmware. An example makefile has been included and should be edited to suit your settings, board selection, check all paths etc are correct before running make. If all is successful you should get a firmware.bin file in the same directory as the makefile.

*Note: it does not currently detect when header files are modified so it is required to clean the build files and run make again after modifying them.*

Built upon open source projects including [Explore-M3](https://github.com/ExploreEmbedded/Explore-M3), [CoreNG](https://github.com/dc42/CoreNG), [MBED](https://github.com/ARMmbed/mbed-os), [Smoothieware](https://github.com/Smoothieware/Smoothieware), [FreeRTOS](https://www.freertos.org/), [FreeRTOS+TCP](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/index.html) and [LPCOpen](https://www.nxp.com/support/developer-resources/software-development-tools/lpc-developer-resources-/lpcopen-libraries-and-examples/lpcopen-software-development-platform-lpc17xx:LPCOPEN-SOFTWARE-FOR-LPC17XX)

License: GPLv3, see http://www.gnu.org/licenses/gpl-3.0.en.html. This project includes source code from the above 3rd party projects and are covered by their respective licenses.
