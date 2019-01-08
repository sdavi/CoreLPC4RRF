# **Important Changes**

[8 Jan 19]
* Endstop configuration has changed for boards with both min/max endstop headers. Endstops are now connected to their respective headers as normal (i.e. min endstop to the min header). [M574](https://duet3d.dozuki.com/Wiki/Gcode#Section_M574_Set_endstop_configuration) is used to specify the low/high end and now also selects the correct hardware Min/Max header respectively. Check endstops are working correctly with M119.
* Boards without a dedicated probe header will need to configure [M558](https://duet3d.dozuki.com/Wiki/Gcode#Section_M558_Set_Z_probe_type) to P4 and use the C parameter to select a free endstop header that the probe is connected to. For example, if the ZProbe is connected to the ZMax header, then P4 C5 should be added to M558. See table below for endstop number mapping.

| Endstop Header | Xmin | Ymin | Zmin | Xmax | Ymax | Zmax |
| - | - | - | - | - | - | - |
| Endstop Number | 0 | 1 | 2 | 3 | 4 | 5 |

* For boards with only X,Y,Z and Probe headers, the endstops are configured as being either on the high or low end via M574. The probe can use M558 P5 to select the probe header to operate as a digital input.



## LPC Port of RepRapFirmware

There are two configurations of the firmware:
* Non-Networking build that includes LCD support; and
* One with ethernet networking support and No LCD support.  *Note: Networking builds should not be used on a board that does not have a PHY chip installed.*

**The LPC port is experimental and are likely to contain bugs - Use at your own risk - All configuration/settings etc should be checked in the source code to ensure they are correct**

*Note: This firmware does not show up as a mass storage device when connected to a computer. Physical access to the internal sdcard is generally required in order to revert back or update.*


### Main Differences to [DC42's RepRapFirmware Version 2 (RTOS)](https://github.com/dc42/RepRapFirmware/tree/v2-dev):
* A maximum of 4 files can be open at a time.
* External interrupts (i.e., fan rpm etc) are limited to 3 and need to be defined at compile time to help reduce memory usage.
* Hardware Timers may be used generate 3 PWM frequencies used: 10Hz, 50Hz and 250Hz by default, to provide PWM to non-PWM pins or support multiple PWM frequencies to run at the same time. This is in addition to the Hardware PWM which is configured to generally run at 250Hz.
* Users that wish to use a spare pin for servo probes will need to either update the header file and recompile or use a spare HW pin which may be running at a higher frequency than 50Hz if one of the other HW-PWM pins is running. ReArm config is setup to provide 50Hz on Servo1, Servo2 and Servo3 pins.
* Specific to Networking Builds:
  * Unable to run M503 or any other command that will output a lot of text to the console over the web interface due to memory constraints - this may cause the web interface to become unresponsive. The web interface provides an alternative way to view and edit the config files.
  * Only 1 connection to the web interface at a time.
  * No Ftp and Telnet interfaces
* Configuration:
  * GCode [M350](https://duet3d.dozuki.com/Wiki/Gcode#Section_M350_Set_microstepping_mode) - Microstepping for boards included in this port is done via hardware and thus M350 is not required. You may include it in your config.g if you like, but the command has no effect on the operation of the firmware.
  * Some drivers (such as the DRV8825) may require specifying the timing information as they require longer pulse timings than the configured default that can cause missed steps. Timing information for stepper drivers can be added using [M569](https://duet3d.dozuki.com/Wiki/Gcode#Section_M569_Set_motor_driver_direction_enable_polarity_and_step_pulse_timing). Timing information can usually be found in the stepper driver data sheets.    
  * Networking builds have the following restrictions when using auto-calibration:
    * Maximum number of probe points of 121; and
    * Delta maximum calibration points of 16
