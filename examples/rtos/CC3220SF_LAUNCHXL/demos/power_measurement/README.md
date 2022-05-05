## Example Summary

This example introduce the user to the easily configurable power management drivers on CC32xx devices.

## Peripherals Exercised

* `Board_LED0` - Indicates that the board was initialized within `main()`
and while blinking, indicates that the board is trying to connect to AP.

## Example Usage

* Connect current measurement tool.
> Remove the jumper from J24 (VBAT) and connect your current measurement tool to it.

* Open the common.h file and configure your network parameters.
Set values for the: __SSID_NAME__, __SECURITY_TYPE__ and __SECURITY_KEY__ defines

* Build the project and flash it by using the SimpleLink Uniflash application
> The example could not be executed from the debugger.

* Open a serial session (e.g. `HyperTerminal`,`puTTY`, etc.) to the appropriate COM port.
> The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

The connection should have the following settings
```
    Baud-rate:    115200
    Data bits:       8
    Stop bits:       1
    Parity:       None
    Flow Control: None
```

* Run the example by pressing the reset button. `Board_LED0` turns ON to indicate TI-RTOS driver
initialization is complete.

* The example starts by showing on the terminal the following options:
```
    1) for Hibernate.
    2) for LPDS.
    3) for Transceiver Mode
    4) for Intermittently Connected.
    5) for Always Connected
```

* Select the request power mode by pressing the appropriate number

* The example proceeds according to the selected mode:
    - Hibernate mode enters immediately to hibernate which represents the lowest power state of the device.

    - LPDS mode enters immediately to low power deep sleep.

    - Transceiver mode enters the device into hibernate state between operation times.

    - Intermittently Connected mode enters the device into hibernate state and try to establish connection
      with the access point between operation times.

    - Always Connected mode starts by showing on the terminal the following options:
```
    1) for LPDS.
    2) for LSI.
    3) for IoT Low Power.
```
    Always Connected mode enters the device into LPDS\ LSI (long sleep interval) \ IoT low power
    and periodically wakes up to service beacons in order to keep connected with the access point
	(The LSI \ Iot Low Power duration is configured by user input).

* Start measure current
> Please refer to the expected power consumption website link (see References).

* Reset the device in order to start measure again

## Application Design Details

* This example provides users the ability to configure the device in various low power uses cases,
  for the purposes of current consumption measurements.
  The following describes the power profiles details:

    - Hibernate mode enters immediately to hibernate which represents the lowest power state of the device.
      In this mode, all the voltage sources, like DCDC or LDOs, within the power management unit are shut off.
      Very few logic, which works directly on battery power, is ON and they work on 32 KHz clock.

    - LPDS mode enters immediately to low power deep sleep with current range (0.12mA - 0.14mA)
      In this mode each subsystem processor requests the clock management unit for shutting off their subsystem.
      When both the subsystems request for this mode, Clock management unit will turn off the PLL, 40Mhz xtal,
      and the power management unit will shut off the power to each subsystem and scale down the voltage
      of always on domain to 0.9V. Active logic in this mode will work on 32 KHz clock.

    - Transceiver mode enters the device into hibernate state between operation times,
      and the socket in use is RAW hence not requires use of networking services.
      'NOT_ACTIVE_DURATION_MSEC' defines the hibernate time period between 2 active states.

    - Intermittently Connected mode is for devices that need to operate between long time intervals.
      In this mode, the device is trying to connect to the access point (Board_LED0 starts blinking)
      then enters hibernate state between working cycles.
      Almost all the device components are shut-down, hence when waking up a new connection needs to be established.
      In order to change the network parameters (e.g. `SSID_NAME`), open for edit the common.h file, make
      your changes and build.
      'NOT_ACTIVE_DURATION_MSEC' defines the hibernate time period between 2 working cycles.

    - Always Connected mode is trying to connect to the access point (Board_LED0 starts blinking)
      then enters the entire system is in LPDS\ LSI(long sleep interval) \ IoT low power 
      with only the WLAN subsystem periodically waking up to service beacons.
      In order to change the network parameters (e.g. `SSID_NAME`), open for edit the common.h file, make
      your changes and build.

> Please install the latest CC32xx Service Pack in order to get the most optimization and lowest current consumption.

## References
* The expected power consumption numbers are described in the the following website:
  [http://www.ti.com/product/CC3220/datasheet/specifications#SWAS031889](http://www.ti.com/product/CC3220/datasheet/specifications#SWAS031889)
* For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)