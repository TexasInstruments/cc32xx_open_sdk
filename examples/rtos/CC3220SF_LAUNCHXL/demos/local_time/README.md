## Example Summary

This example demonstrates the use of clock synchronization
Update is done upon need only and after period of time from the last update.

## Peripherals Exercised

* `Board_LED0` - Indicates that the board was initialized within `main()`
and while blinking, indicates that the board is trying to connect to AP.

## Example Usage

* open local_time.c and configure your network parameters.
Set values for the: __LOCALTIME_SSID_NAME__, __LOCALTIME_SECURITY_TYPE__ and __LOCALTIME_SECURITY_KEY__ defines

* Build the project and flash it by using the SimpleLink Uniflash application
or by executed it from the debugger.

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

* Run the example by pressing the reset button or by the debugger. `Board_LED0` turns ON to indicate TI-RTOS driver
initialization is complete.

* The example starts by showing on the terminal the application banner:
```
         ==============================================
            Local Time Example Ver: 1.0.0
         ==============================================

         CHIP: 0x20000010
         MAC:  2.0.0.0
         PHY:  2.2.0.4
         NWP:  3.2.0.0
         ROM:  0
         HOST: 2.0.1.22
         MAC address: 08:00:28:5b:55:e3

         ==============================================
```

* Afterwards, there could be two scenarios depends on device configured mode:
    Case the device configured as Station, the example tries to connect to defined AP
    Case the device configured as AP it shows the AP SSID  

* last, the example shows the following menu:
```
    1) Get time.
    2) Update time.
    3) Set time zone.
    4) Get time zone.
    5) Switch to AP mode.
    6) Switch to Station mode.
```

* Select the request option by pressing the appropriate number

* The example proceeds according to the selected mode:
    - Get time - update current local time and shows it.
                 Update whould be executed only if needed.

    - Update Time - only update current local time.
                 Update whould be executed only if needed.

    - Set time zone - set your local time zone as offset from GMT time in minutes.
                    The value could be positive or negetive

    - Get time zone - get your local time zone in minutes from GMT time 

    - Switch to AP mode - Set the device to Access Point mode (reset is required).

    - Switch to Station mode - Set the device to Station mode (reset is required).


## Application Overview

* This example demonstrates the ability to update the device current local time by global NTP servers.
  It make use of ClockSync library which updates current local time by using the SNTP library.
  The SNTP library sends request over the net to predefined NTP servers which are built in the library.
  The update should executed only after period of time was elapsed from the last update.
  The default is 1 minutes but it could be change by redefine __CLOCKSYNC_INTERVAL__ and rebuild ClockSync library.
  While calling ClockSync to update the local time, it could update NWP time in case __CLOCKSYNC_UPDATE_NWP__ definition set to 1.
  
* The example also provide the ability to switch between WLAN modes - Station and AP.
    > While in AP mode currently it could not update the time because the device does not support routing or bridging. 


> Please install the latest CC32xx Service Pack in order to get the most optimization.

## References
For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)

