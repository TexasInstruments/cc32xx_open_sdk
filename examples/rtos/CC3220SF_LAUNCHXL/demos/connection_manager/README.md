## Example Summary

This example introduce the user to use of the SimpleLink Connection Manager with CC32xx/CC31xx devices.
This module completes the interface abstarction for the simplelink devices (together with SlNetIf, SlNetSock and SlNetUtils).

The SimpleLink Connection Manager enables a simple interface control for the SimpleLink network interfces.
A user can enable or disable the interface through the SlNetConn_start() and SlNetConn_Stop() API calls.
This will work for Wi-Fi and/or Ethernet (on MSP432E) device.

The SlNetConn enables different application modules to request a connection the AP with a different service levels (MAC, IP or INTERNET).
Once the connection is established, the SlNetConn_start() will return (see also the options of setting timeout for the API) and enable networking functionilty.
The SlNetConn event handlers (handler per start/stop client) will provides indication on connection status changes.

The example runs through couple of iterations in which the app requests a connection (SlNetConn_start()), run a ping to the local gateway or to external 
internet server, and then cancel the request (SlNetConn_stop()) and sleep for 2 seconds - which enable the device enter a low power mode.
The first iteration may include provisioning or setting of a static network profile. 

In order to enable the connection manager with CC32XX, the SlWifiConn module needs to be initialized.
The SlWifiConn enables control over Wi-Fi specifc parameters such AP connection timeouts and provsioning settings (see SlWifiConn_setConfiguration()).
The user can enable provisioining through calls to SlWifiConn_enableProvisioning() or add a profile based on
on external (e.g. WAC) or out-of-band provsioning methods (e.g. via BLE or NFC) using SlWifiConn_addProfile().

The SlWifiConn Callback provides indications when the WI-FI enter or exit low-power mode, indocation upon entering and exiting provisioning and requests
to start or stop external provisioning.  

wifi_if - is an application provides another layer of simplification for the app developer supporting basic WLAN Station use-cases 
through hard coded configuration (see wifi_if.h).
Please use the user settings section in "wifi_if.h" to enable/disable provisioning and to set proviosionign parameters.
It is also possible to set a static profile for the local network (for development).
Other configurations include the WIFI debug log level and settings (stack size and priority) of the wi-fi related threads.

## Limitations
1. The new interface should be used by applications using the Station role only (AP provisioning is supported but not other AP use-case).
2. The new interface requires NWP to be enabled at all time (it will maintain low-power mode when possible). Any exteranl call to sl_Start()
and sl_Stop() should be avoided. Please use the SlWifiConn_setRole, SlWifiConn_restart, or SlWifiConn_deinit instead.
 


## Peripherals Exercised

* `Board_LED0` - Blinking led Indicates disconnected. Led on indicates connection established,
				 Fast Blinking led Indicates provisioing in process.
* `Board_LED1` - Led on indicates error occurred
* `Board_LED2` - Led on indicates error occurred
and while blinking, indicates that the board is trying to connect to AP.

## Example Usage

* Build the project and flash from the debugger or by using the SimpleLink Uniflash 

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

* The example starts by showing on the terminal the application name and tries to establish connection with existing profiles.
If connection is not establish during preconfigured time out, provisioning will start, wait for connection to be established and IP acquire.
After connection is establish Ping is continuously sent to default gateway to verify the connection.

### notes 
1. By default the app is configured to start a one-shot provisioning. If the provsioning fails or AP needs to be replaced, please use SW2 to re-enable a one-shot provsioning.
2. Static network profile can be set through macro defnition in "wifi_if.h" (look for AP_SSID, AP_PASSWORD), or through a user file (as defined by AP_CFG_FILENAME).
3. If static profile is not in use please set the AP_SSID and AP_PASSWRD as NULL (no string). If static profile is defined, it will precede any stored profile.
   In such case the device will try to connect to the static profile. If not connection is established after "Connection Timeout" expires, it will remove the static configuration 
   and try again to connect to the stored profiles (waiting another "Connection Timeout before starting provisioning or going to the idle (low-power) state). If NULL is used
   for the definition of the static profile - the first connection-wait timeis omitted. 
4. Please set WIFI_IF_DEBUG_LEVEL to E_DEBUG - in order to get internal wi-fi information.


## Application Design Details

The SimpleLink Wi-Fi provisioning process is divided into the Configuration stage and the Confirmation stage. 
The process begins with the configuration stage. During this stage, the SimpleLink Wi-Fi device waits for the end user to provide 
(using an external device such as a smartphone or tablet running a dedicated provisioning application) the information needed to connect to 
the wireless network as follows:  

•  Network name (SSID)  
•  Password  
•  Device name (optional)  
•  UUID (optional)  

The device internally saves the provided network information into its serial flash memory as a new profile. Once a profile is successfully configured, 
the device moves to the confirmation stage. The confirmation stage tests the profile that was configured during the configuration stage. 
During the confirmation stage, the device tries to connect to the wireless network found in the new configured profile. 
If the connection is successful, the device also tries to provide feedback about the successful connection to the user’s smart phone provisioning application, 
which configured the profile. A connection is defined as successful if the WLAN connection is established, and an IP address is acquired.
If the connection is successful, and the feedback is delivered to the user, the confirmation stage is successful, and the provisioning process successfully ends. 
If the connection attempt fails, or if it is successful but the feedback is not delivered to the user, the confirmation stage fails, 
and the device moves back to the configuration stage. At this point, the user’s smart phone provisioning app can ask the device to send the fail reason 
of the previous confirmation attempt, and configure a new profile.

## External Configuration (partial supported currently)

When the provisioning process is started in APSC + external configuration mode , the device is ready to
serve stations trying to connect to it (for AP provisioning), ready to handle SmartConfig transmissions (SC
provisioning), and can allow the host to execute an additional external provisioning method that is not
implemented inside the networking subsystem (for example: WAC).
Please use the the wifi_if API to register the external provsioning Start and Stop callbacks (note that the Start callback is already called from a dedicated execution
thread context).

The provisionig mode is defined in "wifi_if.h". Please also refer to the "wifi_if.c" to check the actual SlWifiConn interface.
The "wifi_if.h" is provided as a reference and and can be modified.

> Please install the latest CC32xx Service Pack in order to get the most optimization and lowest current consumption.
### Note: SP 4.8 (for CC3x3x) or SP 3.17 (for CC3x2x) are the minimum requirement for using the connection manager. 

## References
* For more detailed information please refer to the [programmer's user guide](http://www.ti.com/lit/SWRU455) and review the Provisioning chapter 15.
* For more detailed information regarding the provisioning feature, please refer to the [provisioning application report](http://www.ti.com/lit/SWRA513)
* For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)
