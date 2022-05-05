## Example Summary

This example introduce the user to simple provisioning configuration on CC32xx devices.

## Peripherals Exercised

* `Board_LED0` - Blinking led Indicates disconnected. Led on indicates connection established
* `Board_LED1` - Led on indicates error occurred
* `Board_LED2` - Led on indicates error occurred
and while blinking, indicates that the board is trying to connect to AP.

## Example Usage


* Build the project and flash from the debugger or by using the SimpleLink Uniflash 
* In order to use HTTP, use the SimpleLink Uniflash under Advanced->Network Applications->HTTP Server
  - enable the secured check box
  - set Port Number to 443
  - Make sure Rom Pages are enabled
  - Enable secondary port and set to 80
  - In case of using the certificate playground (tools\cc32xx_tools\certificate-playground):
    Set certificate file name to: dummy-root-ca-cert 
    Set HTTP server private key: dummy-root-ca-cert-key
    Set mcu image certificate to: dummy-trusted-ca-cert and key to dummy-trusted-ca-cert-key


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

## External Configuration

When the provisioning process is started in APSC + external configuration mode , the device is ready to
serve stations trying to connect to it (for AP provisioning), ready to handle SmartConfig transmissions (SC
provisioning), and can allow the host to execute an additional external provisioning method that is not
implemented inside the networking subsystem (for example: WAC).

Using the external configuration requires to implement and uncomment the following functions (search for [External configuration] in the example code):
•  StartExternalProvisioning() - start the external provisioning process. once SL_WLAN_PROVISIONING_EXTERNAL_CONFIGURATION_READY arrives, call this 
function in order to start the external provisioning process (i.e open socket and receive the profile credentials).
•  StopExternalProvisioning() - stop the external provisioning process (i.e in case of AP provisioning or AP SC success, external provisioning is no longer needed).
•  IsActiveExternalConfirmation() - Implement a function which check the current status of the external provisioning run.

In order to start the provisioning in SL_WLAN_PROVISIONING_CMD_START_MODE_APSC_EXTERNAL_CONFIGURATION call sl_WlanProvisioning API with 
provisioningCmd = SL_WLAN_PROVISIONING_CMD_START_MODE_APSC_EXTERNAL_CONFIGURATION; 


> Please install the latest CC32xx Service Pack in order to get the most optimization and lowest current consumption.

## References
* For more detailed information please refer to the [programmer's user guide](http://www.ti.com/lit/SWRU455) and review the Provisioning chapter 15.
* For more detailed information regarding the provisioning feature, please refer to the [provisioning application report](http://www.ti.com/lit/SWRA513)
* For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)
