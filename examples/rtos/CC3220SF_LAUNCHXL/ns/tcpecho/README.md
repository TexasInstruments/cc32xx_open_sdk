## Example Summary

This application demonstrates how to use TCP.


## Resources & Jumper Settings

> If you're using an IDE (such as CCS or IAR), please refer to Board.html in
your project directory for resources used and board-specific jumper settings.
Otherwise, you can find Board.html in the directory
&lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

Python 3.X is required for this example. To install python download the latest
version at https://www.python.org/downloads/release

## Example Usage

* Example output is generated through use of Display driver APIs. Refer to the
Display driver documentation found in the SimpleLink MCU SDK User's Guide.

* Build the project and flash it by using the Uniflash tool for cc32xx,
Or equivalently, run debug session on the IDE of your choice.

* Open a serial port session (e.g. 'HyperTerminal','puTTY', 'Tera Term' etc.) to the appropriate COM port -   
listed as 'User UART'.
The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

The connection should have the following connection settings:
```
    Baud-rate:     115200
    Data bits:          8
    Stop bits:          1
    Parity:          None
    Flow Control:    None
```

* The device must be connected to a network with a DHCP server to run this
example successfully (DHCP is configured by default).
In order to change the network parameters (e.g. `SSID_NAME`), open for edit the platform.c file, make
your changes and build.

For secured AP use:
```
#define SSID_NAME                             "DemoAP"                  /* AP SSID */
#define SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2
#define SECURITY_KEY                          "12345678"                /* Password of the secured AP */
```
For unsecured AP use:
```
#define SSID_NAME                             "DemoAP"                  /* AP SSID */
#define SECURITY_TYPE                         SL_WLAN_SEC_TYPE_OPEN
#define SECURITY_KEY                          ""
```                

* The example starts the network stack. When the stack receives an IP address from the DHCP server, the IP address is written to the console.

* Run the tcpSendReceive python script that is shipped with your SDK. The script is found in:
**&lt;SDK_INSTALL_DIR&gt;/tools/cc32xx_tools/echo-python-scripts/tcpSendReceive.py**

Usage:

```
python tcpSendReceive.py <IP-addr> <port> <id> -l[length] -s[sleep in mS] -n[number of transmits per report]

  <IP-addr>     is the IP address of the device
  <port>        is the TCP port being listened to (1000)
  <id>          is a unique id for the executable. Printed out with a packet transmission report.
                It allows the user to run multiple instances of tcpSendReceive.


  Optional:
    -l[length]      size of the packet in bytes. Default is 1024 bytes.
    -s[sleep in mS] usleep time to between sends. Default is 0 mSecs.
    -n[number of transmits per report] the number of transmits to occur before being reported onto the console. Default is 100 transmits.
```

  Example:
        **python tcpSendReceive.py 192.168.1.100 1000 1 -s100**

* Messages such as the following will begin to appear on the terminal window when a TCP packet has been echoed back:
```
        Starting test with a 100 mSec delay between transmits and reporting every 100 transmit(s)
        [id 1] count = 100, time = 10
        [id 1] count = 200, time = 20
        [id 1] count = 300, time = 30
```

## Application Design Details

* This application uses two types of tasks:

1. **tcpHandler** - Creates a socket and accepts incoming connections.  When a
                  connection is established a **tcpWorker** task is dynamically
                  created to send or receive data.
2. **tcpWorker**  - Echoes TCP packages back to the client.

	**tcpHandler** performs the following actions:
	   * Create a socket and bind it to a port (1000 for this example).
	   * Wait for incoming requests.
	   * Once a request is received, a new tcpWorker task is dynamically created to
	     manage the communication (echo TCP packets).
	   * Waiting for new requests.
	
	**tcpWorker** performs the following actions:
	   * Allocate memory to serve as a TCP packet buffer.
	   * Receive data from socket client.
	   * Echo the TCP packet back to the client.
	   * When client closes the socket, close server socket, free TCP buffer memory
	     and exit the task.

* TI-RTOS:

    * When building in Code Composer Studio, the kernel configuration project will
be imported along with the example. The kernel configuration project is
referenced by the example, so it will be built first. The "release" kernel
configuration is the default project used. It has many debug features disabled.
These feature include assert checking, logging and runtime stack checks. For a
detailed difference between the "release" and "debug" kernel configurations and
how to switch between them, please refer to the SimpleLink MCU SDK User's
Guide. The "release" and "debug" kernel configuration projects can be found
under &lt;SDK_INSTALL_DIR&gt;/kernel/tirtos/builds/&lt;BOARD&gt;/(release|debug)/(ccs|gcc).

* FreeRTOS:

    * Please view the `FreeRTOSConfig.h` header file for example configuration
information.

## References
For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)