## Example Summary

This example demonstrates the various range of networking capabilities which the CC32xx device family provides.  
For simplicity, We divide our Application's functionality into four different silos:

| Silo          | Description           | 
| ------------- |:-------------|
| Wlan          | Contains link layer functions like scan, connect, etc. | 
| NetApp.       | Demonstrates the usage of networking applications.      | 
| Socket        | Shows variety of socket API and responsible for sending and receiving data.      |
| Transceiver    | Gives the user a direct interface to the NWP radio for RF tests, raw sockets (L1) and more.|    
Each silo contains several commands, accessible by the user via command line interface,    
which implemented using CC32xx user UART. Any events or application output, would be displayed over 
the UART terminal screen. Later on, in the [Available Commands](#Available-Commands-header) section, we'll discuss in greater detail about each silo  and the commands it includes.

## Example Usage

* Build the project and flash it by using the Uniflash tool for cc32xx,  
Or equivalently, run debug session on the IDE of your choice.
> This example contains one command that cannot be executed while running from the debugger - Wowlan command,  
since this command essentially sends the host processor to Low power deep sleep (LPDS).
* Open a serial port session (e.g. 'HyperTerminal','puTTY', 'Tera Term' etc.) to the appropriate COM port -   
listed as 'User UART'.
The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

The connection should have the following connection settings:
```
    Baud-rate:    115200
    Data bits:         8
    Stop bits:         1
    Parity:         None
    Flow Control:   None
```

* Run the example by pressing the reset button or by running debug session through your IDE.  
 `Board_LED0` turns ON to indicate the Application initialization is complete.

Once the application has complete it's initialization and the network processor is up,  
the application banner would be displayed, showing version details:
```
        ============================================
           Network Terminal Example Ver: 1.0.1.0
        ============================================
         CHIP: 0x30000019
         MAC:  2.0.0.0
         PHY:  2.2.0.3
         NWP:  3.1.99.4
         ROM:  0
         HOST: 2.0.1.15
         MAC address: 70:ff:76:1c:2a:15
        ============================================
```
At this point, a menu showing the available commands would be printed followed by command prompt,  
indicating that the application is ready for user input.

**Note**: The command line input is limited to 128 characters for each command.   
  
## Application's Overview
  
Network Terminal is a command line interface (cli) based application,  
used to demonstrate the CC32xx/CC31xx networking capabilities. It does that by offering a list  
of commands, fed to the application via UART and later parsed and dispatched.  
  
The application starts with invoking `cmd_prompt()` which polls the UART input for a command.  
Polling is completed once a cartridge return is detected. At this point in time, `cmd_prompt` possess  
the command name, followed by the user's choice of parameters. Once command is obtained,  
 `cmd_prompt` searches for the user's command in the global command table, `gCmdList`.  
For each command, this table stores three entries: the command string,  
 a callback which handles the command and a menu print callback that prints the help menu.  
  
If the typed command matches a table entry, the handler is dispatched and the parameters  
fed by the user are passed to the command callback.  
  All callback functions follow the same naming convention: `cmd<command_name>Callback`  
and resides at the appropriate file. For example, all WLAN related callbacks are implemented in: `wlan_cmd.c`.   
  
Inside each command callback, the first action is to parse the user's input. This is achieved by  
calling the corresponding parser function. A structure containing each command's parameters  
is also passed to the designated parser and returned filled with the parsed user's parameters.  
Later, the parsed structure is handled by the callback.  
All command parser function follows the same naming convention: `Parse<command_name>Cmd`.  
Parser functions resides in `cmd_parser.c`.  

Once a dispatched callback finish it's run, the application's return to the `cmd_prompt` function,   
ready of another input from the user.
 

## Available Commands



###Wlan commands:###  


* **Scan**: Allows user to retrieve scan results from network processor's (NWP) scan cache. 
Scan command supports two kinds of scans: 'One shot'  mode  and 'background scans', both subjected to a predefined policy.
1. One shot: If background scans **Aren't** set, the API call `sl_getNetworkEntries()` would result in return value  
of `SL_ERROR_WLAN_GET_NETWORK_LIST_EAGAIN` and the NWP triggers one scan only, in order to fill it's cache with results.  
Upon calling `sl_getNetworkEntries()` once again, user can now retrieve these results and scans stays shut.  

2. Background: These kind of scans are issued on a regular time bases in the background, according to the scan policy.  
If user has set background scans, (using **setpolicy** command), invoking the API: `sl_getNetworkEntries()` would return the results collected in the NWP's cache up to this point.

With CC3235 devices only, extended scan results are available which also include the country code and the AP supported channels in 2.4Ghz and 5Ghz (if available)
```
Usage: 
  scan [-help] [-n <number of APs to display>]
  
  -n        Maximum number of Scan results to show
  EXT       Extended scan results (supported only with CC3235 devices)
  -help     Display this help.
```
> **Note**: If no policy is set, **scan** command issue One-shot scan according to system persistent settings.

Once **scan** command has successfully completed, available devices would be printed to UART console.  

Example:
```
scan -n 30
```

----------
* **setpolicy**: Allows user to define the device's scan behavior and start background scans.   
Setting the scan policy is achieved by configuring the following parameters:
1. Scan interval: times between consecutive scans.
2. Hidden scan: If this option is set, scan results would include hidden SSIDs as well.
3. Enabled Channels: This option uses HEX bitmask to determine the enabled scan channels.
4. RSSI threshold: By setting an RSSI threshold, users can eliminate results with poor signal quality.
5. Turn background scans off: This option would shut background scans immediately. 

```
Usage: 
  setpolicy [-help] [-i <Scan interval in Sec>] [-h <hidden scan>]
            [-c <Channel Mask in HEX>] [-r <RSSI threshold>] [-o <Turn scans off>]
            
  -i      Sets scan interval in seconds (default is 10 seconds)
  -h      Scan for hidden AP as well (hidden scan = [YES, NO], default is NO)
  -c      Sets specific scan channels for 2.4Ghz, using HEX base bitmask. (default is `0x1FFF` - all enabled)
  -d      Sets specific scan channels for 5Ghz, using HEX base bitmask (if bit i is set, scan for the channel at index i according to 5Ghz channels bits order which the device support. E.g 36->0001, 40->0010, 44->0100.. etc). (default is `1F7FFFF` - all enabled)
  -r      Sets RSSI threshold for scan in dBm units (range: [-95,0] default: `-95`)
  -o      Turn scans off (YES = scans running in background would stop immediately)
  -help   Show Help regarding the setpolicy command.
```
> **Note**: Once 'setpolicy' command has been executed, background scans would start immediately,  
according to provided parameters. If no parameter is provided to this command, It would be using  
it's default values.

> **Note**: calling 'setpolicy' command with `-o YES` parameter, would shut down scans  
and would not set any other parameters given to this command. 

After setting policy, calling **Scan** command would retrieve results from NWP cache, according to set settings.  
Please note that the NWP cache has ageing time, so older retrieved results could violate the new policy.  

Example:
```
setpolicy -i 30 -c 1D5 -r -48
```
----------
* **wlanconnect**: Allows user to connect to an AP. 
Connection is based on the SSID of the AP,   
and receives the following parameters:
1. SSID: The requested SSID of the AP. Application expects this parameter to be inside quotation marks (" /"). 
2. Security Type: Device supports four modes: `[OPEN, WEP, WPA, WPA2, WPA2_PLUS, WPA3, WPS]`.  
3. Password: If needed, application expects this parameter to appear inside quotation marks ("/").
4. Static IP: User can decide to configure a static IP address. Application expect this parameter in dotted decimal format.
5. Gateway: User can set it's gw address statically. Unless provided, default parameter is set (IP address netmasked with 24).
6. DNS: User can also set it's dns address. Unless provided, default parameter is the gw address.
7. Enterprise credentials: Enterprise user name needed in order to log in an enterprise network. 

```
Usage: 
  wlanconnect [-help] [-s <"ssid name">] [-t <security type>] [-p <"password">]
  [-ip <static ip>] [-gw <static gw>] [-dns <static dns>] [-ent <"ent_username">]
  
    -s      SSID
    -t      Type of security (security type = [OPEN, WEP, WPA, WPA2, WPA2_PLUS, WPA3, WPS])
    -p      Password in ascii character (pin in case of WPS)
    -ip     Static IP in '.' format for ipv4 and in ':' for ipv6
    -gw     Static Gateway (relevant in case of using a static IP)
    -dns    Static DNS (relevant in case of using a static IP)
    -ent    Enterprise user name credentials
```
* Expected Host events:  
In connection process to an AP, some async events are expected from the NWP's side.  
These are the following:
1. `SL_WLAN_EVENT_CONNECT`: This event signals that the NWP is connected to an AP. It also prints details  
about the connected AP, such as SSID and BSSID. Please Refer to `SimpleLinkWlanEventHandler()` implemented  
in `network_terminal.c` for more info.

2. `SL_DEVICE_EVENT_DROPPED_NETAPP_IPACQUIRED` : This event signals that the device has an IP address.  
Event type is the same in both dynamic or static IP settings. Also, this event print useful information  
such as the leased IP, and the designated gateway. Please refer to `SimpleLinkNetAppEventHandler()`  
implemented in `network_terminal.c` for more info.

3. `SL_WLAN_EVENT_DISCONNECT` : In case the device is already connected to an AP, before attempting to connect with the requested AP, the device would disconnect from the old one. Disconnect reason code would than be: `SL_WLAN_DISCONNECT_USER_INITIATED`. Please refer to `SimpleLinkWlanEventHandler()` implemented  
in `network_terminal.c` for more info.  

> **Note**: In this version, the only cypher used for enterprise Wlan connection is MSCHAPv2.
Cypher option would be added in future releases.  

Example:
```
wlanconnect -s "myhome network" -t WPA2 -p "password" -ip 10.123.45.11
```
----------
* **wlan_ap_start**: Configure the device to operate in AP mode.  
This command also allows the user to set other AP related settings:

```
Usage: 
	wlan_ap_start [-help] [-s <"ssid name">] [-t <security type>] 
	[-p <"password">][-h <hidden AP>][-txp <Tx power [0-15]>]
	[-c <Wlan channel [1-13]>] [-l <STA connection limit [1-4]>]
		  
	-s      SSID
	-t      Type of security (security type = [OPEN, WPA/WPA2, WEP, WPS])
	-p      Password in ascii character (pin in case of WPS)
	-h      Start the AP in hidden mode (hidden mode = [YES, NO], Default is not hidden)
	-txp    Set Wlan Tx power (0 = Max Power, Default is Max)
	-c      Set channel for the AP (Default is channel 1)
	-l      Limit the number of connected stations
	-help   Display this help.
```

* Expected Host events:  
In the process of starting the device in AP mode, some async events are expected from the NWP's side.  
These are the following:

1. `SL_DEVICE_EVENT_DROPPED_NETAPP_IPACQUIRED` : This event signals that the device has an IP address.  
This essentially states that device has started in AP mode, since it sets it's own IP address.   
Also, this event print the leased IP, and the designated gateway (normally they're the same).  
Please refer to `SimpleLinkNetAppEventHandler()` implemented in `network_terminal.c` for more info.
  
2. `SL_WLAN_EVENT_STA_ADDED` : In case a station is connected to the device, this event is prompt.
It than prints to the UART monitor the BSSID of the added station. Please refer to `SimpleLinkNetAppEventHandler()` implemented in `network_terminal.c` for more info.

3. `SL_WLAN_EVENT_STA_REMOVED` : This event states that a connected station has disconnected.  
the BSSID of the disconnected station would also be displayed. Please refer to `SimpleLinkNetAppEventHandler()` implemented in `network_terminal.c` for more info.

> **Note**: As mentioned in wlanconnect command, SSID and password parameters are expected to appear inside quotation marks.

> **Note**: Invoking 'wlan_ap_start' when device is in p2p/station mode, would cause an NWP reset, and any ongoing connection would be disconnected.

Example:
```
wlan_ap_start -s "mySimpleLink" -t OPEN -l 1 -c 11 -txp 0 
```
----------
* **CoexEnable**: This command enables the coex feature with single antennna, Input = Pad 0 (GPIO 50) and Output = Pad 4 (GPIO 59). This command doesn't require any parameters.
* **CoexDisable**: This command disables the coex feature. This command doesn't require any parameters.
* **AntSelectionEnable**: This command enables the antena selection feature with auto mode, Antenna 1 = Pad 16 (GPIO 7) and Antenna 2 = Pad 17 (GPIO 8). This command doesn't require any parameters.
* **AntSelectionDisable**: This command disables feature. This command doesn't require any parameters.

----------

* **createfilter**: This command allows user to create an RX filter.  
RX filters are a set of rules and actions which imposed on each packet received from the air. When creating  an RX filter,  
user must determine what kind of rule this specific filter would enforce and upon a match for the criteria set by this rule, what action should the filter perform.

* Possible actions when filter rule is matched:
1. Drop: The packet which met the rule set by the filter, would be discarded.
2. Pass: No action would be taken - this option is necessary in case of parent filter.
3. Host event: Upon action, a user event would be triggered by the host driver.

* Possible rule kinds:

1. S_MAC: source mac, meaning set the match criteria according to packet's source mac address.
2. D_MAC: Destination mac address - following the same logic - match criteria is destination mac address.
3. BSSID: This allows the user to filter packets according to the packet's BSSID field.
4. S_IP: Source Ip, meaning set the match criteria according to the packet source IP address.
5. D_IP: Destination Ip, meaning set the match criteria according to the packet destination IP address.
6. FRAME_TYPE: Set match rule according to frame types: Management, Control or Data.
7. FRAME_SUBTYPE: Set match rule according to frame subtype field in packet header.
8. PATTERN: Set match rule according to specific pattern in packet's payload.  

Moreover, aside from rule and action, filters must have two more important settings, which are 'Mode' and  'comparison type'.  
By mode, we let the filter know in which connection configuration it should operate and by comparison type we let the filter know what criteria the rule is being checked against.

* Possible Modes:
1. L4: Filter applies when device is in station mode.
2. L4_CON: Filter applies when device is in station mode, but also connected to an AP.
3. L1: Filter applies when device is in transceiver mode.

* Possible comparison types:

1. Equals: Does the field in packet header equals to the set rule? If so, we have rule match.
2. Not Equals: Does the field in packet header not equal to the set rule? If so, we have a rule match.

Once a filter has it's set of rules, actions, mode and comparison type configured, it is ready to be enabled.
There's one more thing 'create filter' allows a user to do and that's to combine filters in an hierarchy structure, specifically in a parent and son configuration. That issue would be covered in the parent filter section below.

```
Usage: 
  createfilter[-help] [-f <filter Type>] [-v <Compare value>] [-e <Compare type>]  
  [-a <Action type>] [-m <Mode>] [-o <Offset>] [-i <Parent filter ID>]
  
        -f      Filter Types:
                [S_MAC, D_MAC, BSSID, S_IP, D_IP, FRAME_TYPE, FRAME_SUBTYPE, PATTERN]
                [S - Source, D - Destination]
                [MAC - MAC address, Value: xx:xx:xx:xx:xx:xx]
                [BSSID - MAC address OF THE AP, Value: xx:xx:xx:xx:xx:xx]
                [IP - IP Address, Values: IPv4 - xxx.xxx.xxx.xxx, IPv6 - xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx]
                [FRAME_TYPE Values: management, control, data]

        -v      Comparison value used by the filter (sets the filter match criteria)
        -e      Compare type (Compare type = [equals, not equals])
        -a      Action on match (Action type = [event, drop, pass]), Either async event, drop the packet or let it pass
        -m      Filter Mode (Mode = [L4_CON, L4, L1])
                L4_CON - L4 connected to AP, L4 - L4 not connected to AP and  L1 - transceiver mode
        -o      Offset from L1 / L4 Header, according to chosen 'Mode' field (Valid only for 'PATTERN' filter Type)
        -i      Parent filter for a leaf filter, otherwise: 0
```

* Parent filter:  
Filters can be combined to define a more intricate set of rules. For example, I'm interested in triggering a host event (action), every time I receive a packet from a specific mac address (rule), and not just any packet, I'm specifically interested in packets which contain certain pattern (0xCAFE) within their payload.

How to achieve this?  
The answer is simple : We define two different filters, in a 'parent - son' configuration:  
* First one, is a source mac (S_MAC) based filter. It's important to have this filter's action to be 'pass' which is a null action, since this going to be a parent filter - action is only defined for the 'son' filter.   

Example:
```
createfilter -f S_MAC -v XX:XX:XX:XX:XX:XX -e equals -a pass -m L4 -i 0

Filter Created successfully, filter's ID = 4
```
A message stating the filter was created successfully and it's ID, should appear on screen. 

* Second one, is a pattern based filter, specifically designed to catch our (0xCAFE) pattern, which resides in offset of 20 bytes within the packet payload. We must also state which filter would be the parent for this filter:  

Example:
```
createfilter -f PATTERN -v CAFE -e equals -a event -m L4 -o 20 -i 4

Please enter user ID, in range: [0,63] (needed for Host event action).
user@CC32xx:
```
Since the action chosen is 'event', application would prompt you for host event ID.
> Host event ID is used to indicate which filter has triggered the event. Elaborate explenation would follow.

```
user@CC32xx:60
 Filter Created successfully, filters ID = 5
```

Finally, by setting the option `-i 4`, we've combined the two filters, such that each packet is checked first by the first filter, and upon match it would be checked against our second filter. 

* Expected Host events:  
For filter which triggers host event, we expect an async event from host driver. This event bears the information regarding which filter has triggered and user can write he's / hers RX filter match handler there.

1. `SL_WLAN_EVENT_RXFILTER`: This event prints the user Action ID bitmap. If bit `i` is set,  
host event ID number `i` has been triggered. 

> **Note**: Rx filters can have more advanced settings and filter combinations. Also, there are some limitations on which kind of filter can be a parent filter for another one. It's highly advised to look at the RX filters section on programmers manual, before incorperating RX filters in your application.

> **Note**: Rx filters are **NOT** supported for AP mode.

> **Note**: 'createfilter' only creates a filter. In order to enable the filters, look at **enablefilter**.

Example:
```
createfilter -f D_IP -v 10.123.45.1 -e not equals -a drop -m L4_CON -i 0   
```
----------
* **enablefilter**: This command enables all the filter defined up to this point at once.

```
Usage: 
  enablefilter [-help] 
 
  -help     Display this help.
```
> **Note**: TI recommends enable all filters at once, since enabling each filter at a time, could cause undefined behaviour in case we're enabling combined filters / filters with parent.

Once **enablefilter** command has successfully completed, All filters are operational.  

Example:
```
user@CC32xx:enablefilter
```
----------
* **disablefilter**: This command disables all the filter defined up to this point at once.

```
Usage: 
  disablefilter [-help] 
 
  -help     Display this help.
```
> **Note**: TI recommends disable all filters at once, since disabling each filter at a time, could cause undefined behaviour in case we're disabling combined filters / filters with parent.

Once **disablefilter** command has successfully completed, All filters are shut.  

Example:
```
user@CC32xx:disablefilter
```
----------
* **deletefilter**: This command deletes all the filter defined up to this point at once.

```
Usage: 
  deletefilter [-help] 
 
  -help     Display this help.
```
> **Note**: TI recommends delete all filters at once, since deleting each filter at a time, could cause undefined behaviour in case we're deleting combined filters / filters with parent.

Once **deletefilter** command has successfully completed, All filters are deleted.  

Example:
```
user@CC32xx:deletefilter
```
----------
* **enablewowlan**: Enable Wake On Wireless Lan (Wowlan) command defines a pattern based filter,  
then sends host MCU to Low Power Deep Sleep (LPDS).  
Host MCU would wake up from LPDS once the pattern filter would trigger, at which point the NWP would wake the host MCU from LPDS, using host IRQ as a wake up source.

```
Usage: 
        enablewowlan[-help] [-v <"pattern value">] [-u <Action ID>] [-i <pattern offset>]
        
    -v      Pattern Value used by the Filter (string, of less than 16 characters)
    -u      WoWLAN filter Action ID, should be less than 255
    -i      Pattern offset in payload default 0
    -help   Display this help
```

Example:
```
enablewowlan -v Wake_up_str -u 5 -i 0
```
----------
* **p2pstart**: This command sets the NWP in peer to peer mode and discover, connects to a p2p device.  
'p2pstart' shows the device name, and makes it discoverable by other p2p device.

* Expected Host events:  
In connection process to another p2p device, some async events are expected from the NWP's side.  
These are the following:
1. `SL_WLAN_EVENT_P2P_DEVFOUND`: This event signals that a remote peer to peer device has been found. From this point on, the device is waiting for negotiation request. Please Refer to `SimpleLinkWlanEventHandler()` implemented  
in `network_terminal.c` for more info.

2. `SL_WLAN_EVENT_P2P_REQUEST` : This event signals that the device got negotiation request. 
Here we save the remote p2p device SSID, in order to manually connect with it later.  
Please refer to `SimpleLinkNetAppEventHandler()` implemented in `network_terminal.c` for more info.

3. `SL_WLAN_EVENT_P2P_CONNECT` : This event signify that device has successfully connected to a remote peer to peer device. At this event handler, we print the remote's BSSID. Please Refer to `SimpleLinkWlanEventHandler()` implemented in `network_terminal.c` for more info.  

4. `SL_DEVICE_EVENT_DROPPED_NETAPP_IPACQUIRED` : This event signals that the device has an IP address.  
Event type is the same as in STA/AP mode. Also, this event print useful information  
such as the leased IP, and the designated gateway. Please refer to `SimpleLinkNetAppEventHandler()`  
implemented in `network_terminal.c` for more info.

5. `SL_WLAN_EVENT_P2P_DISCONNECT` : Once the remote p2p device disconnects, this event is triggered.  
Please Refer to `SimpleLinkWlanEventHandler()` implemented in `network_terminal.c` for more info.  

> **Note**: At any point in the p2p connection process, user can type the command 'p2pstop' to exist p2p mode.

```
Usage: 
  p2pstart [-help] 
 
  -help     Show Help regarding the p2pstart command.
```
Example:
```
user@CC32xx:p2pstart
Starting NWP in P2P role..

Device name: mysimplelink

User can stop the P2P connection process by typing 'p2pstop' command.
```  
----------
* **Countrycode**: Allows user to set the AP country code. 
Valid input consist of two capital characters (i.e US) 

```
Usage:
        Countrycode -help  -g <Country code i.e US>

Description:
        Set device country code

        -a      Set the device country code.
        -help   Display this help


        Note:    Country code consist of two capital characters.
```



After setting country code, device will restart to apply changes  

Example:
```
Countrycode -g US
```
----------

###Socket commands:### 

* **send**: This command demonstrate sending data in packets using networking API. 

This command allows user to open a udp or tcp sockets, and send packets in various configurations, which include the following:

1. Run as server or client.
2. Protocol: tcp or udp.
3. Port number.
4. Blocking vs. Non-blocking option.
5. Number of packet to transmit.
6. Use Ipv6 rather than Ipv4 (Default).

```
Usage:
        send [-help] [-c <server ip address>] [-u] [-p <port number>]  [-nb] [-n <number of packets>]  
        send [-help] [-s] [-u] [-p <port number>]  [-V] [-nb] [-n <number of packets>]
        
        -c      Run in client mode and connect to mentioned server - IP should be in '.' format for ipv4 and in ':' for ipv6
        -s      Run in server mode - (Note that for client, a server ip address is required.)
        -u      Use UDP rather than TCP
        -p      Port number to send/receive data (Default is 5001)
        -nb     Create non-blocking socket rather than blocking
        -n      Number of packets to transmit (Default is 1000)
        -V      Use IPv6 rather than IPv4
```

Once all packets have been sent or timeout expired (60 seconds), application would return to main menu.  
> **Note**: Make sure the server side is listening for incoming connections before sending packets as client.

Example:
```
send -c 10.123.45.6 -p 5050 -n 2000
```
----------
* **recv**: This command demonstrate reception of data in packets using networking API. 

This command allows user to open a udp or tcp sockets and receive packets in various configurations, which include the following:

1. Run as server or client mode.
2. Protocol: tcp or udp.
3. Port number.
4. Blocking vs. Non-blocking option.
5. Number of packet to transmit.
6. Use Ipv6 rather than Ipv4 (Default).

```
Usage:
        recv [-help] [-c <server ip address>] [-u] [-p <port number>]  [-nb] [-n <number of packets>]  
        recv [-help] [-s] [-u] [-p <port number>]  [-V] [-nb] [-n <number of packets>]
        
        -c      Run in client mode and connect to mentioned server - IP should be in '.' format for ipv4 and in ':' for ipv6
        -s      Run in server mode - (Note that for client, a server ip address is required.)
        -u      Use UDP rather than TCP
        -p      Port number to send/receive data (Default is 5001)
        -nb     Create non-blocking socket rather than blocking
        -n      Number of packets to transmit (Default is 1000)
        -V      Use IPv6 rather than IPv4
```

Once all packets have been received or timeout expired (60 seconds), application would return to main menu.  

Example:
```
recv -s -u -n 1000 
```

###Netapp commands:### 

* **ping**: Lets user run ping command, usually for connectivity tests purpose.  
Ping can be issued according to the ping parameters stated below. For each successful 'Echo reply' packet received, statistics including round-trip times,number of received packets and more. would be printed to user UART monitor.

```
Usage: 
  ping [-help] [-h <host name or ip>] [-c <Num of packets>]  
  [-s <data size>] [-i <delay interval>] [-t <timeout>]
  
    -h      Host name or ip (In IPv4 '.' format)
    -c      Send <Num of packets> number of ECHO_REQUEST packets (Default is 10)
    -s      Specify the number of data bytes to be sent (Default is 56)
    -i      Wait <delay interval> time (in seconds) between sending each packet (Default is 1)
    -t      Specify a timeout (in seconds) before ping exits, (default is 3 seconds per ping packet)
```
> **Note**: Application is blocked until ping timeout or ping ends successfully.

Example:
```
ping -h www.ti.com -c 30 -s 112 -t 10
```

----------
* **mdnsadvertise**: Lets user advertise a service over mDNS. The new service would be advertised according to the command parameters:
```
Usage: 
    mdnsadvertise [-help] [-n <Device name>] [-st <Service type>] [-so <Service over>]
    [-ttl <Service TTL>] [-t <"Adv. Text">] [-p <Service port>]
    
    -n      Sets Device name
    -st     Sets service type
    -so     Specify L4 protocol over which the service runs
    -ttl    Sets the time to stop advertising [seconds]
    -t      Additional text to be sent for query response
    -p      Specify port over which the service runs
    -help   Display this help
```
Requested service would be advertised as: 'ServiceName._serviceType._serviceOver._local

> **Note**: mDNS services are not persistent, i.e any advertised service would be lost after NWP reset, with one exception: the internal http server of the NWP. 

Example:
```
user@CC32xx:mdnsadvertise -n PC1 -so tcp -st ipp -ttl 3000 -p 5001
Setting service name: PC1
[mDNS advertise] : MDNS service started successfully
[mDNS advertise] : MDNS service name PC1._ipp._tcp.local registered successfully.
```

----------
* **mdnsquery**: Lets user run mDNS query for services over the local LAN.  
mDNS query command supports two types of query:
1. One-Shot: The NWP issues one query, wait for the first result that comes up and exists.
2. Continues: The NWP issue several queries, and store these results in the NWP cache.

```
Usage: 
    mdnsquery [-help] [-n <Device name>] [-st <Service type>] [-so <Service over>] [-o <Oneshot>]
   
    -n      Device name for query, should be character string (optional)
    -st     Service Type for query, should be character string (mandatory)
    -so     Set L4 protocol over which the service operates, should be character string (mandatory)
    -o      OneShot/Continues (Oneshot = [YES, NO (default)])
    -help   Display this help
```
User can query for a partial service name, or full. Meaning, query can be issued with partial service name.  
For example,

```
mdnsquery -st ipp -so tcp
```
would query for **any** server who offers internet printer protocol service over tcp, whereas
```
mdnsquery -n PC1 -st ipp -so tcp
```  
would query for the specific server named "PC1" who offers such service.
On successful response, the service name and server's port found would be printed to the screen.
### Transceiver commands:  


In order to start the device in transceiver mode, user can type the command: 'radiotool' to the UART terminal.  
radiotool command lets the user run several radio related tests, for both RX and TX operations.  

NOTE: 5Ghz channels are supported only by the CC3235 device. The rates in 5Ghz are 11a only reates.  

 Once NWP has entered the device to transceiver mode, the following menu is printed:  
 >```
 >      ------------------------------------------------
 >                    RadioTool Option Menu
 >       ------------------------------------------------
 >         |            Rx task options.            |
 >         |            Tx task options.            |
 >         |            Exit radio tool.            |
 >       ------------------------------------------------
 >```
 * Rx task options:  
 This option provides the user with an interface to the NWP radio. It allows the user to open the RX path of the radio on a specific channel for a pre-defined amount of time. When said time finishes, statistics regarding received packets would be printed, according to user preferences.

```
        ------------------------------------------------
                     RX Task Option Menu
        ------------------------------------------------
        Choose Rx channel: 2.4Ghz [1,13] 5Ghz [36,165]
        Enter Rx duration (in mSec units): [1,UINT32_MAX]
        Show Rx statistics? [Yes, No]
```
Once all Rx task parameters have been provided, the application is waiting for a key press, to give the user time to set up radio testing equipment:
```
Ready to start RX, press any key to continue...
```
If Rx statistics are configured, once RX task is finished a rate and RSSI histogram for received packets would appear:
```
*********************************Rx Statistics**********************************
Received Packets - 226
Average Rssi for management: -70  Average Rssi for other packets: -77
    -----------------------   RSSI Histogram   -----------------------
-40dBm to -87dBm (below and above RSSI will appear in the first and last cells]
           10   20   30   40   50   60   70   80   90   100
         |----+----+----+----+----+----+----+----+----+----|
  >-40dBm                                                  (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
  -49dBm *                                                 (3.54%)
         |----+----+----+----+----+----+----+----+----+----|
  -58dBm *****                                             (11.06%)
         |----+----+----+----+----+----+----+----+----+----|
  -68dBm ***************************                       (55.75%)
         |----+----+----+----+----+----+----+----+----+----|
  -77dBm ***                                               (6.19%)
         |----+----+----+----+----+----+----+----+----+----|
<-87dBm  ***********                                       (23.45%)
         |----+----+----+----+----+----+----+----+----+----|
              10   20   30   40   50   60   70   80   90   100

    -----------------------   Rate Histogram   -----------------------

           10   20   30   40   50   60   70   80   90   100
         |----+----+----+----+----+----+----+----+----+----|
0        *************************************************  (99.56%)
         |----+----+----+----+----+----+----+----+----+----|
1                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
2                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
3                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
4                                                            (0.44%)
         |----+----+----+----+----+----+----+----+----+----|
5                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
6                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
7                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
8                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
9                                                            (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
10                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
11                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
12                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
13                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
14                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
15                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
16                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
17                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
18                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
19                                                           (0.00%)
         |----+----+----+----+----+----+----+----+----+----|
              10   20   30   40   50   60   70   80   90   100
                     The data was sampled during 162109mSec
*******************************End Rx Statistics********************************
 Press any key to continue...
```
* Tx task Options:  
 This option provides the user with an interface to the NWP radio. It allows the user to open the TX path of the radio in one of three modes:
```
        ------------------------------------------------
                     TX Task Option Menu
        ------------------------------------------------
        Choose Tx task Mode:
        1. Continuous:  This mode TX frames generated internally by NWP, for a given amount time.
        2. Packetized: This mode sends each TX frame from host processor to the NWP TX queue.
        3. CW Mode:    TX carrier wave signal. User can set CW tone offset.
```
For each mode, user is required to provide additional parameters:

	**Continuous Mode**: Let's the user use the internal packet generator of the NWP.

This mode expects the following parameters:

	1. Destination MAC address that would appear in generated packet's header.
	2. Packet size.
	3. Tx channel.
	4. Packet's payload pattern. Can be either:
		* All zeros.
		* All ones.
		* Incremental numbers.
		* Decremental numbers.
	5. Tx duration.

Once all the above are provided to the application, user is prompt to enter PHY parameters.  
User can choose between enter the parameters, or use defaults. for more information, refer to Radio parameters section in this document.  


	**Packetized Mode**: Transmitting fixed amount of packets, generated by Host processor.  
Each packet is sent one at a time from the Host processor to the NWP.  
  
This mode expects the following parameters:

	1. Destination MAC address that would appear in generated packet's header.
	2. Packet size.
	3. Tx channel.
	4. Packet's payload pattern. Can be either:
		* All zeros.
		* All ones.
		* Incremental numbers.
		* Decremental numbers.
	5. Amount of packets to transmit.

Once all the above are provided to the application, user is prompt to enter PHY parameters.  
User can choose between enter the parameters, or use defaults. for more information, refer to Radio parameters section in this document.

	**Carrier Wave (CW) Mode**: In this mode the device transmits an un-modulated RF tone. 
```
Choose Tx channel: 2.4Ghz [1,13] 5Ghz [36,165] - This option determine the center frequency for the Carrier wave.
Tone offset: Specify desired Tone offset from center frequency. Value is in multiples of 312.5kHz In range: [-25,25].
```
Tone offset option can be explained by the following rule: for a tone offset of N, (N is a whole number in range [-25, 25] ) 
> Carrier frequency = center frequency + N*312.5kHz.
```
Enter TX duration: [1,UINT_MAX] (in mSec)
```
Now, user is presented with key press screen to start the test. When the Tx duration time elapsed, application returns to the main radio tool option menu.  

----------
* **Radio parameters**:  
For both packetized and continuous modes, user need to provide radio settings, or use default.  
These are the default parameters:
```
Default Values:

        Preamble type:  Long.
        Rate:  1Mbps.
        Tx Power:  Max Power.
        CCA override:  Yes.
        CCA threshold:  Not used.
```
Or, if needed, user can input these parameters:

* Preamble:  
```
Specify desired Preamble type. In range: [0,4]. Available modes:

        0. Long preamble.
        1. Short preamble.
        2. OFDM mode.
        3. Mixed mode.
        4. Greenfield mode.
```
> **Note**: For OFDM rates, OFDM Preamble would be selected automatically.

* Rate: 
```
Specify desired Rate, or Modulation/Coding. In range: [1,21].
        1.  1 Mbps (DSSS)
        2.  2 Mbps (DSSS)
        3.  5 Mbps (CCK)
        4.  11 Mbps (CCK)
        5.  Deprecated.
        6.  6 Mbps. (OFDM)
        7.  9 Mbps. (OFDM)
        8.  12 Mbps. (OFDM)
        9.  18 Mbps. (OFDM)
        10. 24 Mbps. (OFDM)
        11. 36 Mbps. (OFDM)
        12. 48 Mbps. (OFDM)
        13. 54 Mbps. (OFDM)
        14. MCS0 : Modulation - BPSK,  Code rate - 1/2.
        15. MCS1 : Modulation - QPSK,  Code rate - 1/2.
        16. MCS2 : Modulation - QPSK,  Code rate - 3/4.
        17. MCS3 : Modulation - 16QAM, Code rate - 1/2.
        18. MCS4 : Modulation - 16QAM, Code rate - 3/4.
        19. MCS5 : Modulation - 64QAM, Code rate - 2/3.
        20. MCS6 : Modulation - 64QAM, Code rate - 3/4.
        21. MCS7 : Modulation - 64QAM, Code rate - 5/6.
```
> **Note**: It's possible to choose MCS or rate.

* Tx power: 0 being the maximum power and 15 being the minimum power.
```
Specify desired Tx Power, as dB offset from max power (0 indicates MAX power) In range: [0,15]:
```
> **Note**: The TX power level numbers quoted in the datasheet are at the device RF pin.

* Override CCA: Lets user set threshold/override Clear Channel Assessment mechanism (CCA).  
```
        CCA threshold determine the threshold in which the channel is considered as occupied.
        The following can be set:

        1. Min:           (-88dBm)
        2. Low:           (-78dBm)
        3. Default:       (-68dBm)
        4. Med:           (-58dBm)
        5. High:          (-48dBm)
        6. Max:           (-38dBm)
```

> **Note**:  override is recommended if the wireless environment is too congested in order to have a reliable periodic transmission.

## References
* For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)

