## Example Summary

This example introduce the MQTT Client Server library API and usage.

## Peripherals Exercised

* The board LEDs are used for status indication. To distinguish similar indications, the user needs to be aware of the executed procedure.   
The following table lists all options.

<table>
  <tr>
    <th>LED indication</th>
    <th>Led Color for CC3220</th>
    <th>Led Color for CC3235</th>
    <th>Interpretation</th>
  </tr>
  <tr>
    <td>Solidly on</td>
    <td>Green</td>
    <td>Green</td>
    <td>Indicate Simplelink is properly up - Every Reset / Initialize</td>
  </tr>
  <tr>
    <td>Blinking</td>
    <td>Red</td>
    <td>Blue</td>
    <td>Device is trying to connect to AP - Every Reset / Initialize</td>
  </tr>
  <tr>
    <td>Solidly off</td>
    <td>All</td>
    <td>All</td>
    <td>Device connected and working - Only after connection</td>
  </tr>
</table>

## Example Usage

* Access Point (AP) Configuration
	- AP information is set in 'network\_if.h' file.

The application have two MQTT entities  
MQTT Client Role - Can connect to remote broker

* In order to activate this role, ENABLE\_CLIENT must be defined
* Remote Broker Configuration
	- Broker parameters can be configured in Mqtt\_ClientCtx parameter which can be found in 'mqtt\_server\_app.c'
	- The broker parameters are:
		- Connection types and security options
			- IPv4 connection
			- IPv6 connection
			- URL connection
			- Secure connection
			- skip domain name verfication in secure connection
			- skip certificate catalog verfication in secture connection
		- Server Address: URL or IP
    	- Port number of MQTT server
    	- Method to tcp secured socket
    	- Cipher to tcp secured socket
    	- Number of files for secure transfer
    	- The secure Files 
    	
* Secured socket  
	In order to activate the secured example, SECURE\_CLIENT must be defined 'mqtt\_server\_app.c' file  ( certificates should be programmed ).

* Client Authentication  
	In order to activate the Client authentication by the server, CLNT\_USR\_PWD must be defined  ( ClientUsername and ClientPassword must be defined ). 
* Topics Configuration  
	- The topics can be set by changing the definitions in 'mqtt\_server\_app.c' file 
	- The subscription topics can be set in the **SUBSCRIPTION\_TOPICX** definitions
	- The Client is subscribe to the following default topic  
		**"/Broker/To/cc32xx"**  
	- The publish topic and data can be set in the **PUBLISH\_TOPICX** and **PUBLISH\_TOPICX\_DATA** definitions	  
	- The Client publish the following default topic "/cc32xx/ButtonPressEvtSw2" - 
				the topic will be published by pressing SW2 on the board

MQTT Server Role - Broker that is ready for external client connection

* In order to activate this role, ENABLE\_SERVER must be defined
* Local Broker Configuration
	- Broker parameters can be configured in Mqtt\_Server parameter which can be found in 'mqtt\_server\_app.c'
	- The broker parameters are:
    	- Port number of MQTT server
    	- Method to tcp secured socket
    	- Cipher to tcp secured socket
    	- Number of files for secure transfer
    	- The secure Files
* Secured socket 
	In order to activate the secured example, SECURE\_SERVER must be defined  ( certificates should be programmed ).
* Client Authentication
	In order to activate the Client authentication by the server, SRVR\_USR\_PWD must be defined  ( ClientUsername and ClientPassword must be defined ). 
* Topics Configuration
	- The topics can be set by changing the definitions in 'mqtt\_server\_app.c' file
	- The server can subscribe to a topic (loopback topic), this topic can be set in the **ENROLLED\_TOPIC** definition
	- The Server is subscribed to the following default topics 
		"/cc32xx/To/Broker"  
	- Each client connected to the Broker, can publish on that topic, and the internal client will forward that topic to the remote broker
		
		
Both MQTT roles have internal loopback to allow topic forwarding.	  
			
* Build the project and flash it by using the Uniflash tool for cc32xx, or equivalently, run debug session on the IDE of your choice.

* Open a serial port session (e.g. 'HyperTerminal','puTTY', 'Tera Term' etc.) to the appropriate COM port - listed as 'User UART'.  
The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

	The connection should have the following connection settings:

    	Baud-rate:    115200
	    Data bits:         8
	    Stop bits:         1
	    Parity:         None
	    Flow Control:   None


* Run the example by pressing the reset button or by running debug session through your IDE.  
 `Green LED` turns ON to indicate the Application initialization is complete 

* Once the application has completed it's initialization and the network processor is up,  
  the application banner would be displayed, showing version details:

        ============================================
           MQTT client server Example Ver: 1.1.1
        ============================================

         CHIP: 0x30000019
         MAC:  2.0.0.0
         PHY:  2.2.0.5
         NWP:  3.3.99.2
         ROM:  0
         HOST: 2.0.1.17
         MAC address: 04:a3:16:45:89:8e

        ============================================

* At this point `Board_LED0` will blink until the device will connect to the hard coded AP.  
	* In case connection to the hard coded SSID AP fails, user will be requested to fill the SSID of an open AP it wishes to connect to.  
	* If no AP is available or connection failed, the example reset and tries to connect again.
	* Once the connection success all LEDs turn off.

* Special handling
	- In case the internal client will disconnect (for any reason) from the remote broker, the MQTT won't be restarted.  
	The user can change that behavior by adding **gResetApplication = true** to *MQTT\_CLIENT\_DISCONNECT\_CB\_EVENT* case in 'Client\_server\_cbs.c' file.

## Application Design Details

* This example provides users the ability to work with both Client and Server (Broker) MQTT Entities by combine the two entities to one example, we also allow the user to provide a loop back between internal and external clients (using the enrolled topic)

	- Server/Broker
		Allows full Broker capabilities (up to 4 clients)  
		Clients can connect/subscribed/publish to the broker at any time
		
	- Client
		Allows full mqtt client abilities  
		The Client can connect to remote broker, subscribe and publish

	- Loopback
		The internal Client will be connected to the internal Broker, with enrolled topic (subscribed).  
		Any published data on that topic, that arrived to the broker, will be passed to the internal 
		client, and will be forward to the remote broker.  
		Any topic that the client is subscribed to at the remote broker, that was published, will be followed to the internal broker, and from there to all subscribed clients
		This will allow full forwarding of topics.


## References

[MQTT Org - MQTT Home page](http://mqtt.org/documentation)  
[MQTT v3.1.1 specification](http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html)  
[MQTT v3.1 specification](http://www.ibm.com/developerworks/webservices/library/ws-mqtt/index.html)  

For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)
