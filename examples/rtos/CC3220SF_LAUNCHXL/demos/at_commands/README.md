## Example Summary

This example demonstrates the use of AT Commands to operates the various range of networking capabilities which the CC32xx/CC31xx device family provides.  
For details about the AT Commands and their parameters please refer to the AT Commands and attributes: [Commands and attributes](http://www.ti.com/lit/pdf/swru534)


## Example Usage

* Build the project and flash it by using the Uniflash tool for cc32xx,  
Or equivalently, run debug session on the IDE of your choice.
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
         ==============================================
            AT Commands Example Ver: 1.1.1
         ==============================================

         CHIP: 0x20000010
         MAC:  2.0.0.0
         PHY:  2.2.0.4
         NWP:  3.2.0.0
         ROM:  0
         HOST: 2.0.1.17
         MAC address: 08:00:28:5b:55:e0

         ==============================================
```
At this point, you will see the following line:
```
Enter AT Command:
```
indicating that you can enter AT command at the command prompt.
  
## AT Command Application Overview
  
This example is a command line interface (cli) based application, which is used to operate the CC32xx/CC31xx networking capabilities by using AT Commands. 

It does that by offering a list of propriety commands, fed to the application via UART and later parsed and dispatched.

The application starts with invoking API function: "ATCMD_create" which creates the ATCMD module by the following actions:
-	Creates the SimpleLink Host thread
-	Creates queue for ATCMD module events
-	Creates map list of all AT commands

Next, it creates the events thread which responsible for receiving events and command response from the ATCMD module by invoking the API function: "ATCMD_recv"

Lastly, it invokes the API function "ATCMD_send" which is responsible for sending commands to the ATCMD module.

If the typed command matches a table entry, the handler is dispatched and the parameters fed by the user are passed to the command callback.
Inside each command callback, the first action is to parse the user's input. This is achieved by calling the corresponding parser function. 

A structure containing each command's parameters is also passed to the designated parser and returned filled with the parsed user's parameters.

Later, the parsed structure is handled by the callback.

Once a dispatched callback has completed, it returns a command response of OK or ERROR. 

The application then returns to the starting point and is ready for another command from the user.

for getting usage help for specific command put '?' instead of parameters ([AT Command name] = ?)

### Commands
Syntax:
```
    AT<command name>=<param1>, <param2>,  ,<paramX>
```

-	Command that contains parameters, should be includes equal sign (=) between the command name and the first parameter.
-	Command that contains parameters, should be includes comma mark (,) as delimiter between them - comma delimiters are mandatory!
-	In case the parameter defined as "ignore" or "optional", it could be left empty but the comma delimiter should be mentioned - it looks like two conjunction delimiters (,,).
-	Parameter that left empty be treated as 0 or NULL (according to parameter type) and in case it was not defined as "ignore" or "optional" - error should be raise.
-	String parameter contains spaces should be enclosed with quotes (" ")
-	String parameter contains comma delimiter (,) should be enclosed with quotes (" ")
-	Numeric value parameter could be on of the following:

    *    Decimal

    *	 Hexadecimal - should be with prefix of zero x notation (0x)

-	Numeric array parameter could be enclosed with square brackets ([ ]).
-	Numeric array parameter could be one of the following:

    *	IPv4  address - contains 4 numeric values (8 bits each) with point mark (.)  as delimiter between them enclosed with or without square bracket - X.X.X.X or [X.X.X.X]
    
    *	IPv6 address -  contains 4 numeric values (32 bit each) with colon mark (:)  as delimiter between them enclosed with or without square bracket - X:X:X:X or [X:X:X:X]
    
    *	MAC address - contains 6 numeric values (8 bit each) with colon mark (:)  as delimiter between them enclosed with or without square bracket - X:X:X:X:X:X or [X:X:X:X:X:X]
    
-	Bit mask parameter should contains values with vertical bar ( | ) as delimiter between them enclosed with or without square bracket - X|X|X or [X|X|X]
-	AT command handler allows for the AT commands to be entered in upper or lower case and with spaces between the arguments.
-	Data parameter should be one of the following formats:

    *	Binary format
    
    *	Base64 format - binary to text encoding

### Command return status
Command return status could be one of the following cases:

*	Command that return values:
```
    <command name >: <value1>, ,<valueX>
```
*	Command that return success:
```
    OK
```
*	Command that return failure:
```
    ERROR:<error description>, <error code>
```

-	Command return status should include colon mark (:) between the command name and the first value.
-	Command return status that contains list values should include semicolon mark (;) as delimiter between the list members.

### Asynchronous event
The events may arrive at any time.
They will always be built in the following format:
```
    <event name>: <event ID>,<value1>, ,<valueX>
```

*	Event should include colon mark (:) between the event name and the event ID.
 
Please install the latest CC32xx Service Pack in order to get the most optimization and lowest current consumption.

## References

* **For full list of commands and attributes, please refer to [Commands and attributes](http://www.ti.com/lit/pdf/swru534)**
* For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)





