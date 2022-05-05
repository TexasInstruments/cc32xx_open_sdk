## Example Summary

This application demonstrates how to perform an HTTP GET request.

## Example Usage

* Configure the AP parameters in  platform.c:

 	\#define SSID_NAME                          
         
	\#define SECURITY_TYPE                        

	\#define SECURITY_KEY   

* Configure the HTTP parameters in httpget.c
	
	\#define HOSTNAME              "http://www.example.com"
	
	\#define REQUEST_URI           "/"
	
	\#define USER_AGENT            "HTTPClient (ARM; TI-RTOS)"

                       

* Build the project, flash it by using the Uniflash tool for cc32xx,  
Or equivalently, run debug session on the IDE of your choice.

* Open a serial port session (e.g. 'HyperTerminal','puTTY', 'Tera Term' etc.) to the appropriate COM port -   
listed as 'User UART'.
The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

The connection should have the following connection settings:

    Baud-rate:    115200
    Data bits:         8
    Stop bits:         1
    Parity:         None
    Flow Control:   None
		

* Run the example by pressing the reset button or by running debug session through your IDE.  

* The example then makes an HTTP GET call to "www.example.com" and prints
the HTTP response status and the number of bytes of data received.

## Application Design Details

* This application uses a task for HTTP communication:

``httpTask``  - is waiting for ip to be acquired and then creates a connection to the HTTP    server. 
          When a connection is
          established, the application makes an HTTP GET method call using the request URI. The
          response status code, header fields and body from the HTTP server are
          processed to get response status code and data. The connection is
          closed and resources are freed before the task exits.

## References
For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)

