## Example Summary

This example demonstrates the use of the JSON library.

The app enables the user to use all the JSON parsing and building APIs.


## Example Usage

* Run the Uniflash tool for cc32xx, navigate into ` Files -> User Files` and add the Json template and Json file see example [Json file and Json template examples](#Json-file-and-Json-template-examples) using the `Add File` button.
* Change the `JSON_FILENAME` and `TEMPLATE_FILENAME` #define  in 'json\_app.c' according to the added files


> \#define TEMPLATE_FILENAME                       "template1"


> \#define JSON_FILENAME                           "json1" 


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

Once the application has complete it's initialization and the network processor is up,  
the application banner would be displayed, showing version details:

     ==============================================
        JSON Parser & Builder Example Ver: 1.0.0
     ==============================================

     CHIP: 0x20000010
     MAC:  2.0.0.0
     PHY:  2.2.0.4
     NWP:  3.3.0.0
     ROM:  0
     HOST: 2.0.1.15
     MAC address: 08:00:28:5b:55:e3

     ==============================================

* The example starts by showing on the terminal the following options:




		0.Create template object
		
		1.Create json object
		
		2.Parse
		
		3.Get value
		
		4.Set value
		
		5.Get array member count
		
		6.Build the json
		
		7.Destroy template object
		
		8.Destroy json object  
	


* Select the requested action by pressing the appropriate number

	* `Create template object` - Creates template object from the json file defined via the 
	  							`TEMPLATE_FILENAME` .
	* `Create json object` - Creates an empty json object according to the template object which was 	      				 previously created.    
		 Input:   
					1.  Json size - The maximum size which the user wants to allocate for the json. 
	* `Parse` - Parses the json file which was defined via the `JSON_FILENAME` .
	* `Get value` - Gives the the user the ability to retrieve data from the parsed json, by 		  				entering the key to the data.
		Input: 
		

		1.  Key - The key specifies which data the user wants to retrieve.  
        Example: the following json represents John's car collection.
					

					{    
					    "name":"John",   
					    "age":30,  
					    "cars": [  
					        { "name":"Ford", "models":[ "Fiesta", "Focus", "Mustang" ] },
					        { "name":"BMW", "models":[ "320", "X3", "X5" ] },
					        { "name":"Fiat", "models":[ "500", "Panda" ] }
					    ]
					 }
					

			Inorder to retrieve the "X3" from the above json the user should input : "cars".[1]."models".[1]   
			Inorder to retrieve 30 from the above json the user should input : "age" 
		2. Value type - The type of the value which the user requested  [0 - Int32, 1 - String / RAW , 2 - Boolean ].
		 
	* `Set value` - Gives the user the ability to set a value at a specific key, the value can be a 					new value or to replace an old one, The value type must be according to the 					template.   
		Input:
		1.  Key - The key specifies which data the user wants to set. (See the above example)
		2.  Value type - The type of value which the user will set.(See the above example)
		3.  Value - The value to be set.
 
 
	* `Get array member count` - Gives the user the ability to know the number of array members at a 							 specific key.   
		Input:
		1. Key - The key specifies which arrays member count is requested.   
        Example according to John's json:
			1. Key - "cars"  . Member count - 3
			2. Key - "cars".[2]."models" . Member count -  2  
	* `Build the json` - Builds the json back into a readable json string.
	* `Destroy template object` - Destroys the template object.
	* `Destroy json object` - Destroys the json object.

## Scenarios examples
The scenario examples are written in high level using the json lib APIs.
	
	 Create json from       | Parse the json and get | Parse the json, change
	 scratch and build it   | its value              | its value and build it
	 +--------------------+ | +--------------------+ | +--------------------+
	 |Json_createTemplate | | |Json_createTemplate | | |Json_createTemplate |
	 +--------------------+ | +--------------------+ | +--------------------+
	            |           |            |           |            |
	            v           |            v           |            v
	 +--------------------+ | +--------------------+ | +--------------------+
	 |Json_createObject   | | |Json_createObject   | | |Json_createObject   |
	 +--------------------+ | +--------------------+ | +--------------------+
	            |           |            |           |            |
	            v           |            v           |            v
	 +--------------------+ | +--------------------+ | +--------------------+
	 |Json_setValue       | | |Json_parse          | | |Json_parse          |
	 +--------------------+ | +--------------------+ | +--------------------+
	            |           |            |           |            |
	            v           |            v           |            v
	 +--------------------+ | +--------------------+ | +--------------------+
	 |Json_build          | | |Json_getValue       | | |Json_getValue       |
	 +--------------------+ | +--------------------+ | +--------------------+
	            |           |            |           |            |
	            v           |            v           |            v
	 +--------------------+ | +--------------------+ | +--------------------+
	 |Json_destroyObject  | | |Json_destroyObject  | | |Json_setValue       |
	 +--------------------+ | +--------------------+ | +--------------------+
	            |           |            |           |            |
	            v           |            v           |            v
	 +--------------------+ | +--------------------+ | +--------------------+
	 |Json_destroyTemplate| | |Json_destroyTemplate| | |Json_build          |
	 +--------------------+ | +--------------------+ | +--------------------+
	                        |                        |            |
	                        |                        |            v
	                        |                        | +--------------------+
	                        |                        | |Json_destroyObject  |
	                        |                        | +--------------------+
	                        |                        |            |
	                        |                        |            v
	                        |                        | +--------------------+
	                        |                        | |Json_destroyTemplate|
	                        |                        | +--------------------+     


## Application Design details

* Json app is supporting the following data types :
	1. int32 - signed int 32 bit 
	2. String - string  (Uart terminal is maximum 99 chars)
	3. RAW - Raw data type relates to a json object as a string instead of key:value pair. 
	4. Boolean - 1 - True , 0 - False
* Json object is dependent upon Template object, thus template object need to be created prior to the json object.
* Choosing the correct value type is important in order to set and get the value correctly.
* Adding elements into array beyond the specified in the template, can be done with a value type which specified in the last element of the array.
* If the json's top level entity is an array , the template should be created as an object named as "#", e.g:
	`json : 
	[
	"name": "Alex",
	"age" :  30
	],
	[
	"name" : "Mark",
	"age": 25
	]`

	`template:
	{ "#": [
	"name": string,
	"age" :  int32
	] }`
	
	this feature is supported only for parsing and not for building.
* Adding more data into the json is limited by the json object size.
* Json default object size is 1024 bytes.



## Json file and Json template examples

###Json template example
	{
	  "firstName": string,
	  "lastName": string,
	  "isAlive": boolean,
	  "age": int32,
	  "address": {
	    "streetAddress": string,
	    "city": string,
	    "state": string,
	    "postalCode": string
	  },
	  "phoneNumbers": [
	    {
	      "type": string,
	      "number": string
	    },
	    {
	      "type": string,
	      "number": string
	    },
	    {
	      "type": string,
	      "number": string
	    }
	  ],
	  "children": [raw],
	  "spouse": boolean
	}
###Json file example
	
	{
	  "firstName": "John",
	  "lastName": "Smith",
	  "isAlive": true,
	  "age": 25,
	  "address": {
	    "streetAddress": "21 2nd Street",
	    "city": "New York",
	    "state": "NY",
	    "postalCode": "10021-3100"
	  },
	  "phoneNumbers": [
	    {
	      "type": "home",
	      "number": "212 555-1234"
	    },
	    {
	      "type": "office",
	      "number": "646 555-4567"
	    },
	    {
	      "type": "mobile",
	      "number": "123 456-7890"
	    }
	  ],
	  "children": [],
	  "spouse": null
	}

## References
For further information please refer to the user programmers guide: [CC3X20 Programmer's Guide](http://www.ti.com/lit/swru455)