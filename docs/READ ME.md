##PiDACS READ ME

###Overview

PiDACS (Raspberry Pi Data Acquisition and Control System) is a collection of Python modules that provide access to digital inputs, analog inputs, and digital outputs on a RaspBerry Pi computer. PiDACS modules can be deployed to run interactively on a local host, be embedded in user applications on a local host, or serve clients on local/remote hosts. In particular, PiDACS modules can serve the Indigo Home Automation Server (on a Mac) using the *PiDACS Bridge* Indigo plugin available in a separate repository at <https://github.com/papamac/PiDACS-Bridge>.

###Hardware/Software Compatibility

PiDACS should run on any Raspberry Pi. It has been tested on Raspberry Pi models Zero, Zero W, 2B+, 3B, and 3B+ using varying Raspbian versions and varying I/O configurations. The current preferred configuration is a Raspberry Pi 3B+ running Raspbian Stretch (NOOBS 3.0.0) and Python 3.5.3.

Compatible I/O configurations include the Raspberry Pi built-in General Purpose I/O (GPIO) and external I/O devices connected via the built-in Inter-Integrated Circuit (I2C) bus. On 40-pin Raspberry Pi boards, 16 of the 40 pins can be individually programmed as GPIO digital inputs or outputs. I2C digital I/O is provided by Microchip (MCP) MCP23008 8-Bit I/O Expander chips and/or MCP23017 16-Bit I/O Expander chips. Like the built-in GPIO, each MCP I/O pin can be individually programmed as a digital input or output. I2C analog inputs are provided by MCP3424 4-channel, 18-bit A/D converter chips.

The MCP I2C bus devices are conveniently implemented on Raspberry Pi hat boards made by AB Electronics UK (<https://www.abelectronics.co.uk>). PiDACS has been tested with the the following AB Electronics expander boards:

* IO Pi Zero - 16-channel digital I/O expander for the Raspberry Pi Zero.
* IO Pi Plus - 32-channel digital I/O expander for the Raspberry Pi models B+, 2B+, 3B, and 3B+.
* ADC Pi v2.0 - 8-channel, 17-bit single ended A/D expander board using two MCP3424 4-channel chips. Note that this board is a legacy product that has been replaced by the ADC Pi v3.0 board with a smaller form factor. The v3.0 board has not been tested with PiDACS, but should be compatible because it uses the same MCP3424 chips.  

###Key Concepts

* *Channel* - A *channel* is the basic path for all PiDACS operations. *Channels* can be digital inputs, digital outputs, or analog inputs. *Channels* can be configured, read, written, polled, and reported. *Channels* are addressed by *channel* numbers which are non-negative integers assigned by the user.

* *Port* - A *port* is a group of channels that are related by I2C bus address and/or chip hardware addressing. Channels in a specific *port* are numbered consecutively, polled and reported in the same manner, and processed in a single concurrent thread. The mapping between channel numbers and the bus/chip hardware addresses are defined in a *port_defs.xml* file (see below). Channel numbers must be unique within the collection of *ports* defined in the file. The common polling/reporting characteristics for a *port* are also defined in the *port_defs.xml* file. 

###Features

* *Versatile I/O Configurations* - PiDACS provides access to multiple digital input, digital output, and analog input channels on a single Raspberry Pi. I/O channel types and the numbers of channels of each type are easily configurable by the user in the *port_defs.xml* file (see below). Individual channel characteristics are specified using the *configure* method in stand-alone apps or by using a *configure* request from a remote client.

* *Optional Polling and Change Reporting for Inputs* - Output channels are written by individual request. Input channels may be read on request or automatically polled at varying rates. Values read from input polling can be reported at the polling rate, or can be change detected and reported only on change. Analog channels are change detected using a percentage of the of the current value.  If the absolute value of the difference between the current and prior values exceeds a specified percentage of the current value then a change is declared. Polling rates and change reporting are specified by port in the *port_defs.xml* file as described below. Analog change percentages are set individually by channel using the *configure* method or a *configure* request.

* *Easy, Flexible Stand-alone App Deployment* - Stand-alone apps running on a host Raspberry Pi use the *PiDACS* class and its methods to perform I/O. An app first creates an instance of the *PiDACS* class. The instance reads a *port_defs.xml* file during initialization to define its ports and their characteristics. The app then invokes the *PiDACS start, stop, configure, read, write, and process_request* methods to perform I/O (see *PiDACS.py* below). Input data, whether read individually or automatically polled, are asynchronously queued to an instance-specific *data_queue* which is synchronously dequeued using the *get_data* method. Although stand-alone apps normally require only a single *PiDACS* instance, multiple instances can be employed if desired.

* *Flexible Client/Server Deployment* - The *PiDACS_server.py* module is a stand-alone app on the *PiDACS* host that accepts requests from and serves data to multiple local and/or remote clients. Clients connect to the server using the server *hostname:port number* which uniquely identifies the server. The connection creates a bidirectional TCP/IP socket that is used to receive requests from and send data to the client. A client may request channel configurations and read/write operations using a text-based request language (see below) that implements the functionality of the *PiDACS* access methods described above. The server unqueues all data from the *PiDACS data_queue* and sends it to all connected clients regardless of which client requested the data. This continues as long as one or more clients is connected. If there are no active connections, the server remains active and waits for new clients to connect. Note that multiple servers (with multiple *PiDACS* instances and *port_def.xml* files) can run on a single host as long as they use different *port numbers*.

* *Two Available Clients* - The *PiDACS_client.py* module is an *PiDACS* client app that runs interactively from the Raspbian terminal command line.  It connects to an *PiDACS* server, accepts requests from the command line, sends the requests to the server, receives the data from the server, and prints the data to the terminal. The *PiDACS Bridge* Indigo plugin is an *PiDACS* client that runs as a plugin module that extends the Indigo Home Automation Server. It has all the functionality of *PiDACS_client.py* but offers an easy-to-use GUI interface within Indigo. It also allows Indigo to simultaneously connect to multiple *PiDACS* servers. The *PiDACS Bridge* Indigo plugin is available in a separate repository at <https://github.com/papamac/PiDACS-Bridge>.

* *Optional Printing* - If enabled, *PiDACS* will echo all requests and print all reported data on the host terminal. Terminal output, of course, can be re-directed to a file in the command line as with any LINUX process. There is, however, currently no logging capability with file closeout/swapping capability. Printing is controlled using the class methods *PiDACS.enable_printing* and *PiDACS.disable_printing*. These apply to all instances of the *PiDACS* class that are running in a single Raspbian process.

###Modules
* *iomgr.py*
* *pidacs.py*
* *pic.py*
* *pidacs_global.py*
* *i2cbus.py*

###Port Naming

###Interactive Request Syntax

