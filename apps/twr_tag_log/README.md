
## mynewt-dw1000-apps for logging

## Overview 
Newt Manager (newtmgr) is the application tool that enables a user to communicate with and manage remote devices running the Mynewt OS. It uses a connection profile to establish a connection with a device and sends command requests to the device.However,only the required commands can be enabled using this tool.

# Description
In this apps,

**apps/twr_node_log** waits for the tag to be connected to perform ranging.

**apps/twr_tag_log** enable the logs as it ranges with the node.

**NOTE:** Run tag followed by node.

## Current application (Work-in-Progress)
twr_node_log   // Elementary use-case, no network management layer, no auto site survey
twr_tag_log   // Elementary use-case, no network management layer, no auto site survey


Each twr_node_log and twr_tag_log are build as follows. 
(executed from the mynewt-dw1000-app directory).

### To create and build twr_node_log
```
newt target create node_log
newt target set node_log app=apps/twr_node_log
newt target set node_log bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set node_log build_profile=debug
```
### To run node_log
```
newt run node_log 0
```
### To create and build twr_tag_log
```
newt target create tag_log
newt target set tag_log app=apps/twr_tag_log
newt target set tag_log bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set tag_log build_profile=debug
```
### To run tag_log
```
newt run tag_log 0
```

The examples are configured to use the RTT interface. To launch the console with RTT interface,use 


telnet localhost 19021

After connecting both the tag and node, on the console type help to see what commands are supported for your application.

### To test the applications with newtmgr tool

Enable UART and disable RTT in twr_node_log and twr_tag_log and change the UART pins in hw/bsp/dwm1001/syscfg.yml

## To run newtmgr commands

Establish connections using newtmgr commands.

```
newtmgr conn add port_0 type=serial connstring="dev=/dev/ttyACM0" (based on device available)
                                
                                  (or)

newtmgr conn add port_1 type=serial connstring="dev=/dev/ttyACM1" (based on device available)
```

Check the logs on desired port

```
newtmgr log level_list -c port_0/port_1 - shows log levels on device

newtmgr log list -c port_0/port_1 - shows log names on a device

newtmgr log show -c port_0/port_1 - shows all the logs on a device

```
Likewise, there are many newtmgr commands to display the logs,echo,reset,statistics,memory acquired,tasks initiated etc.

To use those commands, please find below reference

https://mynewt.apache.org/latest/newtmgr/command_list/newtmgr_config/



