
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


