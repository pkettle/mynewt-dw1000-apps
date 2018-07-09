
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

### To test the applications with newtmgr tool

Enable UART and disable RTT in twr_node_log and twr_tag_log and change the UART pins in hw/bsp/dwm1001/syscfg.yml

### To create twr_node_log
```
newt target create node_log
newt target set node_log app=apps/twr_node_log
newt target set node_log bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set node_log build_profile=debug
```
### To build and run node_log
```
newt build node_log
newt create-image node_log 1.0.0
newt load node_log 
```
### To create twr_tag_log
```
newt target create tag_log
newt target set tag_log app=apps/twr_tag_log
newt target set tag_log bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set tag_log build_profile=debug
```
### To build and run tag_log
```
newt build tag_log
newt create-image tag_log 1.0.0
newt load tag_log
```

The examples are configured to use the UART interface. 

### To test the applications with newtmgr tool

Enable UART and disable RTT in twr_node_log and twr_tag_log and change the UART pins in hw/bsp/dwm1001/syscfg.yml

## To run newtmgr commands

Establish connections using newtmgr commands.

```
To check the logs over shell

newtmgr conn add port_0 type=serial connstring="dev=/dev/ttyACM0" ----> (tag_log)
                                
                                  (and)

newtmgr conn add port_1 type=serial connstring="dev=/dev/ttyACM1" ----> (node_log)
```
```
To check the logs over BLE

newtmgr conn add myble type=ble connstring="peer_name=nimble-bleprph"
```
Check the logs on desired port

```
check the logs in tag_log as newtmgr APIs are in twr_tag_log

newtmgr log level_list -c port_0 - shows log levels on device

newtmgr log list -c port_0 - shows log names on a device

newtmgr log show -c port_0 - shows all the logs on a device

newtmgr log show -c ble - shows all the logs on a device over BLE
```
Likewise, there are many newtmgr commands to display the logs,echo,reset,statistics,memory acquired,tasks initiated etc.

To use those commands, please find below reference

https://mynewt.apache.org/latest/newtmgr/command_list/newtmgr_config/




