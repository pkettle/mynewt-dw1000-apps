## mynewt-rtls-apps
### Overview
his distribution contains the example applications for the dw1000 IR-UWB transceiver within the mynewt-OS. The dw1000 device driver model is integrated into the mynewt-OS (https://github.com/decawave/mynewt-dw1000-core). This driver includes native support for a 6lowPAN stack, Ranging Services, and Location Services, etc. Mynewt and it's tools build environment newt and management tools newtmgt create a powerful environment and deploying large-scale distributions within IoT.

For these examples, we leverage the Decawave dwm1001 module and dwm1001-dev kit. The dwm1001 includes a nrf52832 and the dw1000 transceiver. The dwm1001-dev is a breakout board that supports a Seggar OB-JLink interface with RTT support. The mynewt build environment provides a clean interface for maintaining platform agnostics distributions. The dwm1001-dev and the examples contained herein provide a clean out-of-the-box experience for UWB Location Based Services.

Warning: The dwm1001 comes flashed with a UWB Location Based Services stack. This distribution repurposes the hardware and is not intended to replace the functionality of the shipped stack. This distribution is intended to be a starting point for evaluating and developing one's own such stacks.

The unified app examples showcases how a small RTLS system can be built with underlying services. In this example first nodes are setup by going through first discovery followed by provisioning and then moves to infinite receive mode waiting for the tag's ranging or provisioning commands. The tags also goes through the same states as discovery followed by provisioning & ranging in a periodic manner. All the communication happens in a time slotted manner with the help of CCP packets. Each tag has a dedicated slot in which it is allowed to do Discovery, provisioning & ranging.

**NOTE:** First setup all the available nodes and plugin the tags only after all the nodes starts blinking the blue LED. Also a minimum of 3 nodes are required to run this example

### Project Status
Below are the current examples and associated hardware plaforms.

### To build pan_master
```no-highlight
newt target create pan_master
newt target set pan_master app=apps/pan_master
newt target set pan_master bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set pan_master build_profile=debug 
newt run pan_master 0
```

### To build clock_master
```no-highlight
newt target create clock_master
newt target set clock_master app=apps/clock_master
newt target set clock_master bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set clock_master build_profile=debug 
newt run clock_master 0
```

### To build twr_tag_unified
```no-highlight
newt target create twr_tag_unified
newt target set twr_tag_unified app=apps/twr_tag_unified
newt target set twr_tag_unified bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set twr_tag_unified build_profile=debug 
newt target amend twr_tag_unified syscfg=N_NODES=4
newt run twr_tag_unified 0
```

### To build twr_node_unified
```no-highlight
newt target create twr_node_unified 
newt target set twr_node_unified app=apps/twr_node_unified
newt target set twr_node_unified bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set twr_node_unified build_profile=debug 
newt run twr_node_unified 0
```

The examples are configured to use the Segger RTT console interface. This is covered within the mynewt tutorials/Tooling/SeggarRTT (https://mynewt.apache.org/latest/os/tutorials/segger_rtt/). To launch the console simply run

```no-highlight
telnet localhost 19021.
```

