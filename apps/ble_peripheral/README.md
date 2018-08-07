#mynewt-logging-apps

##Overview

This project contains BLE based central and peripheral applications which communicates over BLE.

1.**ble_central** scans for the available devices, connects to the devices based on the device name"DECAWAVE_BLE" and receives notifications (logs)from the peripherals.
2.**ble_peripheral** continuously sends the logs to the central device after connected with the central device.

### Building the Apps
The following commands are useful get the source code, build and test the above apps.
```
git clone https://github.com/Decawave/mynewt-dw1000-apps.git
newt install -v
cd mynewt-dw1000-apps
git checkout ble_logging
cd repos/apache-mynewt-core
git checkout 1_4_0_dev
cd ../repos/apache-mynewt-nimble
git checkout 1_0_0_dev
cd ../../

```
Each ble_central and ble_peripheral apps are built as follows. (executed from the mynewt-dw1000-apps directory).

#To create target for ble_cent
```
newt target create ble_cent
newt target set ble_cent app=apps/ble_central
newt target set ble_cent bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set ble_cent build_profile=debug
```
#To build and run ble_cent
```
newt build ble_cent
newt create-image ble_cent 1.0.0
newt load ble_cent
```
#To create target for ble_prph
```
newt target create ble_prph
newt target set ble_prph app=apps/ble_peripheral
newt target set ble_prph bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set ble_prph build_profile=debug
```
#To build and run ble_prph
```
newt target amend ble_prph syscfg=BLE_PUBLIC_DEV_ADDR="(uint8_t[6]){0xAA,0xAA,0xAA,0xAA,0xAA,0xAA}":BLE_DEVICE_ID=1
newt build ble_prph
newt create-image ble_prph 1.0.0
newt load ble_prph

newt target amend ble_prph syscfg=BLE_PUBLIC_DEV_ADDR="(uint8_t[6]){0xBB,0xBB,0xBB,0xBB,0xBB,0xBB}":BLE_DEVICE_ID=2
newt build ble_prph
newt create-image ble_prph 1.0.0
newt load ble_prph

newt target amend ble_prph syscfg=BLE_PUBLIC_DEV_ADDR="(uint8_t[6]){0xCC,0xCC,0xCC,0xCC,0xCC,0xCC}":BLE_DEVICE_ID=3
newt build ble_prph
newt create-image ble_prph 1.0.0
newt load ble_prph

newt target amend ble_prph syscfg=BLE_PUBLIC_DEV_ADDR="(uint8_t[6]){0xDD,0xDD,0xDD,0xDD,0xDD,0xDD}":BLE_DEVICE_ID=4
newt build ble_prph
newt create-image ble_prph 1.0.0
newt load ble_prph
```
The examples are configured to use the UART interface.
