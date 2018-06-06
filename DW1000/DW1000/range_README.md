<!--
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->

# Range Service

## Overview

Range service allows a device to initiate ranging with multiple target devices and get the range values corresponding to each target device.
The service accepts number of target devices and their addresses during initialization. It initiates ranging with each target device in a sequence and returns the time stamps used to calculate the ToF and range. The frequency of ranging is controlled by a MACRO setting.

In this example, application (apps/twr_node_range) initiates ranging with multiple targets (apps/twr_tag_range) and prints the range values with each target and repeats the same.

### 1. Build range example applications
```no-highlight
newt target create node
newt target set node app=apps/twr_node_range
newt target set node bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set node build_profile=debug 
newt target amend node syscfg=DEVICE_ID=0x6001
newt run node 0


newt target create tag1
newt target set tag1 app=apps/twr_tag_range
newt target set tag1 bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set tag1 build_profile=debug
newt target amend tag1 syscfg=DEVICE_ID=0x6002
newt run tag1 0

newt target create tag2
newt target set tag2 app=apps/twr_tag_range
newt target set tag2 bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set tag2 build_profile=debug
newt target amend tag2 syscfg=DEVICE_ID=0x6003
newt run tag2 0
```

