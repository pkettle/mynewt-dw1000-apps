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

# Decawave DW1000 Application twr_node0_lwip_p2p

## Overview
The Decawave DW1000 Application twr_node0_lwip_p2p showcases the ability of lwIP P2P service to send and receive 
user defined payloads to another node via lwip_p2p service.
In this case, we are sending a command from master node (node0) to other node to start ranging with third node.
The received RANGE value is then displayed on the debug console.

## Building

1. Build the twr_node0_lwip_p2p app.

```no-highlight

newt target create twr_node0_lwip_p2p
newt target set twr_node0_lwip_p2p app=apps/twr_node0_lwip_p2p
newt target set twr_node0_lwip_p2p bsp=@mynewt-dw1000-core/hw/bsp/dwm1001
newt target set twr_node0_lwip_p2p build_profile=debug 
newt run twr_node0_lwip_p2p 0
```

## Testing

1. Flash twr_node0_lwip_p2p into Master Node.
2. Build and flash twr_node_lwip_p2p into Node(let's assume node 1).
3. Connect both the nodes to Dev PC and monitor the logs of both the nodes in serial comm application.
4. The expected output is as shown below,  
	When only master node is running  
	```
	[Payload Sent]
	```

	When both master node and node 1 are running  
	```
	[Payload Sent]  
		[Range : <Received Range Value>]
	```
