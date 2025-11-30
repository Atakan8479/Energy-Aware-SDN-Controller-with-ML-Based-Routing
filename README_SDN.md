# SDN-Enhanced Routing Project

## Overview
This project implements a Software-Defined Networking (SDN) architecture in OMNeT++ where:
- A central SDN controller manages the network
- Regular nodes broadcast discovery information (battery level, distance)
- The SDN controller uses cTopology for topology discovery
- After discovery, nodes send application data packets

## Architecture

### Components Created/Modified:

1. **SDNController.ned** - NED definition for SDN controller
2. **SDNController.cc** - SDN controller implementation with cTopology
3. **SDNNode.ned** - Wrapper module for SDN node
4. **Packet.msg** - Enhanced packet with discovery fields (packetType, batteryLevel, distanceToSDN)
5. **Routing.cc** - Modified to broadcast discovery information
6. **NetSDN.ned** - Example network topology with SDN controller
7. **omnetpp.ini** - Added [NetSDN] configuration

## How It Works

### Discovery Phase:
1. At startup (0.1-0.5s), each node creates a DISCOVERY packet
2. The discovery packet contains:
   - Source address
   - Battery level (initialized at 100%, decreases with use)
   - Distance to SDN controller
3. Nodes send discovery packets to the SDN controller (address 0)
4. SDN controller receives and stores node information in its database

### Data Phase:
5. After discovery, nodes send DATA packets (application traffic)
6. SDN controller uses cTopology to maintain network view
7. Periodic topology updates occur every 5 seconds (configurable)

## Compilation

```bash
cd routing2
make clean
make
```

## Running the Simulation

```bash
# Run with GUI
./routing2 -u Qtenv -c NetSDN

# Run in command line
./routing2 -u Cmdenv -c NetSDN
```

## Key Files Modified

### Packet.msg
Added fields:
- `int packetType` (DATA=0, DISCOVERY=1)
- `double batteryLevel` (%)
- `double distanceToSDN` (meters)

### Routing.cc
Added functionality:
- `broadcastDiscovery()` - Broadcasts node info at startup
- `calculateDistanceToSDN()` - Calculates distance to controller
- Battery level simulation
- Discovery packet forwarding to SDN

### SDNController.cc
Key features:
- Uses `cTopology` for topology discovery
- Maintains `nodeDatabase` with battery and distance info
- Periodic topology updates
- Processes discovery packets from nodes
- Forwards data packets based on routing decisions

## Network Topology (NetSDN)

```
    device1 ----+
                |
    device2 ----+---- SDN Controller (address 0)
                |
    device3 ----+
                |
    device4 ----+
                |
    device5 ----+
```

Plus additional mesh connections between devices.

## Parameters

### SDN Controller
- `address` = 0 (fixed)
- `discoveryInterval` = 5s (configurable in omnetpp.ini)

### Regular Nodes
- `appType` = "App"
- `destAddresses` = "1 3 4" (traffic destinations)
- `sendIaTime` = uniform(1s, 3s)
- `packetLength` = 1024 bytes

## Monitoring

### Signals/Statistics:
- SDN Controller:
  - `topologyUpdated` - Counts topology discovery events

- Regular Nodes:
  - `drop` - Dropped packets
  - `outputIf` - Output interface selection
  - `endToEndDelay` - Packet delays
  - `hopCount` - Number of hops

## Differences from OpenFlow

This implementation uses a **simplified SDN approach**:
- No OpenFlow protocol overhead
- Direct packet broadcasting for discovery
- Centralized topology view using cTopology
- No flow tables (uses traditional routing tables)
- Custom packet format instead of OpenFlow messages

## Future Enhancements

1. Implement flow-based routing decisions
2. Add dynamic route updates based on battery/distance
3. Implement energy-aware routing algorithms
4. Add link quality metrics
5. Implement actual distance calculation using topology
6. Add flow table management

## Troubleshooting

If you encounter compilation errors:
1. Regenerate message files: `opp_msgc Packet.msg`
2. Clean and rebuild: `make clean && make`
3. Ensure OMNeT++ version 5.x or 6.x

## Notes

- SDN controller is always at address 0
- Discovery happens once at simulation start
- Battery levels decrease with each discovery broadcast
- Distance calculation is currently randomized (uniform 10-100m)
