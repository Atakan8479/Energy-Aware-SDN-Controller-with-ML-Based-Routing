//
// Modified Packet with SDN Discovery Information
//

enum PacketType
{
    DATA = 0;
    DISCOVERY = 1;
}

//
// Represents a packet in the network with SDN capabilities
//
packet Packet
{
    int srcAddr @packetData;
    int destAddr @packetData;
    int hopCount @packetData;
    double pathDelay @packetData;
    double pathCost @packetData;

    // SDN Discovery Fields
    int packetType @packetData = 0;  // 0=DATA, 1=DISCOVERY
    double batteryLevel @packetData = 100.0;  // Node battery level (%)
    double distanceToSDN @packetData = 0.0;   // Distance to SDN controller
}
