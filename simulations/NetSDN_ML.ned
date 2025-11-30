package modelingproject4sdn.simulations;

import modelingproject4sdn.SDNNode_ML;
import modelingproject4sdn.Node;
import ned.DatarateChannel;

//
// Network with ML-capable SDN controller
//
network NetSDN_ML
{
    types:
        channel C extends DatarateChannel
        {
            delay = uniform(0.01ms, 10ms);
            datarate = uniform(1Mbps, 10Mbps);
        }
    submodules:
        // ML-Enhanced SDN Controller
        sdn: SDNNode_ML {
            address = 0;
            @display("p=300,200;i=block/control,red");
        }
        
        // Regular nodes
        device1: Node {  address = 1;   @display("p=100,100");   }
        device2: Node {  address = 2;   @display("p=100,300");   }
        device3: Node {  address = 3;   @display("p=500,300");   }
        device4: Node {  address = 4;   @display("p=500,100");   }
        device5: Node {  address = 5;   @display("p=300,400");   }
        
    connections:
        // Star topology with SDN at center
        device1.port++ <--> C <--> sdn.port++;
        device2.port++ <--> C <--> sdn.port++;
        device3.port++ <--> C <--> sdn.port++;
        device4.port++ <--> C <--> sdn.port++;
        device5.port++ <--> C <--> sdn.port++;
        
        // Mesh connections
        device1.port++ <--> C <--> device2.port++;
        device1.port++ <--> C <--> device4.port++;
        device2.port++ <--> C <--> device5.port++;
        device3.port++ <--> C <--> device4.port++;
        device3.port++ <--> C <--> device5.port++;
        device4.port++ <--> C <--> device5.port++;
}