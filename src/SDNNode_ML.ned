 

package modelingproject4sdn;

//
// SDN Node with ML Controller
//

 
//
// SDN Node with ML-enhanced controller
//
module SDNNode_ML
{
    parameters:
        int address;
        @display("i=block/control,red");
    gates:
        inout port[];
    submodules:
        controller: SDNController_ML {
            parameters:
                address = parent.address;
                @display("p=140,130");
            gates:
                in[sizeof(parent.port)];
                out[sizeof(parent.port)];
        }
        queue[sizeof(port)]: L2Queue {
            parameters:
                @display("p=80,200,row");
        }
    connections:
        for i=0..sizeof(port)-1 {
            controller.out[i] --> queue[i].in;
            controller.in[i] <-- queue[i].out;
            queue[i].line <--> port[i];
        }
}