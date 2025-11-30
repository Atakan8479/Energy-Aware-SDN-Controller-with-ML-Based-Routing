package modelingproject4sdn;

//
// Models a router.
//
simple Routing
{
    parameters:
        @display("i=block/routing");
        bool sendDiscovery = default(false);
        double discoveryInterval @unit(s) = default(10s);
        string sdnControllerAddress = default("sdn.controller");
        
        @signal[drop](type="long");
        @signal[outputIf](type="long");
        @statistic[drop](title="dropped packets"; record=vector?,count,sum; interpolationmode=none);
        @statistic[outputIf](title="output interface"; record=vector?; interpolationmode=none);
        
    gates:
        input localIn;
        output localOut;
        input in[];
        output out[];
}