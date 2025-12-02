package modelingproject4sdn;

//
// Central SDN Controller with ML capabilities
// Collects node information and makes intelligent, possibly energy-aware, routing decisions
// CHANGE 1: Description extended to mention *energy-aware* routing.
//
simple SDNController_ML
{
    parameters:
        int    address;
        double discoveryInterval @unit(s) = default(10s);
        string datasetFile               = default("sdn_dataset.csv");  

        bool   enableMLRouting           = default(true);
                                                         
        bool   energyAwareRouting        = default(false);  // CHANGE 3: NEW flag – turn energy-aware gate selection on/off.
        double trainingThreshold         = default(100);    // Number of samples before training ML model
                                                            // (kept from original, comment slightly rephrased).

        // CHANGE: NEW tuning knobs for the energy-aware scoring function
        //           used inside the controller (selectEnergyAwareGate / findBestRouteTraditional).
        double lowBatteryThreshold       = default(20);     // [%] below this is considered "low"
        double batteryWeight             = default(0.4);    // weight of node battery in score
        double linkQualityWeight         = default(0.3);    // weight of link quality in score
        double distanceWeight            = default(0.2);    // weight of (inverse) distance in score
        double fairnessWeight            = default(0.1);    // weight for degree/fairness term

        @display("i=block/control,blue");

        // Statistics (unchanged)
        @signal[topologyUpdated](type="long");
        @signal[mlPrediction](type="double");
        @signal[routingDecision](type="long");

        @statistic[topologyUpdated](title="topology update events";record=count,vector);
        @statistic[mlPrediction](title="ML routing predictions";record=stats,vector);
        @statistic[routingDecision](title="routing decisions";record=count,histogram);

    gates:
        input  in[];
        output out[];
}
