package modelingproject4sdn;

//
// Central SDN Controller with ML capabilities
// Collects node information and makes intelligent, possibly energy-aware, routing decisions
//
simple SDNController_ML
{
    parameters:
        int    address;
        double discoveryInterval @unit(s) = default(10s);
        string datasetFile               = default("sdn_dataset.csv");  // Output dataset file

        bool   enableMLRouting           = default(true);   // Use ML model if trained
        bool   energyAwareRouting        = default(false);  // Enable energy-aware gate selection
        double trainingThreshold         = default(100);    // Samples before training ML model

        // Energy-aware tuning knobs (can override in omnetppNewML.ini)
        double lowBatteryThreshold       = default(20);     // [%] below this is considered "low"
        double batteryWeight             = default(0.4);
        double linkQualityWeight         = default(0.3);
        double distanceWeight            = default(0.2);
        double fairnessWeight            = default(0.1);

        @display("i=block/control,blue");

        // Statistics
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
