#include <omnetpp.h>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include "Packet_m.h"

using namespace omnetpp;

/**
 * SDN Controller with Machine Learning capabilities
 */
class SDNController_ML : public cSimpleModule
{
  private:
    int myAddress;
    double discoveryInterval;
    std::string datasetFile;
    bool enableMLRouting;
    double trainingThreshold;

    bool   energyAwareRouting;
    double lowBatteryThreshold;
    double batteryWeight;
    double linkQualityWeight;
    double distanceWeight;
    double fairnessWeight;

    cMessage *discoveryTimer;

    struct NodeMetrics {
        int address;
        double batteryLevel;
        double distance;
        double avgDelay;
        double packetLoss;
        double throughput;
        int hopCount;
        double linkQuality;
        simtime_t lastUpdate;
        int connectedNeighbors;
    };

    struct FlowData {
        int srcAddr;
        int destAddr;
        double srcBattery;
        double destBattery;
        double pathDistance;
        int chosenPath;
        double pathDelay;
        double pathQuality;
        simtime_t timestamp;
    };

    std::map<int, NodeMetrics> nodeDatabase;
    std::vector<FlowData> trainingDataset;
    int totalFlowsProcessed;

    struct MLModel {
        bool isTrained;
        std::vector<FlowData> trainingSet;
        int k;
    } mlModel;

    simsignal_t topologyUpdatedSignal;
    simsignal_t mlPredictionSignal;
    simsignal_t routingDecisionSignal;

    std::ofstream datasetStream;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    void performTopologyDiscovery();
    void processDiscoveryPacket(Packet *pkt);
    void forwardDataPacket(Packet *pkt);

    int findBestRouteML(int srcAddr, int destAddr);
    int findBestRouteTraditional(int srcAddr, int destAddr);
    void exportToDataset(const FlowData &data);
    void trainMLModel();
    int predictBestPath(int srcAddr, int destAddr);
    double calculateEuclideanDistance(const FlowData &a, const FlowData &b);
    double calculatePathQuality(int srcAddr, int destAddr, int pathIndex);
    int findGateToDestination(int destAddr);
    int selectEnergyAwareGate(int srcAddr, int destAddr, int preferredGate);

  public:
    virtual ~SDNController_ML();
};

Define_Module(SDNController_ML);

SDNController_ML::~SDNController_ML()
{
    cancelAndDelete(discoveryTimer);
    if (datasetStream.is_open())
        datasetStream.close();
}

void SDNController_ML::initialize()
{
    myAddress = par("address");
    discoveryInterval = par("discoveryInterval");
    datasetFile = par("datasetFile").stdstringValue();
    enableMLRouting = par("enableMLRouting");
    trainingThreshold = par("trainingThreshold");

    energyAwareRouting   = par("energyAwareRouting");
    lowBatteryThreshold  = par("lowBatteryThreshold");
    batteryWeight        = par("batteryWeight");
    linkQualityWeight    = par("linkQualityWeight");
    distanceWeight       = par("distanceWeight");
    fairnessWeight       = par("fairnessWeight");

    topologyUpdatedSignal = registerSignal("topologyUpdated");
    mlPredictionSignal = registerSignal("mlPrediction");
    routingDecisionSignal = registerSignal("routingDecision");

    mlModel.isTrained = false;
    mlModel.k = 3;
    totalFlowsProcessed = 0;

    datasetStream.open(datasetFile, std::ios::out);
    if (datasetStream.is_open()) {
        datasetStream << "timestamp,src_addr,dest_addr,src_battery,dest_battery,"
                      << "path_distance,chosen_path,path_delay,path_quality\n";
        datasetStream.flush();
        EV << "SDN Controller: Dataset file opened: " << datasetFile << "\n";
    } else {
        EV << "SDN Controller: ERROR - Could not open dataset file: " << datasetFile << "\n";
    }

    discoveryTimer = new cMessage("discoveryTimer");
    scheduleAt(simTime() + 1.0, discoveryTimer);

    EV << "SDN Controller initialized at address " << myAddress << "\n";
    EV << "ML Routing: " << (enableMLRouting ? "ENABLED" : "DISABLED") << "\n";
    EV << "Training Threshold: " << trainingThreshold << " samples\n";
    EV << "Energy-aware routing: " << (energyAwareRouting ? "ENABLED" : "DISABLED")
       << " (lowBatteryThreshold=" << lowBatteryThreshold << "%)\n";
}

void SDNController_ML::handleMessage(cMessage *msg)
{
    if (msg == discoveryTimer) {
        performTopologyDiscovery();
        if (trainingDataset.size() >= trainingThreshold && !mlModel.isTrained) {
            trainMLModel();
        }

        scheduleAt(simTime() + discoveryInterval, discoveryTimer);
    }
    else {
        Packet *pkt = check_and_cast<Packet *>(msg);

        if (pkt->getPacketType() == DISCOVERY) {
            EV << "SDN: Received DISCOVERY packet from node " << pkt->getSrcAddr() << "\n";
            processDiscoveryPacket(pkt);
            delete pkt;
        }
        else {
            EV << "SDN: Received DATA packet from " << pkt->getSrcAddr()
               << " to " << pkt->getDestAddr() << "\n";
            forwardDataPacket(pkt);
        }
    }
}

void SDNController_ML::performTopologyDiscovery()
{
    EV << "\n==== TOPOLOGY DISCOVERY ====\n";
    EV << "Time: " << simTime() << "\n";
    EV << "Node database has " << nodeDatabase.size() << " entries\n";

    if (nodeDatabase.size() > 0) {
        EV << "\n--- Node Metrics Database ---\n";
        EV << "Addr | Battery | Distance | Delay | Quality\n";
        EV << "-----+---------+----------+-------+--------\n";

        for (auto &entry : nodeDatabase) {
            NodeMetrics &nm = entry.second;
            EV << std::setw(4) << nm.address << " | "
               << std::setw(6) << std::fixed << std::setprecision(1) << nm.batteryLevel << "% | "
               << std::setw(7) << std::setprecision(2) << nm.distance << "m | "
               << std::setw(5) << std::setprecision(3) << nm.avgDelay << "s | "
               << std::setw(6) << std::setprecision(2) << nm.linkQuality << "%\n";
        }
    }

    EV << "\nTraining dataset size: " << trainingDataset.size() << "\n";
    EV << "Total flows processed: " << totalFlowsProcessed << "\n";
    EV << "ML Model trained: " << (mlModel.isTrained ? "YES" : "NO") << "\n";
    EV << "=============================\n\n";

    emit(topologyUpdatedSignal, (long)nodeDatabase.size());
}

void SDNController_ML::processDiscoveryPacket(Packet *pkt)
{
    int srcAddr = pkt->getSrcAddr();

    EV << "SDN: Processing DISCOVERY from Node " << srcAddr
       << " (Battery: " << pkt->getBatteryLevel() << "%, Distance: "
       << pkt->getDistanceToSDN() << "m)\n";

    NodeMetrics &nm = nodeDatabase[srcAddr];
    nm.address = srcAddr;
    nm.batteryLevel = pkt->getBatteryLevel();
    nm.distance = pkt->getDistanceToSDN();
    nm.avgDelay = pkt->getPathDelay();
    nm.packetLoss = uniform(0, 5);
    nm.throughput = uniform(1, 10);
    nm.hopCount = pkt->getHopCount();
    nm.linkQuality = 100.0 - nm.packetLoss;
    nm.lastUpdate = simTime();
    nm.connectedNeighbors = intuniform(1, 4);

    EV << "SDN: Node " << srcAddr << " added/updated in database\n";
}

int SDNController_ML::findGateToDestination(int destAddr)
{
    int numGates = gateSize("out");

    if (numGates == 0) {
        EV << "SDN: No output gates available!\n";
        return -1;
    }

    int gateIndex = (destAddr - 1) % numGates;

    EV << "SDN: Routing to device " << destAddr << " via gate " << gateIndex << "\n";

    return gateIndex;
}

int SDNController_ML::selectEnergyAwareGate(int srcAddr, int destAddr, int preferredGate)
{
    int numGates = gateSize("out");
    if (numGates <= 0)
        return -1;

    double avgBattery = 100.0;
    if (!nodeDatabase.empty()) {
        double sum = 0.0;
        int count = 0;
        for (auto &kv : nodeDatabase) {
            sum += kv.second.batteryLevel;
            count++;
        }
        if (count > 0)
            avgBattery = sum / count;
    }

    double bestScore = -1e9;
    int bestGate = preferredGate;

    for (int i = 0; i < numGates; i++) {
        int neighborAddr = i + 1;

        double battery   = 100.0;
        double quality   = 90.0;
        double distance  = 50.0;
        double degree    = 1.0;

        auto it = nodeDatabase.find(neighborAddr);
        if (it != nodeDatabase.end()) {
            NodeMetrics &nm = it->second;
            battery  = nm.batteryLevel;
            quality  = nm.linkQuality;
            distance = 100.0 - std::min(nm.distance, 100.0);
            degree   = (double)nm.connectedNeighbors;
        }

        double fairnessPenalty = 0.0;
        if (battery < avgBattery)
            fairnessPenalty = (avgBattery - battery);

        double score =
            batteryWeight      * battery   +
            linkQualityWeight  * quality   +
            distanceWeight     * distance  +
            fairnessWeight     * degree    -
            fairnessWeight     * fairnessPenalty;

        if (battery < lowBatteryThreshold)
            score -= 50.0;

        if (i == preferredGate)
            score += 5.0;

        if (score > bestScore) {
            bestScore = score;
            bestGate  = i;
        }
    }

    return bestGate;
}

void SDNController_ML::forwardDataPacket(Packet *pkt)
{
    int srcAddr = pkt->getSrcAddr();
    int destAddr = pkt->getDestAddr();
    totalFlowsProcessed++;
    EV << "SDN: Routing DATA packet #" << totalFlowsProcessed
       << " from " << srcAddr << " to " << destAddr << "\n";

    int outGateIndex = -1;
    if (enableMLRouting && mlModel.isTrained) {
        outGateIndex = findBestRouteML(srcAddr, destAddr);
        EV << "  Using ML-based routing -> gate " << outGateIndex << "\n";
    } else {
        outGateIndex = findBestRouteTraditional(srcAddr, destAddr);
        EV << "  Using traditional routing -> gate " << outGateIndex << "\n";
    }

    if (outGateIndex < 0 || outGateIndex >= gateSize("out")) {
        outGateIndex = findGateToDestination(destAddr);
        EV << "  Fallback routing -> gate " << outGateIndex << "\n";
    }

    if (outGateIndex >= 0 && outGateIndex < gateSize("out")) {
        EV << "  Forwarding via gate " << outGateIndex << "\n";

        FlowData fd;
        fd.srcAddr = srcAddr;
        fd.destAddr = destAddr;
        fd.srcBattery = (nodeDatabase.find(srcAddr) != nodeDatabase.end()) ?
                        nodeDatabase[srcAddr].batteryLevel : 100.0;
        fd.destBattery = (nodeDatabase.find(destAddr) != nodeDatabase.end()) ?
                         nodeDatabase[destAddr].batteryLevel : 100.0;
        fd.pathDistance = (nodeDatabase.find(srcAddr) != nodeDatabase.end()) ?
                          nodeDatabase[srcAddr].distance : 50.0;
        fd.chosenPath = outGateIndex;
        fd.pathDelay = pkt->getPathDelay();
        fd.pathQuality = calculatePathQuality(srcAddr, destAddr, outGateIndex);
        fd.timestamp = simTime();

        exportToDataset(fd);
        trainingDataset.push_back(fd);
        emit(routingDecisionSignal, outGateIndex);
        send(pkt, "out", outGateIndex);
    }
    else {
        EV << "  No valid route, dropping packet\n";
        delete pkt;
    }
}

int SDNController_ML::findBestRouteML(int srcAddr, int destAddr)
{
    int predictedPath = predictBestPath(srcAddr, destAddr);
    emit(mlPredictionSignal, (double)predictedPath);

    if (!energyAwareRouting)
        return predictedPath;

    int gate = selectEnergyAwareGate(srcAddr, destAddr, predictedPath);

    EV << "SDN: [EA-ML] src=" << srcAddr
       << " dest=" << destAddr
       << " mlGate=" << predictedPath
       << " chosenGate=" << gate << "\n";

    return gate;
}

int SDNController_ML::findBestRouteTraditional(int srcAddr, int destAddr)
{
    int directGate = findGateToDestination(destAddr);

    if (!energyAwareRouting)
        return directGate;

    int gate = selectEnergyAwareGate(srcAddr, destAddr, directGate);

    EV << "SDN: [EA-TRAD] src=" << srcAddr
       << " dest=" << destAddr
       << " directGate=" << directGate
       << " chosenGate=" << gate << "\n";

    return gate;
}

void SDNController_ML::exportToDataset(const FlowData &data)
{
    if (datasetStream.is_open()) {
        datasetStream << std::fixed << std::setprecision(6)
                      << data.timestamp.dbl() << ","
                      << data.srcAddr << ","
                      << data.destAddr << ","
                      << data.srcBattery << ","
                      << data.destBattery << ","
                      << data.pathDistance << ","
                      << data.chosenPath << ","
                      << data.pathDelay << ","
                      << data.pathQuality << "\n";
        datasetStream.flush();

        EV << "  Data exported to CSV (row #" << trainingDataset.size() + 1 << ")\n";
    } else {
        EV << "  WARNING: Dataset file not open!\n";
    }
}

void SDNController_ML::trainMLModel()
{
    EV << "\n*** TRAINING ML MODEL ***\n";
    EV << "Training samples: " << trainingDataset.size() << "\n";

    mlModel.trainingSet = trainingDataset;
    mlModel.isTrained = true;

    EV << "ML Model trained successfully!\n";
    EV << "Model type: K-Nearest Neighbors (k=" << mlModel.k << ")\n";
    EV << "*************************\n\n";
}

int SDNController_ML::predictBestPath(int srcAddr, int destAddr)
{
    if (!mlModel.isTrained || mlModel.trainingSet.empty()) {
        return findBestRouteTraditional(srcAddr, destAddr);
    }

    FlowData query;
    query.srcAddr = srcAddr;
    query.destAddr = destAddr;
    query.srcBattery = (nodeDatabase.find(srcAddr) != nodeDatabase.end()) ?
                       nodeDatabase[srcAddr].batteryLevel : 100.0;
    query.destBattery = (nodeDatabase.find(destAddr) != nodeDatabase.end()) ?
                        nodeDatabase[destAddr].batteryLevel : 100.0;
    query.pathDistance = (nodeDatabase.find(srcAddr) != nodeDatabase.end()) ?
                         nodeDatabase[srcAddr].distance : 50.0;

    std::vector<std::pair<double, int>> distances;

    for (const auto &sample : mlModel.trainingSet) {
        double dist = calculateEuclideanDistance(query, sample);
        distances.push_back({dist, sample.chosenPath});
    }

    std::sort(distances.begin(), distances.end());

    std::map<int, int> votes;
    int limit = std::min((int)distances.size(), mlModel.k);

    for (int i = 0; i < limit; i++) {
        votes[distances[i].second]++;
    }

    int bestPath = -1;
    int maxVotes = 0;
    for (auto &vote : votes) {
        if (vote.second > maxVotes) {
            maxVotes = vote.second;
            bestPath = vote.first;
        }
    }

    if (bestPath < 0 || bestPath >= gateSize("out")) {
        bestPath = findGateToDestination(destAddr);
    }

    return bestPath;
}

double SDNController_ML::calculateEuclideanDistance(const FlowData &a, const FlowData &b)
{
    double d1 = (a.srcBattery - b.srcBattery) / 100.0;
    double d2 = (a.destBattery - b.destBattery) / 100.0;
    double d3 = (a.pathDistance - b.pathDistance) / 100.0;

    return sqrt(d1*d1 + d2*d2 + d3*d3);
}

double SDNController_ML::calculatePathQuality(int srcAddr, int destAddr, int pathIndex)
{
    double quality = 50.0;

    if (nodeDatabase.find(srcAddr) != nodeDatabase.end()) {
        quality += nodeDatabase[srcAddr].linkQuality * 0.25;
        quality += nodeDatabase[srcAddr].batteryLevel * 0.15;
    }
    if (nodeDatabase.find(destAddr) != nodeDatabase.end()) {
        quality += nodeDatabase[destAddr].linkQuality * 0.25;
        quality += nodeDatabase[destAddr].batteryLevel * 0.15;
    }

    quality += uniform(-10, 10);

    return std::max(0.0, std::min(100.0, quality));
}

void SDNController_ML::finish()
{
    EV << "\n==== SDN CONTROLLER FINAL REPORT ====\n";
    EV << "Total nodes discovered: " << nodeDatabase.size() << "\n";
    EV << "Total flows recorded: " << trainingDataset.size() << "\n";
    EV << "Total flows processed: " << totalFlowsProcessed << "\n";
    EV << "ML Model trained: " << (mlModel.isTrained ? "YES" : "NO") << "\n";
    EV << "Dataset file: " << datasetFile << "\n";
    EV << "======================================\n";

    if (datasetStream.is_open()) {
        datasetStream.close();
        EV << "Dataset file closed.\n";
    }

    if (nodeDatabase.size() > 0) {
        EV << "\nFinal Node Statistics:\n";
        for (auto &entry : nodeDatabase) {
            NodeMetrics &nm = entry.second;
            EV << "Node " << nm.address << ": "
               << "Battery=" << nm.batteryLevel << "%, "
               << "Quality=" << nm.linkQuality << "%\n";
        }
    }
}
