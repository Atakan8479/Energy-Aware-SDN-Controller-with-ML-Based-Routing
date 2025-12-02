//
// Modified Routing with SDN-based Data Forwarding
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <map>
#include <omnetpp.h>
#include "Packet_m.h"

using namespace omnetpp;

/**
 * Enhanced routing with SDN discovery and data forwarding through SDN
 * + battery-aware behaviour (FSM) on each node.
 */
class Routing : public cSimpleModule
{
  private:
    int myAddress;
    double batteryLevel;
    int sdnAddress;

    typedef std::map<int, int> RoutingTable;
    RoutingTable rtable;

    cMessage *discoveryTimer;
    bool sendDiscovery;
    double discoveryInterval;

    // CHANGE 1: new battery model – per–node FSM and timer
    //           (before: only a simple scalar batteryLevel updated inline)
    cMessage *batteryTimer;
    cFSM batteryFsm;
    enum {
        BAT_ACTIVE   = 0,
        BAT_CHARGING = 1
    };

    // (existing signals, unchanged in meaning)
    simsignal_t dropSignal;
    simsignal_t outputIfSignal;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    void sendDiscoveryPacket();
    double calculateDistanceToSDN();
    int getGateToSDN();

    // CHANGE 2: new helpers for the battery model
    void processBatteryTimer();                           // periodic FSM update
    void updateBatteryOnActivity(double minDrain, double maxDrain); // per-packet drain

  public:
    virtual ~Routing();
};

Define_Module(Routing);

Routing::~Routing()
{
    cancelAndDelete(discoveryTimer);
    // CHANGE 3: delete the new battery timer as well
    cancelAndDelete(batteryTimer);
}

void Routing::initialize()
{
    myAddress    = getParentModule()->par("address");
    batteryLevel = 100.0;
    sdnAddress   = 0;  // SDN controller address is always 0

    // CHANGE 4: initialise FSM and start periodic battery timer
    batteryFsm.setName("batteryFsm");
    batteryFsm.setState(BAT_ACTIVE);          // start in ACTIVE state
    batteryTimer = new cMessage("batteryTimer");
    scheduleAt(simTime() + 1, batteryTimer);  // periodic battery updates

    // Discovery / routing setup (as before)
    sendDiscovery     = par("sendDiscovery").boolValue();
    discoveryInterval = par("discoveryInterval");
    dropSignal        = registerSignal("drop");
    outputIfSignal    = registerSignal("outputIf");

    // Routing table build (same logic as before, just formatted)
    cTopology *topo = new cTopology("topo");
    std::vector<std::string> nedTypes;
    nedTypes.push_back("modelingproject4sdn.Node");
    nedTypes.push_back("modelingproject4sdn.SDNNode_ML");
    topo->extractByNedTypeName(nedTypes);

    EV << "Node " << myAddress << ": cTopology found "
       << topo->getNumNodes() << " nodes\n";

    cTopology::Node *thisNode = topo->getNodeFor(getParentModule());
    if (thisNode) {
        for (int i = 0; i < topo->getNumNodes(); i++) {
            if (topo->getNode(i) == thisNode)
                continue;

            topo->calculateUnweightedSingleShortestPathsTo(topo->getNode(i));
            if (thisNode->getNumPaths() == 0)
                continue;

            cGate *parentModuleGate = thisNode->getPath(0)->getLocalGate();
            int gateIndex = parentModuleGate->getIndex();

            int destAddr = topo->getNode(i)->getModule()->par("address");
            rtable[destAddr] = gateIndex;
            EV << "Node " << myAddress << ": route to "
               << destAddr << " via gate " << gateIndex << "\n";
        }
    }
    delete topo;

    EV << "Node " << myAddress << ": Routing table has "
       << rtable.size() << " entries\n";

    // Verify we have a route to SDN (unchanged)
    if (rtable.find(sdnAddress) != rtable.end()) {
        EV << "Node " << myAddress << ": Route to SDN controller FOUND via gate "
           << rtable[sdnAddress] << "\n";
    }
    else {
        EV << "Node " << myAddress << ": WARNING - No route to SDN controller!\n";
    }

    // Discovery timer setup (same logic)
    if (sendDiscovery && myAddress != sdnAddress) {
        discoveryTimer = new cMessage("discoveryTimer");
        scheduleAt(simTime() + uniform(0.5, 2.0), discoveryTimer);
        EV << "Node " << myAddress << ": Discovery scheduled\n";
    }
    else {
        discoveryTimer = nullptr;
    }
}

int Routing::getGateToSDN()
{
    auto it = rtable.find(sdnAddress);
    if (it != rtable.end())
        return it->second;
    return -1;
}

void Routing::handleMessage(cMessage *msg)
{
    if (msg == discoveryTimer) {
        // periodic discovery (as before)
        sendDiscoveryPacket();
        scheduleAt(simTime() + discoveryInterval, discoveryTimer);
    }
    // CHANGE 5: new branch – periodic battery FSM update
    else if (msg == batteryTimer) {
        processBatteryTimer();
    }
    // CHANGE 6: local traffic now gated by battery FSM, with structured drain
    else if (msg->arrivedOn("localIn")) {
        Packet *pkt = check_and_cast<Packet *>(msg);

        // if node is not ACTIVE, drop local traffic
        if (batteryFsm.getState() != BAT_ACTIVE) {
            EV << "Node " << myAddress
               << ": battery not available for transmission, dropping local packet\n";
            delete pkt;
            return;
        }

        int destAddr = pkt->getDestAddr();
        EV << "Node " << myAddress << ": Sending DATA packet to "
           << destAddr << " via SDN controller\n";

        // activity-based battery drain (was inline uniform() before)
        updateBatteryOnActivity(0.05, 0.2);

        // propagate updated metrics to packet
        pkt->setBatteryLevel(batteryLevel);
        pkt->setHopCount(pkt->getHopCount() + 1);
        pkt->setPathDelay(pkt->getPathDelay() + uniform(0.001, 0.005));

        int sdnGate = getGateToSDN();
        if (sdnGate >= 0) {
            EV << "Node " << myAddress
               << ": Forwarding to SDN via gate " << sdnGate << "\n";
            emit(outputIfSignal, sdnGate);
            send(pkt, "out", sdnGate);
        }
        else {
            EV << "Node " << myAddress
               << ": ERROR - No route to SDN controller, dropping\n";
            emit(dropSignal, (long)pkt->getByteLength());
            delete pkt;
        }
    }
    // CHANGE 7: transit traffic also checks battery FSM and uses shared drain helper
    else {
        Packet *pkt = check_and_cast<Packet *>(msg);
        int destAddr = pkt->getDestAddr();

        EV << "Node " << myAddress
           << ": Received packet destined to " << destAddr << "\n";

        if (destAddr == myAddress) {
            // simplified: we now always deliver to localOut
            // (old code special-cased DISCOVERY packets)
            EV << "Node " << myAddress << ": Packet arrived at destination\n";
            send(pkt, "localOut");
        }
        else {
            if (batteryFsm.getState() != BAT_ACTIVE) {
                EV << "Node " << myAddress
                   << ": battery not available for forwarding, dropping transit packet\n";
                delete pkt;
                return;
            }

            EV << "Node " << myAddress
               << ": Forwarding packet to " << destAddr << "\n";

            // smaller drain for transit forwarding
            updateBatteryOnActivity(0.02, 0.1);

            pkt->setBatteryLevel(batteryLevel);
            pkt->setHopCount(pkt->getHopCount() + 1);
            pkt->setPathDelay(pkt->getPathDelay() + uniform(0.001, 0.005));

            auto it = rtable.find(destAddr);
            if (it != rtable.end()) {
                int outGateIndex = it->second;
                emit(outputIfSignal, outGateIndex);
                send(pkt, "out", outGateIndex);
            }
            else {
                EV << "Node " << myAddress
                   << ": No route to " << destAddr << ", dropping\n";
                emit(dropSignal, (long)pkt->getByteLength());
                delete pkt;
            }
        }
    }
}

void Routing::sendDiscoveryPacket()
{
    // CHANGE 8: discovery is now also gated by battery FSM
    if (batteryFsm.getState() != BAT_ACTIVE) {
        EV << "Node " << myAddress
           << ": battery not available (state=" << batteryFsm.getState()
           << "), skipping discovery\n";
        return;
    }

    EV << "Node " << myAddress
       << ": Sending discovery packet to SDN controller\n";

    // use shared helper for discovery drain (instead of inline uniform())
    updateBatteryOnActivity(0.1, 0.5);

    char pkname[40];
    sprintf(pkname, "discovery-%d", myAddress);

    Packet *discoveryPkt = new Packet(pkname);
    discoveryPkt->setSrcAddr(myAddress);
    discoveryPkt->setDestAddr(sdnAddress);
    discoveryPkt->setPacketType(DISCOVERY);
    discoveryPkt->setBatteryLevel(batteryLevel);
    discoveryPkt->setDistanceToSDN(calculateDistanceToSDN());
    discoveryPkt->setPathDelay(uniform(0.001, 0.01));
    discoveryPkt->setByteLength(512);
    discoveryPkt->setHopCount(0);

    int sdnGate = getGateToSDN();
    if (sdnGate >= 0) {
        EV << "Node " << myAddress
           << ": Sending discovery via gate " << sdnGate << "\n";
        send(discoveryPkt, "out", sdnGate);
    }
    else {
        EV << "Node " << myAddress
           << ": ERROR - No route to SDN controller!\n";
        delete discoveryPkt;
    }
}

double Routing::calculateDistanceToSDN()
{
    // same simple synthetic distance model as before
    return uniform(10.0, 100.0) + (myAddress * 5.0);
}

// CHANGE 9: new FSM-based periodic battery evolution
void Routing::processBatteryTimer()
{
    FSM_Switch(batteryFsm)
    {
        case BAT_ACTIVE:
            batteryLevel -= uniform(0.01, 0.03);
            if (batteryLevel < 0)
                batteryLevel = 0;

            if (batteryLevel < 20.0) {
                FSM_Goto(batteryFsm, BAT_CHARGING);
                EV << "Node " << myAddress << ": battery low ("
                   << batteryLevel << "%), entering CHARGING state\n";
            }
            break;

        case BAT_CHARGING:
            batteryLevel += uniform(0.2, 0.5);
            if (batteryLevel > 100.0)
                batteryLevel = 100.0;

            if (batteryLevel >= 100.0) {
                FSM_Goto(batteryFsm, BAT_ACTIVE);
                EV << "Node " << myAddress
                   << ": battery full, returning to ACTIVE state\n";
            }
            break;
    }

    scheduleAt(simTime() + 1, batteryTimer);
}

// CHANGE 10: centralised helper for per-packet drain + state transitions
void Routing::updateBatteryOnActivity(double minDrain, double maxDrain)
{
    if (batteryFsm.getState() != BAT_ACTIVE)
        return;

    double delta = uniform(minDrain, maxDrain);
    batteryLevel -= delta;

    if (batteryLevel < 0)
        batteryLevel = 0;

    if (batteryLevel == 0) {
        if (batteryFsm.getState() != BAT_CHARGING) {
            FSM_Goto(batteryFsm, BAT_CHARGING);
            EV << "Node " << myAddress
               << ": battery depleted to 0%, entering CHARGING state\n";
        }
    }
    else if (batteryLevel < 20.0 && batteryFsm.getState() == BAT_ACTIVE) {
        FSM_Goto(batteryFsm, BAT_CHARGING);
        EV << "Node " << myAddress
           << ": battery low (" << batteryLevel
           << "%), entering CHARGING state\n";
    }
}

// CHANGE 11: finish() now reports the FSM state label, not just the %
void Routing::finish()
{
    const char *stateName = "unknown";
    switch (batteryFsm.getState()) {
        case BAT_ACTIVE:   stateName = "ACTIVE";   break;
        case BAT_CHARGING: stateName = "CHARGING"; break;
        default: break;
    }

    EV << "Node " << myAddress << ": Final battery level = "
       << batteryLevel << "%, state = " << stateName << "\n";
}
