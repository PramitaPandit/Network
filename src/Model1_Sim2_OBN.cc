/*
 * my_simulation_network.cc
 *
 *  Created on: Feb 1, 2024
 *      Author: pramita
 */

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include "KalmanFilter.h"
#include "SensorNodeMsg_m.h" // Include the generated message
#include "DataAggregationMessage_m.h" // Include the generated DataAggregationMessage

using namespace omnetpp;

// Constants for normalization
const double HR_MIN = 40.0; // Placeholder
const double HR_MAX = 200.0; // Placeholder

// Assuming the KalmanFilter class has a constructor that takes matrices
const std::vector<std::vector<double>> F = {{1.0, 0.0}, {0.0, 1.0}}; // Example state transition matrix
const std::vector<std::vector<double>> H = {{1.0, 0.0}}; // Example observation matrix
const std::vector<std::vector<double>> Q = {{0.1, 0.0}, {0.0, 0.1}}; // Process noise covariance
const std::vector<std::vector<double>> R = {{0.1}}; // Measurement noise covariance
const std::vector<std::vector<double>> P0 = {{1.0, 0.0}, {0.0, 1.0}}; // Initial state covariance
const std::vector<double> x0 = {0.0, 0.0}; // Initial state

class OBN_node : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    void handleAggregatedData(cMessage *msg);
    virtual void handleSensorData(int sourceNodeId, double receivedValue, KalmanFilter &kf, DataAggregationMessage *dataAggregationMsg);
    virtual void sendConfigAndEnergy();
    void processData(double HR_Node11, double HR_Node12, double HR_Node21, double HR_Node22, double HR_Node31, double HR_Node32);

    int nodeId;
    int numDataAggregationMessagesReceived = 0;
    bool simulationEnded = false;

    KalmanFilter kf_hub1_node11;
    KalmanFilter kf_hub1_node12;
    KalmanFilter kf_hub2_node21;
    KalmanFilter kf_hub2_node22;
    KalmanFilter kf_hub3_node31;
    KalmanFilter kf_hub3_node32;

    double HR_Node11;
    double HR_Node12;
    double HR_Node21;
    double HR_Node22;
    double HR_Node31;
    double HR_Node32;

    int hubNodeCount = 3;
    int endSimMsgCount;

public:
    OBN_node()
        : kf_hub1_node11(F, H, Q, R, P0, x0),
          kf_hub1_node12(F, H, Q, R, P0, x0),
          kf_hub2_node21(F, H, Q, R, P0, x0),
          kf_hub2_node22(F, H, Q, R, P0, x0),
          kf_hub3_node31(F, H, Q, R, P0, x0),
          kf_hub3_node32(F, H, Q, R, P0, x0),
          HR_Node11(0),
          HR_Node12(0),
          HR_Node21(0),
          HR_Node22(0),
          HR_Node31(0),
          HR_Node32(0),
          hubNodeCount(0),
          endSimMsgCount(0){}
};

Define_Module(OBN_node);

void OBN_node::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    // Send initial config and energy message
    sendConfigAndEnergy();
}

void OBN_node::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (strcmp(msg->getName(), "DataAggregation") == 0) {
        EV << "Receiving Aggregated data from hub nodes at " << simTime() << "\n";
        handleAggregatedData(msg);

    } else if (strcmp(msg->getName(), "EndSimulation") == 0) {
        endSimMsgCount++;
        EV << "OBN received EndSimulation message. Count: " << endSimMsgCount << "\n";

        if (endSimMsgCount >= hubNodeCount) {
            EV << "All Hub nodes have signaled end of simulation. Ending simulation.\n";
            endSimulation();
        }

        delete msg;
    } else {
        EV << "Unhandled message type: " << msg->getName() << " from " << msg->getSenderModule()->getName() << " at time " << simTime() << "\n";
    }
}
void OBN_node::handleAggregatedData(cMessage *msg) {
    DataAggregationMessage *dataAggregationMsg = dynamic_cast<DataAggregationMessage *>(msg);
    if (!dataAggregationMsg) {
        EV << "Received message is not of type DataAggregationMessage.\n";
        delete msg;
        return;
    }

    int sourceNodeId = dataAggregationMsg->getSourceNodeId();
    EV << "sourceNodeId: " << sourceNodeId << "\n";

    if (sourceNodeId == 3) {
        double aggregatedValueNode11 = dataAggregationMsg->getAggregatedValueNode11();
        EV << "Received Value from Node11: " << aggregatedValueNode11 << "\n";
        handleSensorData(3, aggregatedValueNode11, kf_hub1_node11, dataAggregationMsg);
        bubble("Message Received from Hub_Node1 (Node11)");

        double aggregatedValueNode12 = dataAggregationMsg->getAggregatedValueNode12();
        EV << "Received Value from Node12: " << aggregatedValueNode12 << "\n";
        handleSensorData(3, aggregatedValueNode12, kf_hub1_node12, dataAggregationMsg);
        bubble("Message Received from Hub_Node1 (Node12)");
    }

    if (sourceNodeId == 4) {
        double aggregatedValueNode21 = dataAggregationMsg->getAggregatedValueNode21();
        EV << "Received Value from Node21: " << aggregatedValueNode21 << "\n";
        handleSensorData(4, aggregatedValueNode21, kf_hub2_node21, dataAggregationMsg);
        bubble("Message Received from Hub_Node2 (Node21)");

        double aggregatedValueNode22 = dataAggregationMsg->getAggregatedValueNode22();
        EV << "Received Value from Node22: " << aggregatedValueNode22 << "\n";
        handleSensorData(4, aggregatedValueNode22, kf_hub2_node22, dataAggregationMsg);
        bubble("Message Received from Hub_Node2 (Node22)");
    }

    if (sourceNodeId == 5) {
        double aggregatedValueNode31 = dataAggregationMsg->getAggregatedValueNode31();
        EV << "Received Value from Node31: " << aggregatedValueNode31 << "\n";
        handleSensorData(5, aggregatedValueNode31, kf_hub3_node31, dataAggregationMsg);
        bubble("Message Received from Hub_Node3 (Node31)");

        double aggregatedValueNode32 = dataAggregationMsg->getAggregatedValueNode32();
        EV << "Received Value from Node32: " << aggregatedValueNode32 << "\n";
        handleSensorData(5, aggregatedValueNode32, kf_hub3_node32, dataAggregationMsg);
        bubble("Message Received from Hub_Node3 (Node32)");
    }

    delete dataAggregationMsg;
}

void OBN_node::handleSensorData(int sourceNodeId, double receivedValue, KalmanFilter &kf, DataAggregationMessage *dataAggregationMsg) {
    EV << "Handling data reception for Node " << sourceNodeId << "\n";

    double predictedValue, filteredValue, predictionError;

    // Perform prediction
    kf.predict();
    predictedValue = kf.getState(); // Access the predicted state
    EV << "OBN Predicted state: " << predictedValue << "\n";

    // Perform update with the received value
    filteredValue = kf.update(receivedValue);
    EV << "Filtered value after update: " << filteredValue << "\n";

    // Log the predicted state and received value
    EV << "Received value from Node " << sourceNodeId << ": " << receivedValue << ", Predicted value: " << predictedValue << ", Filtered value: " << filteredValue << "\n";

    // Calculate prediction error
    predictionError = receivedValue - predictedValue; // Calculate the error
    EV << "Prediction Error: " << predictionError << "\n";

    // Store the filtered value in the appropriate variable based on the source node ID
    if (sourceNodeId == 3) {
        HR_Node11 = filteredValue;
        EV << "HR_Node11: " << HR_Node11 << "\n";

        HR_Node12 = filteredValue;
        EV << "HR_Node12: " << HR_Node12 << "\n";

    } else if (sourceNodeId == 4) {
        HR_Node21 = filteredValue;
        EV << "HR_Node21: " << HR_Node21 << "\n";

        HR_Node22 = filteredValue;
        EV << "HR_Node22: " << HR_Node22 << "\n";

    } else if (sourceNodeId == 5) {
        HR_Node31 = filteredValue;
        EV << "HR_Node31: " << HR_Node31 << "\n";

        HR_Node32 = filteredValue;
        EV << "HR_Node32: " << HR_Node32 << "\n";
    }

    // Process data
    processData(HR_Node11, HR_Node12, HR_Node21, HR_Node22, HR_Node31, HR_Node32);
}


void OBN_node::processData(double HR_Node11, double HR_Node12, double HR_Node21, double HR_Node22, double HR_Node31, double HR_Node32) {
    // Denormalize the HR values
    double denormalizedHR_Node11 = HR_Node11 * (HR_MAX - HR_MIN) + HR_MIN;
    double denormalizedHR_Node12 = HR_Node12 * (HR_MAX - HR_MIN) + HR_MIN;
    double denormalizedHR_Node21 = HR_Node21 * (HR_MAX - HR_MIN) + HR_MIN;
    double denormalizedHR_Node22 = HR_Node22 * (HR_MAX - HR_MIN) + HR_MIN;
    double denormalizedHR_Node31 = HR_Node31 * (HR_MAX - HR_MIN) + HR_MIN;
    double denormalizedHR_Node32 = HR_Node32 * (HR_MAX - HR_MIN) + HR_MIN;

    // Compute average denormalized HR
    double averageHR = (denormalizedHR_Node11 + denormalizedHR_Node12 + denormalizedHR_Node21 + denormalizedHR_Node22 + denormalizedHR_Node31 + denormalizedHR_Node32) / 6.0;
    const double normalHRRangeMin = 60.0;
    const double normalHRRangeMax = 100.0;

    // Check if average HR is within the normal range
    if (averageHR >= normalHRRangeMin && averageHR <= normalHRRangeMax) {
        EV << "Average HR: " << averageHR << " is within the normal range.\n";
        bubble("Average HR is within the normal range.");
    } else {
        EV << "Average HR: " << averageHR << " is outside the normal range.\n";
        bubble("Average HR is outside the normal range.");
    }
}

void OBN_node::sendConfigAndEnergy() {
    EV << "Sending configuration parameters and energy to hub nodes and sensor nodes at " << simTime() << "\n";

    cMessage *configEnergyMsg = new cMessage("ConfigEnergy");
    configEnergyMsg->addPar("configParam1") = 42;
    configEnergyMsg->addPar("energyLevel") = 100.0;

    for (int i = 0; i < gateSize("out"); ++i) {
        send(configEnergyMsg->dup(), "out", i);
    }

    delete configEnergyMsg;
}
