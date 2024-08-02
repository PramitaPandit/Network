/*
 * my_simulation_network2.cc
 *
 *  Created on: July 8, 2024
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
#include "FeedbackThresholdMsg_m.h"
#include "DataAggregationMessage_m.h" // Include the generated DataAggregationMessage


using namespace omnetpp;

// Assuming the KalmanFilter class has a constructor t  hat takes matrices

const std::vector<std::vector<double>> F = {{1.0, 0.0}, {0.0, 1.0}}; // 2x2 state transition matrix
const std::vector<std::vector<double>> H = {{1.0, 0.0}}; // 1x2 observation matrix
const std::vector<std::vector<double>> Q = {{0.1, 0.0}, {0.0, 0.1}}; // 2x2 Process noise covariance
const std::vector<std::vector<double>> R = {{0.1}}; // 1x1 Measurement noise covariance
const std::vector<std::vector<double>> P0 = {{1.0, 0.0}, {0.0, 1.0}}; // 2x2 Initial state covariance
const std::vector<double> x0 = {0.0, 0.0}; // 2x1 Initial State

class Hub_Node1 : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void handleSensorDataReception(cMessage *msg);
    virtual void handleDataAggregation(double observationSum_node11, double observationSum_node12);
    virtual void adjustThresholdValue(double error, int nodeId);
    virtual void sendFeedbackThresholdValue(double eps, int nodeId);
    virtual void handleConfigEnergyMessage(cMessage *msg);

    int nodeId;
    double Δ_D, Δ_M;
    double eps; // Threshold
    double tol_KF; // Tolerance for Kalman filter
    double tol_eps; // Tolerance for threshold adjustment
    double min_eps;
    double max_eps;
    double K;

    KalmanFilter kf_node11; // Kalman Filter for node11
    KalmanFilter kf_node12; // Kalman Filter for node12
    std::vector<double> data_values_node11;
    std::vector<double> data_values_node12;

    double observationSum_node11 = 0;
    double observationSum_node12 = 0;

    bool node11Received = false; // Flag to indicate if Node11 data is received
    bool node12Received = false; // Flag to indicate if Node12 data is received

    bool configReceived = false; // Flag to track receipt of ConfigEnergy message
    bool firstSensorMessageReceived = false; // Flag to track receipt of first sensor message

    int sensorNodeCount;
    int endSimMsgCount;
    int hubNodeId;
    int obnId; // OBN node ID

    std::vector<double> timeStamps;
    std::vector<double> predictedValues_node11;
    std::vector<double> receivedValues_node11;
    std::vector<double> predictionErrors_node11;
    std::vector<int> thresholdChanges_node11;

    std::vector<double> predictedValues_node12;
    std::vector<double> receivedValues_node12;
    std::vector<double> predictionErrors_node12;
    std::vector<int> thresholdChanges_node12;


public:
    Hub_Node1()
        : kf_node11(F, H, Q, R, P0, x0),
          kf_node12(F, H, Q, R, P0, x0),
          observationSum_node11(0.0),
          observationSum_node12(0.0),
          sensorNodeCount(0), endSimMsgCount(0), hubNodeId(-1), obnId(-1) {}
};

Define_Module(Hub_Node1);

void Hub_Node1::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    Δ_D = par("Δ_D").doubleValue();
    Δ_M = par("Δ_M").doubleValue();
    eps = par("eps").doubleValue();
    tol_KF = par("tol_KF").doubleValue();
    tol_eps = par("tol_eps").doubleValue();
    min_eps = par("min_eps").doubleValue();
    max_eps = par("max_eps").doubleValue();
    K = par("K").doubleValue();

    hubNodeId = getId();
    obnId = 2; // ID of the OBN node
  //  sensorNodeCount = par("sensorNodeCount").intValue(); // Number of sensor nodes
    sensorNodeCount = 6; // Number of sensor nodes

    endSimMsgCount = 0;



    for (int i = 0; i < gateSize("in"); i++) {
        gate("in", i)->setDeliverImmediately(true); // Ensure packets are delivered at the start of reception
    }
}

void Hub_Node1::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (!configReceived) {
        if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
            handleConfigEnergyMessage(msg);
            configReceived = true;
            EV << "Config and energy message received at Hub_Node1, initializing timers.\n";
        } else {
            EV << "Waiting for ConfigEnergy message.\n";
            delete msg; // Discard any other messages until config is received
        }
        return;
    } else if (strcmp(msg->getName(), "RegComm") == 0 || strcmp(msg->getName(), "MandComm") == 0) {
        firstSensorMessageReceived = true;
        handleSensorDataReception(msg);

    } else if (strcmp(msg->getName(), "EndSimulation") == 0) {
        endSimMsgCount++;
        EV << "Hub node " << hubNodeId << " received EndSimulation message. Count: " << endSimMsgCount << "\n";

        if (endSimMsgCount >= sensorNodeCount) {
            // Send EndSimulation message to OBN
            SensorNodeMsg *endSimMsg = new SensorNodeMsg("EndSimulation");
            endSimMsg->setSourceNodeId(hubNodeId);
            endSimMsg->setDestinationNodeId(obnId);

            send(endSimMsg, "out", 2); // Sending message to OBN
            EV << "Hub node " << hubNodeId << " sent EndSimulation message to OBN\n";
        }

        delete msg;
    } else {
        EV << "Unhandled message type: " << msg->getName() << "\n";
    }
}

void Hub_Node1::handleSensorDataReception(cMessage *msg) {
    EV << "Handling RegComm/MandComm reception.\n";

    // Type cast the message to SensorNodeMsg for direct access
    SensorNodeMsg *sensorMsg = dynamic_cast<SensorNodeMsg*>(msg);
    if (!sensorMsg) {
        EV << "Received message is not of type SensorNodeMsg.\n";
        delete msg;
        return;
    }

    double receivedValue = sensorMsg->getMean();
    int sourceNodeId = sensorMsg->getSourceNodeId();

    double filteredValue;
    double predictedValue;

    if (sourceNodeId == 6) {
        // Handle node11
        EV << "Hub_Node1 (Node11) -> Received value from Node_11: " << receivedValue << "\n";

        kf_node11.predict();
        predictedValue = kf_node11.getState(); // Access the predicted state
        EV << "Hub_Node1 (Node11) -> Predicted state: " << predictedValue << "\n";

        filteredValue = kf_node11.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node11.push_back(filteredValue);
        EV << "Hub_Node1 (Node11) -> Received value from Node_11: " << receivedValue << ", Predicted value: " << predictedValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node11!");

        double error = receivedValue - predictedValue; // Calculate the error
        adjustThresholdValue(error, 6); // Pass error and predicted value to adjustThresholdValue

        timeStamps.push_back(simTime().dbl());
        predictedValues_node11.push_back(predictedValue);
        receivedValues_node11.push_back(receivedValue);
        predictionErrors_node11.push_back(error);

        if (fabs(error) > tol_KF) {
            observationSum_node11 += receivedValue;
            EV << "observationSum_node11: " << receivedValue << "\n";
        } else {
            observationSum_node11 += filteredValue;
            EV << "observationSum_node11: " << filteredValue << "\n";
        }

        // Flag that Node11 has provided data
        node11Received = true;

    } else if (sourceNodeId == 7) {
        // Handle node12
        EV << "Hub_Node1 (Node12) ->Received value from Node_12: " << receivedValue << "\n";

        kf_node12.predict();
        predictedValue = kf_node12.getState(); // Access the predicted state
        EV << " Hub_Node1 (Node12) -> Predicted state: " << predictedValue << "\n";

        filteredValue = kf_node12.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node12.push_back(filteredValue);
        EV << "Hub_Node1 (Node12) -> Received value from Node_12: " << receivedValue << ", Predicted value: " << predictedValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node12!");

        double error = receivedValue - predictedValue; // Calculate the error
        adjustThresholdValue(error, 7); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node12 += receivedValue;
            EV << "observationSum_node12: " << receivedValue << "\n";
        } else {
            observationSum_node12 += filteredValue;
            EV << "observationSum_node12: " << filteredValue << "\n";
        }

        // Flag that Node12 has provided data
        node12Received = true;

    } else {
        EV << "Unknown sourceNodeId for sensor data reception: " << sourceNodeId << "\n";
    }

    // Check if data from both Node11 and Node12 have been received
    if (node11Received && node12Received) {
        handleDataAggregation(observationSum_node11, observationSum_node12);

        // Reset flags and sums for the next aggregation cycle
        node11Received = false;
        node12Received = false;
        observationSum_node11 = 0.0;
        observationSum_node12 = 0.0;
    }

    delete msg; // Clean up message
}

void Hub_Node1::adjustThresholdValue(double error, int nodeId) {
    EV << "AdjustThresholdValue: Initial threshold: " << eps
       << ", Error: " << error << " for " << nodeId << "\n";

    if (fabs(error) > tol_KF) {
        double eps_new = eps * (1 + K * error);
        EV << "Value of K: " << K << "\n";
        EV << "Value of tol_KF: " << tol_KF << "\n";
        EV << "Value of error: " << error << "\n";
        eps_new = std::max(min_eps, std::min(eps_new, max_eps));
        EV << "Value of min_eps: " << min_eps << "\n";
        EV << "Value of max_eps: " << max_eps << "\n";
        EV << "Value of eps_new: " << eps_new << " for Node: " << nodeId << "\n";
        double diff = std::abs(eps_new - eps);
        EV << "Value of diff: " << diff << "\n";
        if (diff <= tol_eps) {
            eps = eps_new;
            EV << "Threshold adjusted to: " << eps << "\n";
            sendFeedbackThresholdValue(eps, nodeId); // Send the adjusted threshold value back to the sensor node
            EV << "AdjustThresholdValue: New threshold: " << eps << " for Node: " << nodeId << "\n";
            EV << "Feedback message sent" << " for node: " << nodeId << "\n";
        } else {
            EV << "Threshold adjustment skipped " << " for node: " << nodeId << " Difference (" << diff << ") exceeds tolerance (" << tol_eps << ").\n";
            EV << "AdjustThresholdValue: No change in threshold value" << " for Node: " << nodeId << "Error (" << error << ") is within tolerance (" << tol_KF << ").\n";
        }
    }
}

void Hub_Node1::sendFeedbackThresholdValue(double eps, int nodeId) {
    EV << "Sending feedback threshold value: " << eps << " to Node: " << nodeId << "\n";
    FeedbackThresholdMsg *feedbackMsg = new FeedbackThresholdMsg("FeedbackThreshold");
    feedbackMsg->setThreshold(eps);

    if (nodeId == 6) {
        send(feedbackMsg, "out", 0); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << eps << " to Node: " << nodeId << "\n";
    } else if (nodeId == 7) {
        send(feedbackMsg, "out", 1); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << eps << " to Node: " << nodeId << "\n";
    } else {
        EV << "Unknown nodeId for feedback message: " << nodeId << "\n";
        delete feedbackMsg;
    }
}


void Hub_Node1::handleDataAggregation(double observationSum_node11, double observationSum_node12) {
    EV << "Aggregating data for OBN from Hub_Node1!" << "\n";

    double aggregatedValue_node11 = observationSum_node11;
    double aggregatedValue_node12 = observationSum_node12;

    EV << "Aggregated value for Node 11: " << aggregatedValue_node11 << "\n";
    EV << "Aggregated value for Node 12: " << aggregatedValue_node12 << "\n";

    // Create a new DataAggregationMessage object
    DataAggregationMessage *dataAggregationMsg = new DataAggregationMessage("DataAggregation");
    dataAggregationMsg->setSourceNodeId(getId());
    dataAggregationMsg->setAggregatedValueNode11(aggregatedValue_node11);
    dataAggregationMsg->setAggregatedValueNode12(aggregatedValue_node12);

    send(dataAggregationMsg, "out", 2); // Send the aggregated data message to the OBN node
    EV << "DataAggregation message sent with aggregated values to the OBN node from Hub_Node1.\n";
    bubble("AggregationMessage Sent from Hub_Node1!");
}

void Hub_Node1::handleConfigEnergyMessage(cMessage *msg) {
    EV << "Handling config and energy message in Hub_Node1.\n";
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param 1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // Handle configuration and energy information as needed
    // For example, store values for later use, update internal state, etc.
}


class Hub_Node2 : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void handleSensorDataReception(cMessage *msg);
    virtual void handleDataAggregation(double observationSum_node21, double observationSum_node22);
    virtual void adjustThresholdValue(double error, double filteredValue, int nodeId);
    virtual void sendFeedbackThresholdValue(double threshold, int nodeId);
    virtual void handleConfigEnergyMessage(cMessage *msg);

    int nodeId;
    double Δ_D, Δ_M;
    double eps; // Threshold
    double tol_KF; // Tolerance for Kalman filter
    double tol_eps; // Tolerance for threshold adjustment
    double min_eps;
    double max_eps;
    double K;

    KalmanFilter kf_node21; // Kalman Filter for node21
    KalmanFilter kf_node22; // Kalman Filter for node22
    std::vector<double> data_values_node21;
    std::vector<double> data_values_node22;

    double observationSum_node21 = 0;
    double observationSum_node22 = 0;

    bool node21Received = false; // Flag to indicate if Node21 data is received
    bool node22Received = false; // Flag to indicate if Node22 data is received

    bool configReceived = false; // Flag to track receipt of ConfigEnergy message
    bool firstSensorMessageReceived = false; // Flag to track receipt of first sensor message

public:
    Hub_Node2()
        : kf_node21(F, H, Q, R, P0, x0),
          kf_node22(F, H, Q, R, P0, x0),
          observationSum_node21(0.0),
          observationSum_node22(0.0) {}
};

Define_Module(Hub_Node2);

void Hub_Node2::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    Δ_D = par("Δ_D").doubleValue();
    Δ_M = par("Δ_M").doubleValue();
    eps = par("eps").doubleValue();
    tol_KF = par("tol_KF").doubleValue();
    tol_eps = par("tol_eps").doubleValue();
    min_eps = par("min_eps").doubleValue();
    max_eps = par("max_eps").doubleValue();
    K = par("K").doubleValue();

    for (int i = 0; i < gateSize("in"); i++) {
        gate("in", i)->setDeliverImmediately(true); // Ensure packets are delivered at the start of reception
    }
}

void Hub_Node2::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (!configReceived) {
        if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
            handleConfigEnergyMessage(msg);
            configReceived = true;
            EV << "Config and energy message received at Hub_Node2, initializing timers.\n";
        } else {
            EV << "Waiting for ConfigEnergy message.\n";
            delete msg; // Discard any other messages until config is received
        }
        return;
    } else if (strcmp(msg->getName(), "RegComm") == 0 || strcmp(msg->getName(), "MandComm") == 0) {
        firstSensorMessageReceived = true;
        handleSensorDataReception(msg);
    } else {
        EV << "Unhandled message type: " << msg->getName() << "\n";
    }
}

void Hub_Node2::handleSensorDataReception(cMessage *msg) {
    EV << "Handling RegComm/MandComm reception.\n";

    SensorNodeMsg *sensorMsg = dynamic_cast<SensorNodeMsg*>(msg);
    if (!sensorMsg) {
        EV << "Received message is not of type SensorNodeMsg.\n";
        delete msg;
        return;
    }

    double receivedValue = sensorMsg->getMean();
    int sourceNodeId = sensorMsg->getSourceNodeId();

    double filteredValue;
    double predictedValue;

    if (sourceNodeId == 8) {
        // Handle node21
        EV << "Received value from Node_21: " << receivedValue << "\n";

        kf_node21.predict();
        predictedValue = kf_node21.getState(); // Access the predicted state

        filteredValue = kf_node21.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node21.push_back(filteredValue);
        EV << "Predicted state: " << filteredValue << "\n";
        EV << "Received value from Node_21: " << receivedValue << ", Predicted value: " << filteredValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node21!");

        double error = receivedValue - predictedValue; // Calculate the error
        adjustThresholdValue(error, predictedValue, 8); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node21 += receivedValue;
            EV << "observationSum_node21: " << receivedValue << "\n";
        } else {
            observationSum_node21 += filteredValue;
            EV << "observationSum_node21: " << filteredValue << "\n";
        }

        // Flag that Node21 has provided data
        node21Received = true;

    } else if (sourceNodeId == 9) {
        // Handle node22
        EV << "Received value from Node_22: " << receivedValue << "\n";

        kf_node22.predict();
        predictedValue = kf_node22.getState(); // Access the predicted state

        filteredValue = kf_node22.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node22.push_back(filteredValue);
        EV << "Predicted state: " << filteredValue << "\n";
        EV << "Received value from Node_22: " << receivedValue << ", Predicted value: " << filteredValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node22!");

        double error = receivedValue - filteredValue; // Calculate the error
        adjustThresholdValue(error, filteredValue, 9); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node22 += receivedValue;
            EV << "observationSum_node22: " << receivedValue << "\n";
        } else {
            observationSum_node22 += filteredValue;
            EV << "observationSum_node22: " << filteredValue << "\n";
        }

        // Flag that Node22 has provided data
        node22Received = true;

    } else {
        EV << "Unknown sourceNodeId for sensor data reception: " << sourceNodeId << "\n";
    }

    // Check if data from both Node21 and Node22 have been received
    if (node21Received && node22Received) {
        handleDataAggregation(observationSum_node21, observationSum_node22);

        // Reset flags and sums for the next aggregation cycle
        node21Received = false;
        node22Received = false;
        observationSum_node21 = 0.0;
        observationSum_node22 = 0.0;
    }

    delete msg; // Clean up message
}

void Hub_Node2::adjustThresholdValue(double error, double filteredValue, int nodeId) {

    EV << "AdjustThresholdValue: Initial threshold: " << eps
       << ", Error: " << error << ", Predicted Value (State): " << filteredValue << "\n";

    if (fabs(error) > tol_KF) {
        double eps_new = eps * (1 + K * error);
        EV << "Value of K: " << K << "\n";
        EV << "Value of tol_KF: " << tol_KF << "\n";
        EV << "Value of error: " << error << "\n";
        eps_new = std::max(min_eps, std::min(eps_new, max_eps));
        EV << "Value of min_eps: " << min_eps << "\n";
        EV << "Value of max_eps: " << max_eps << "\n";
        EV << "Value of eps_new: " << eps_new << "\n";
        double diff = std::abs(eps_new - eps);
        EV << "Value of diff: " << diff << "\n";
        if (diff <= tol_eps) {
            eps = eps_new;
            EV << "Threshold adjusted to: " << eps << "\n";
            sendFeedbackThresholdValue(eps, nodeId); // Send the adjusted threshold value back to the sensor node
            EV << "AdjustThresholdValue: New threshold: " << eps << "\n";
            EV << "Feedback message sent.\n";
        } else {
            EV << "Threshold adjustment skipped. Difference (" << diff << ") exceeds tolerance (" << tol_eps << ").\n";
            EV << "AdjustThresholdValue: No change in threshold value. Error (" << error << ") is within tolerance (" << tol_KF << ").\n";
        }
    }
}

void Hub_Node2::sendFeedbackThresholdValue(double threshold, int nodeId) {
    EV << "Sending feedback threshold value: " << threshold << " to Node " << nodeId << "\n";
    FeedbackThresholdMsg *feedbackMsg = new FeedbackThresholdMsg("FeedbackThreshold");
    feedbackMsg->setThreshold(threshold);
//    feedbackMsg->setSourceNodeId(getId());
//    feedbackMsg->setDestinationNodeId(nodeId);

    if (nodeId == 8) {
        send(feedbackMsg, "out", 0); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << threshold << " to node: " << nodeId << "\n";
    } else if (nodeId == 9) {
        send(feedbackMsg, "out", 1); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << threshold << " to node: " << nodeId << "\n";
    } else {
        EV << "Unknown nodeId for feedback message: " << nodeId << "\n";
        delete feedbackMsg;
    }
}

void Hub_Node2::handleDataAggregation(double observationSum_node21, double observationSum_node22) {
    EV << "Aggregating data for OBN from Hub_Node1!" << "\n";

    double aggregatedValue_node21 = observationSum_node21;
    double aggregatedValue_node22 = observationSum_node22;

    EV << "Aggregated value for Node 21: " << aggregatedValue_node21 << "\n";
    EV << "Aggregated value for Node 22: " << aggregatedValue_node22 << "\n";

    // Create a new DataAggregationMessage object
    DataAggregationMessage *dataAggregationMsg = new DataAggregationMessage("DataAggregation");
    dataAggregationMsg->setSourceNodeId(getId());
    dataAggregationMsg->setAggregatedValueNode21(aggregatedValue_node21);
    dataAggregationMsg->setAggregatedValueNode22(aggregatedValue_node22);

    send(dataAggregationMsg, "out", 2); // Send the aggregated data message to the OBN node
    EV << "DataAggregation message sent with aggregated values to the OBN node from Hub_Node1.\n";
    bubble("AggregationMessage Sent from Hub_Node1!");
}


    void Hub_Node2::handleConfigEnergyMessage(cMessage *msg) {
    EV << "Handling config and energy message in Hub_Node2.\n";
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param 1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // Handle configuration and energy information as needed
    // For example, store values for later use, update internal state, etc.
}



class Hub_Node3 : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void handleSensorDataReception(cMessage *msg);
    virtual void handleDataAggregation(double observationSum_node31, double observationSum_node32);
    virtual void adjustThresholdValue(double error, double filteredValue, int nodeId);
    virtual void sendFeedbackThresholdValue(double threshold, int nodeId);
    virtual void handleConfigEnergyMessage(cMessage *msg);

    int nodeId;
    double Δ_D, Δ_M;
    double eps; // Threshold
    double tol_KF; // Tolerance for Kalman filter
    double tol_eps; // Tolerance for threshold adjustment
    double min_eps;
    double max_eps;
    double K;

    KalmanFilter kf_node31; // Kalman Filter for node31
    KalmanFilter kf_node32; // Kalman Filter for node32
    std::vector<double> data_values_node31;
    std::vector<double> data_values_node32;

    double observationSum_node31 = 0;
    double observationSum_node32 = 0;

    bool node31Received = false; // Flag to indicate if Node31 data is received
    bool node32Received = false; // Flag to indicate if Node32 data is received

    bool configReceived = false; // Flag to track receipt of ConfigEnergy message
    bool firstSensorMessageReceived = false; // Flag to track receipt of first sensor message

public:
    Hub_Node3()
        : kf_node31(F, H, Q, R, P0, x0),
          kf_node32(F, H, Q, R, P0, x0),
          observationSum_node31(0.0),
          observationSum_node32(0.0) {}
};

Define_Module(Hub_Node3);

void Hub_Node3::initialize() {
    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    EV << "Node " << nodeId << " initialized\n";

    Δ_D = par("Δ_D").doubleValue();
    Δ_M = par("Δ_M").doubleValue();
    eps = par("eps").doubleValue();
    tol_KF = par("tol_KF").doubleValue();
    tol_eps = par("tol_eps").doubleValue();
    min_eps = par("min_eps").doubleValue();
    max_eps = par("max_eps").doubleValue();
    K = par("K").doubleValue();

    for (int i = 0; i < gateSize("in"); i++) {
        gate("in", i)->setDeliverImmediately(true); // Ensure packets are delivered at the start of reception
    }
}

void Hub_Node3::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (!configReceived) {
        if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
            handleConfigEnergyMessage(msg);
            configReceived = true;
            EV << "Config and energy message received at Hub_Node3, initializing timers.\n";
        } else {
            EV << "Waiting for ConfigEnergy message.\n";
            delete msg; // Discard any other messages until config is received
        }
        return;
    } else if (strcmp(msg->getName(), "RegComm") == 0 || strcmp(msg->getName(), "MandComm") == 0) {
        firstSensorMessageReceived = true;
        handleSensorDataReception(msg);
    } else {
        EV << "Unhandled message type: " << msg->getName() << "\n";
    }
}

void Hub_Node3::handleSensorDataReception(cMessage *msg) {
    EV << "Handling RegComm/MandComm reception.\n";

    SensorNodeMsg *sensorMsg = dynamic_cast<SensorNodeMsg*>(msg);
    if (!sensorMsg) {
        EV << "Received message is not of type SensorNodeMsg.\n";
        delete msg;
        return;
    }

    double receivedValue = sensorMsg->getMean();
    int sourceNodeId = sensorMsg->getSourceNodeId();

    double filteredValue;
    double predictedValue;

    if (sourceNodeId == 10) {
        // Handle node31
        EV << "Received value from Node_31: " << receivedValue << "\n";

        kf_node31.predict();
        predictedValue = kf_node31.getState(); // Access the predicted state

        filteredValue = kf_node31.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node31.push_back(filteredValue);
        EV << "Predicted state: " << filteredValue << "\n";
        EV << "Received value from Node_31: " << receivedValue << ", Predicted value: " << filteredValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node31!");

        double error = receivedValue - predictedValue; // Calculate the error
        adjustThresholdValue(error, predictedValue, 10); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node31 += receivedValue;
            EV << "observationSum_node31: " << receivedValue << "\n";
        } else {
            observationSum_node31 += filteredValue;
            EV << "observationSum_node31: " << filteredValue << "\n";
        }

        // Flag that Node31 has provided data
        node31Received = true;

    } else if (sourceNodeId == 11) {
        // Handle node32
        EV << "Received value from Node_32: " << receivedValue << "\n";

        kf_node32.predict();
        predictedValue = kf_node32.getState(); // Access the predicted state

        filteredValue = kf_node32.update(receivedValue);
        EV << "Filtered value after update: " << filteredValue << "\n";

        data_values_node32.push_back(filteredValue);
        EV << "Predicted state: " << filteredValue << "\n";
        EV << "Received value from Node_32: " << receivedValue << ", Predicted value: " << filteredValue << ", Filtered value: " << filteredValue << "\n";
        bubble("Message Received from Node32!");

        double error = receivedValue - filteredValue; // Calculate the error
        adjustThresholdValue(error, filteredValue, 11); // Pass error and predicted value to adjustThresholdValue

        if (fabs(error) > tol_KF) {
            observationSum_node32 += receivedValue;
            EV << "observationSum_node32: " << receivedValue << "\n";
        } else {
            observationSum_node32 += filteredValue;
            EV << "observationSum_node32: " << filteredValue << "\n";
        }

        // Flag that Node32 has provided data
        node32Received = true;

    } else {
        EV << "Unknown sourceNodeId for sensor data reception: " << sourceNodeId << "\n";
    }

    // Check if data from both Node31 and Node32 have been received
    if (node31Received && node32Received) {
        handleDataAggregation(observationSum_node31, observationSum_node32);

        // Reset flags and sums for the next aggregation cycle
        node31Received = false;
        node32Received = false;
        observationSum_node31 = 0.0;
        observationSum_node32 = 0.0;
    }

    delete msg; // Clean up message
}

void Hub_Node3::adjustThresholdValue(double error, double filteredValue, int nodeId) {

    EV << "AdjustThresholdValue: Initial threshold: " << eps
       << ", Error: " << error << ", Predicted Value (State): " << filteredValue << "\n";

    if (fabs(error) > tol_KF) {
        double eps_new = eps * (1 + K * error);
        EV << "Value of K: " << K << "\n";
        EV << "Value of tol_KF: " << tol_KF << "\n";
        EV << "Value of error: " << error << "\n";
        eps_new = std::max(min_eps, std::min(eps_new, max_eps));
        EV << "Value of min_eps: " << min_eps << "\n";
        EV << "Value of max_eps: " << max_eps << "\n";
        EV << "Value of eps_new: " << eps_new << "\n";
        double diff = std::abs(eps_new - eps);
        EV << "Value of diff: " << diff << "\n";
        if (diff <= tol_eps) {
            eps = eps_new;
            EV << "Threshold adjusted to: " << eps << "\n";
            sendFeedbackThresholdValue(eps, nodeId); // Send the adjusted threshold value back to the sensor node
            EV << "AdjustThresholdValue: New threshold: " << eps << "\n";
//            EV << "Feedback message sent.\n";
        } else {
            EV << "Threshold adjustment skipped. Difference (" << diff << ") exceeds tolerance (" << tol_eps << ").\n";
            EV << "AdjustThresholdValue: No change in threshold value. Error (" << error << ") is within tolerance (" << tol_KF << ").\n";
        }
    }
}

void Hub_Node3::sendFeedbackThresholdValue(double threshold, int nodeId) {
    EV << "Sending feedback threshold value: " << threshold << " to Node " << nodeId << "\n";
    FeedbackThresholdMsg *feedbackMsg = new FeedbackThresholdMsg("FeedbackThreshold");
    feedbackMsg->setThreshold(threshold);
//    feedbackMsg->setSourceNodeId(getId());
//    feedbackMsg->setDestinationNodeId(nodeId);

    if (nodeId == 10) {
        send(feedbackMsg, "out", 0); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << threshold << " to node: " << nodeId << "\n";
    } else if (nodeId == 11) {
        send(feedbackMsg, "out", 1); // Adjust the gate index as needed
        EV << "FeedbackThreshold message sent with new threshold: " << threshold << " to node: " << nodeId << "\n";
    } else {
        EV << "Unknown nodeId for feedback message: " << nodeId << "\n";
        delete feedbackMsg;
    }
}

void Hub_Node3::handleDataAggregation(double observationSum_node31, double observationSum_node32) {
    EV << "Aggregating data for OBN from Hub_Node3!" << "\n";

    double aggregatedValue_node31 = observationSum_node31;
    double aggregatedValue_node32 = observationSum_node32;

    EV << "Aggregated value for Node 31: " << aggregatedValue_node31 << "\n";
    EV << "Aggregated value for Node 32: " << aggregatedValue_node32 << "\n";

    // Create a new DataAggregationMessage object
    DataAggregationMessage *dataAggregationMsg = new DataAggregationMessage("DataAggregation");
    dataAggregationMsg->setSourceNodeId(getId());
    dataAggregationMsg->setAggregatedValueNode31(aggregatedValue_node31);
    dataAggregationMsg->setAggregatedValueNode32(aggregatedValue_node32);

    send(dataAggregationMsg, "out", 2); // Send the aggregated data message to the OBN node
    EV << "DataAggregation message sent with aggregated values to the OBN node from Hub_Node1.\n";
    bubble("AggregationMessage Sent from Hub_Node1!");
}

void Hub_Node3::handleConfigEnergyMessage(cMessage *msg) {
    EV << "Handling config and energy message in Hub_Node3.\n";
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param 1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");

    // Initialize the sensors
   //  scheduleAt(simTime() + Δ_M, new cMessage("TimeInterruptReceiveMandatoryData"));

    // Handle configuration and energy information as needed
    // For example, store values for later use, update internal state, etc.
}


