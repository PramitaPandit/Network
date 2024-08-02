/*
 * my_simulation_network.cc
 *
 *  Created on: July 1, 2024
 *      Author: Pramita
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <omnetpp.h>
#include "SensorNodeMsg_m.h"
#include "FeedbackThresholdMsg_m.h"

using namespace omnetpp;


class SensorNode : public cSimpleModule {
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void checkThreshold();
    virtual void handleConfigEnergyMessage(cMessage *msg);
    virtual void adjustThreshold(cMessage *msg);
    virtual void forwardMessage(SensorNodeMsg *msg);
    void readDataFromFile(const char* fileName, int columnIndex);

private:
    int nodeId;
    double eps; // Threshold
    int aboveThresholdCount;

    std::vector<double> sensorReadings;
    int currentReadingIndex;
    double lastSample;

    double Δ_N;
    double Δ_D;
    double Δ_M;  // Timer period for mandatory threshold update

    double previousMean;
    int batchSize;
    std::vector<double> currentBatch;
//    double tol_eps; // Tolerance for threshold adjustment

    int batchCounterSinceLastComm; // Counter to track time since last communication

    bool configReceived; // Flag to track receipt of ConfigEnergy message

    int hubNodeId; // ID of the Hub node to which this sensor node communicates

    double minReading; // Minimum value of the sensor readings
    double maxReading; // Maximum value of the sensor readings

//    int checkThresholdCount; // Counter for the check threshold function

public:
    SensorNode() : currentReadingIndex(0), lastSample(0), previousMean(0), batchSize(60), batchCounterSinceLastComm(0), configReceived(false), minReading(0), maxReading(0) {}
};

Define_Module(SensorNode);

void SensorNode::initialize() {
    Δ_N = par("Δ_N").doubleValue();
    Δ_D = par("Δ_D").doubleValue();
    eps = par("eps").doubleValue(); // Initialize threshold as eps
    Δ_M = par("Δ_M").doubleValue(); // Initialize timer period for mandatory threshold update
    batchSize = par("batchSize").intValue(); // Initialize batch size

    aboveThresholdCount = 0;
    currentReadingIndex = 0;
    lastSample = 0;

    int columnIndex = par("columnIndex").intValue();

    // Read the CSV file
    const char *fileName = par("fileName").stringValue();
    readDataFromFile(fileName, columnIndex);

    nodeId = getId(); // Get the automatically assigned ID from OMNeT++
    // Determine hubNodeId based on nodeId
    if (nodeId == 10 || nodeId == 11) {
        hubNodeId = 5; // Hub node ID for Node 10 and 11
    } else if (nodeId == 8 || nodeId == 9) {
        hubNodeId = 4; // Hub node ID for Node 8 and 9
    } else if (nodeId == 6 || nodeId == 7) {
        hubNodeId = 3; // Hub node ID for Node 6 and 7
    } else {
        hubNodeId = -1; // Invalid hubNodeId if nodeId does not match expected values
    }

    // Calculate min and max values for normalization
    if (!sensorReadings.empty()) {
        minReading = *std::min_element(sensorReadings.begin(), sensorReadings.end());
        EV << "Node " << nodeId << ": minReading: " << minReading << endl;
        maxReading = *std::max_element(sensorReadings.begin(), sensorReadings.end());
        EV << "Node " << nodeId << ": maxReading: " << maxReading << endl;

    }

    EV << "Node " << nodeId << ": eps initialized to: " << eps << endl;
    EV << "Finished reading file. Total readings: " << sensorReadings.size() << "\n";
    EV << "Node " << nodeId << ": Batch size: " << batchSize << endl;
    EV << "Total number of mean values: " << static_cast<double>(sensorReadings.size()) / batchSize << "\n";


    EV << "Initialization complete for sensor node: " << nodeId << " communicating with Hub_Node" << hubNodeId << "\n";

    // Start the sensing process immediately
//    checkThreshold();

    // Start the periodic check for threshold
    scheduleAt(simTime() + Δ_N, new cMessage("CheckThreshold"));
}

void SensorNode::handleMessage(cMessage *msg) {
    EV << "Message received: " << msg->getName() << " at time " << simTime() << "\n";

    if (strcmp(msg->getName(), "FeedbackThreshold") == 0) {
        EV << "Message received: " << msg->getName() << " FeedbackThreshold at " << simTime() << "\n";
        adjustThreshold(msg);
    } else if (strcmp(msg->getName(), "ConfigEnergy") == 0) {
        handleConfigEnergyMessage(msg);
        if (!configReceived) {
            configReceived = true;
            EV << "Config and energy message received, initializing timers.\n";
        }
    } else if (strcmp(msg->getName(), "CheckThreshold") == 0) {
        // Call checkThreshold and schedule the next check
        checkThreshold();
        scheduleAt(simTime() + Δ_N, new cMessage("CheckThreshold"));
    } else {
        EV << "Unhandled message type: " << msg->getName() << " at time " << simTime() << "\n";
    }
}

void SensorNode::adjustThreshold(cMessage *msg) {
    EV << "Node " << nodeId << " current threshold: " << eps << "\n";

    FeedbackThresholdMsg *feedbackMsg = dynamic_cast<FeedbackThresholdMsg *>(msg);
    if (!feedbackMsg) {
        EV << "Received message is not of type FeedbackThresholdMsg.\n";
        delete msg;
        return;
    }

    double newThreshold = feedbackMsg->getThreshold();
    eps = newThreshold;
    EV << "Node " << nodeId << " updated threshold: " << eps << "\n";
    delete feedbackMsg; // Free the memory after processing the message
    EV << "Node " << nodeId << " adjusted threshold to: " << eps << "\n";
}



void SensorNode::checkThreshold() {
    EV << "Node " << nodeId << " starting checkThreshold function. " << endl;
    EV << "Node " << nodeId << ": currentReadingIndex: " << currentReadingIndex << ", sensorReadings.size(): " << sensorReadings.size() << endl;


    while (currentReadingIndex < sensorReadings.size() && currentBatch.size() < batchSize) {
            double reading = sensorReadings[currentReadingIndex];
            EV << "Node " << nodeId << ": reading: " << sensorReadings[currentReadingIndex] << endl;

            // Normalize the reading
            double normalizedReading = (reading - minReading) / (maxReading - minReading);
            EV << "Node " << nodeId << ": Original reading: " << reading << ", Normalized reading: " << normalizedReading << endl;

            currentBatch.push_back(normalizedReading);
            currentReadingIndex++;
            EV << "Node " << nodeId << ": currentReadingIndex: " << currentReadingIndex << endl;
            EV << "Node " << nodeId << ": Current batch size: " << currentBatch.size() << ", Desired batchsize:" << batchSize << endl;
        }

        if (currentBatch.size() == batchSize) {
            double currentMean = std::accumulate(currentBatch.begin(), currentBatch.end(), 0.0) / currentBatch.size();
            EV << "Node " << nodeId << ": Current mean (" << currentMean << endl;


            if (currentMean >= eps) {
                EV << "Node " << nodeId << ": Current mean (" << currentMean << ") is above threshold (" << eps << ")" << endl;
                // Send RegComm
                SensorNodeMsg *regCommMsg = new SensorNodeMsg("RegComm");
                regCommMsg->setMean(currentMean);
                regCommMsg->setSourceNodeId(nodeId);
                regCommMsg->setDestinationNodeId(hubNodeId);

                forwardMessage(regCommMsg);
                EV << "Node " << nodeId << " sending regular communication with mean: " << currentMean << "\n";

                batchCounterSinceLastComm = 0; // Reset the counter after regular communication
            } else {
                EV << "Node " << nodeId << ": Current mean (" << currentMean << ") is below threshold (" << eps << ")" << endl;
                bubble("RegComm is skipped!");

                batchCounterSinceLastComm++;
            }

            previousMean = currentMean;

        }

        if (batchCounterSinceLastComm >= Δ_M) {
            double currentMean = std::accumulate(currentBatch.begin(), currentBatch.end(), 0.0) / currentBatch.size();
            SensorNodeMsg *mandCommMsg = new SensorNodeMsg("MandComm");
            mandCommMsg->setMean(currentMean);
            mandCommMsg->setSourceNodeId(nodeId);
            forwardMessage(mandCommMsg);
            EV << "Node " << nodeId << " sending mandatory communication with mean: " << currentMean << "\n";
            bubble("MandComm is sent!");

            batchCounterSinceLastComm = 0; // Reset the counter after mandatory communication
        }

        // Clear the batch only if it's full and not used for MandComm
        if (currentBatch.size() == batchSize && batchCounterSinceLastComm < Δ_M) {
            currentBatch.clear();
            EV << "Node " << nodeId << ": Current batch cleared." << endl;
        }

        if (currentReadingIndex >= sensorReadings.size()) {
                EV << "Node " << nodeId << ": currentReadingIndex: " << currentReadingIndex << ", sensorReadings.size(): " << sensorReadings.size() << endl;
                // Send EndSimulation message to Hub node
                SensorNodeMsg *endSimMsg = new SensorNodeMsg("EndSimulation");
                endSimMsg->setSourceNodeId(nodeId);
                endSimMsg->setDestinationNodeId(hubNodeId);

                forwardMessage(endSimMsg);
                EV << "Node " << nodeId << " sent EndSimulation message." << endl;

                // Stop scheduling further checks
                return;  // Exit the function, stopping further scheduling
            }
        // Schedule the next check
        cMessage *nextCheck = new cMessage("CheckThreshold");
        scheduleAt(simTime() + Δ_N, nextCheck);

    EV << "CheckThreshold function executing at time " << simTime() << "\n";

}


void SensorNode::forwardMessage(SensorNodeMsg *msg) {
    // Determine the Hub module name based on hubNodeId
    std::string hubModuleName;
    if (hubNodeId == 3) {
        hubModuleName = "Hub_1";
    } else if (hubNodeId == 4) {
        hubModuleName = "Hub_2";
    } else if (hubNodeId == 5) {
        hubModuleName = "Hub_3";
    } else {
        EV << "Error: Invalid hubNodeId " << hubNodeId << ". No corresponding Hub module.\n";
        delete msg; // Clean up the message if the hubNodeId is invalid
        return;
    }

    cModule *hubModule = getParentModule()->findModuleByPath(hubModuleName.c_str());

    if (hubModule) {
        send(msg, "out"); // No need to specify index for non-vector gate
        EV << "Message being forwarded to " << hubModuleName << "\n";
        bubble("Message Sent!");
    } else {
        EV << "Error: Hub module " << hubModuleName << " not found.\n";
        delete msg; // Clean up the message if the destination is not found
    }
}


void SensorNode::handleConfigEnergyMessage(cMessage *msg) {
    double configParam1 = msg->par("configParam1").doubleValue();
    double energyLevel = msg->par("energyLevel").doubleValue();
    EV << "Config Param1: " << configParam1 << ", Energy Level: " << energyLevel << "\n";
    bubble("ConfigEnergyMessage Received from OBN!");
    // Additional logic for handling config and energy messages can be implemented here
}

void SensorNode::readDataFromFile(const char* fileName, int columnIndex) {
    FILE *file = fopen(fileName, "r");
    if (!file) {
        EV << "Error opening file " << fileName << "\n";
        return;
    }

    // Skip the header row
    char buffer[1024];
    fgets(buffer, sizeof(buffer), file);

    // Read each line of the CSV file and store the values in sensorReadings
    double value;
    while (fgets(buffer, sizeof(buffer), file)) {
        char *token = strtok(buffer, ",");
        for (int i = 0; i < columnIndex; ++i) {
            token = strtok(nullptr, ",");
        }
        if (token) {
            sscanf(token, "%lf", &value);
            sensorReadings.push_back(value);
        }
    }

    fclose(file);
}

// Define Node11
class node11 : public SensorNode {
protected:
    virtual void initialize() override {
        SensorNode::initialize();
    }
};

Define_Module(node11);

// Define Node12
class node12 : public SensorNode {
protected:
    virtual void initialize() override {
        SensorNode::initialize();
    }
};

Define_Module(node12);


// Define Node21
class node21 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();
    }
};

Define_Module(node21);

// Define Node22
class node22 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();

    }
};

Define_Module(node22);

// Define Node31
class node31 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();
    }
};

Define_Module(node31);

// Define Node32
class node32 : public SensorNode {
protected:
    virtual void initialize() override {
            SensorNode::initialize();
    }
};

Define_Module(node32);

