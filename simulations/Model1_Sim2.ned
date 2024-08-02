package Model1_Sim3.simulations;
package inet.physicallayer.wireless.common.pathloss;
package inet.physicallayer.wireless.common.pathloss;

import inet.physicallayer.wireless.common.contract.packetlevel.IPathLoss;
//import ned.RayleighChannel;
//import inet.channel.RayleighChannel;


network Model1_Sim3_network
{
    @display("bgb=735,470");
    types:
        simple OBN_node
        {
            parameters:
                double Δ_E; 
                double Δ_M;
                double timerPeriod = default(1.0);
//                double timeSlot @unit(s); // Time slot duration
                double initialX; // Add this line
                //int initialX = default(5)
                bool sendMsgOnInit = default(false);
//                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
            gates:
                input in[3];
                output out[9];
        }
        simple Hub_Node1
        {
            parameters:
                double Δ_D;
                double Δ_E; 
                double Δ_M;
                double eps = default(0.5); // Initialize eps with a default value 
                double tol_KF; // Tolerance for Kalman filter
                double tol_eps; // Tolerance for threshold adjustment
                double min_eps;
                double max_eps;
                double K;
                double timerPeriod = default(1.0);
 //               double timeSlot @unit(s); // Time slot duration
                double initialX; // Add this line
                //int initialX = default(5)
                bool sendMsgOnInit = default(false);
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                min_eps = 0.1;
                max_eps = 0.9;
                tol_eps = 0.001;
                K = 0.1;
                tol_KF = 0.005;
                
               
            gates:
                input in[3];
                output out[3];
        }
        simple Hub_Node2
        {
            parameters:
                double Δ_D;
                double Δ_E; 
                double Δ_M;
                double eps = default(0.5); // Initialize eps with a default value
                double tol_KF; // Tolerance for Kalman filter
                double tol_eps; // Tolerance for threshold adjustment
                double min_eps;
                double max_eps;
                double K;
                double timerPeriod = default(1.0);
 //               double timeSlot @unit(s); // Time slot duration
                double initialX; // Add this line
                //int initialX = default(5)
                bool sendMsgOnInit = default(false);
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                min_eps = 0.1;
                max_eps = 0.9;
                tol_eps = 0.001;
                K = 0.1;
                tol_KF = 0.005;

            gates:
                input in[3];
                output out[3];
        }
        simple Hub_Node3
        {
            parameters:
                double Δ_D;
                double Δ_E; 
                double Δ_M;
                double eps = default(0.5); // Initialize eps with a default value
                double tol_KF; // Tolerance for Kalman filter
                double tol_eps; // Tolerance for threshold adjustment
                double min_eps;
                double max_eps;
                double K;
                double timerPeriod = default(1.0);
  //              double timeSlot @unit(s); // Time slot duration
                double initialX; // Add this line
                //int initialX = default(5)
                bool sendMsgOnInit = default(false);
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                min_eps = 0.1;
                max_eps = 0.9;
                tol_eps = 0.001;
                K = 0.1;
                tol_KF = 0.005;
            gates:
                input in[3];
                output out[3];
        }
        simple SensorNode
        {
            parameters:
                double Δ_N;
                double Δ_D;
                double Δ_E;
                double Δ_M;
                double timerPeriod = default(1.0);
                double eps = default(0.5); // Initialize eps with a default value
                double tol_eps; // Threshold Tolerance
                int batchSize;
                double previousMean;
                string fileName;
                int columnIndex;
//                double timeSlot @unit(s); // Time slot duration
                double initialX;
                bool sendMsgOnInit = default(false);
                //double startTime @unit(s);  // Parameter to set the start time of the first event
            gates:
                input in[2];
                output out;
        }

        simple node11 extends SensorNode
        {
            parameters:
                Δ_N = 1;
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                batchSize = 60;
                double startTime @unit(s);  // Parameter to set the start time of the first event
                fileName = "HR_001.csv";
                //               columnIndex = 0;  // First column
                columnIndex = default(1);  // Index for the first column
                sendMsgOnInit = true;   // Sensor nodes should send a message on init
        }

        simple node12 extends SensorNode
        {
            parameters:
                Δ_N = 1;
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                batchSize = 60;
                
                fileName = "HR_001.csv";
                //                columnIndex = 0;  // First column
                columnIndex = default(1);  // Index for the second column
                sendMsgOnInit = true;   // Sensor nodes should send a message on init
        }

        simple node21 extends SensorNode
        {
            parameters:
                Δ_N = 1;
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                batchSize = 60;
                fileName = "HR_001.csv";
                columnIndex = default(1);  // Index for the third column
                sendMsgOnInit = true;   // Sensor nodes should send a message on init
        }

        simple node22 extends SensorNode
        {
            parameters:
                Δ_N = 1;
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                batchSize = 60;
                fileName = "HR_001.csv";
                columnIndex = default(1);  // Index for the fourth column
                sendMsgOnInit = true;   // Sensor nodes should send a message on init
        }

        simple node31 extends SensorNode
        {
            parameters:
                Δ_N = 1;
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                batchSize = 60;
                fileName = "HR_001.csv";
                columnIndex = default(1);  // Index for the fifth column
                sendMsgOnInit = true;   // Sensor nodes should send a message on init
        }

        simple node32 extends SensorNode
        {
            parameters:
                Δ_N = 1;
                Δ_D = 10;
                Δ_E = 15;
                Δ_M = 5;
                batchSize = 60;
                fileName = "HR_001.csv";
                columnIndex = default(1);  // Index for the sixth column
                sendMsgOnInit = true;   // Sensor nodes should send a message on init
        }



        // Define the Rayleigh channel module
        channel RayleighChannel extends ned.DatarateChannel
        {
            parameters:
   //             double alpha @unit(dB); // Path loss exponent for Rayleigh fading
   //             double systemLoss @unit(dB) = default(0dB); // System loss
   //             double distance @unit(cm); // Custom parameter for distance
        }



    submodules:
        OBN: OBN_node {
            //parameters:
            //initialX = 5;
            initialX = 1000; // Example setting for OBN
            @display("p=27,226");
        }
        Hub_1: Hub_Node1 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=166,98");
        }
        Hub_2: Hub_Node2 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=166,226");
        }
        Hub_3: Hub_Node3 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=166,331");
        }
        Node_11: node11 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=317,47");
        }
        Node_12: node12 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=317,119");
        }
        Node_21: node21 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=314,197");
        }
        Node_22: node22 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=314,265");
        }
        Node_31: node31 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=314,331");
        }
        Node_32: node32 {
            initialX = 1000; // Example setting for Hub_1
            @display("p=314,395");
        }

    connections:
         // Connect nodes through the Rayleigh channel

         //Sending messages from OBN to Hubs
         
        OBN.out[3] --> RayleighChannel --> Node_32.in[1];
        OBN.out[4] --> RayleighChannel --> Node_31.in[1];
        OBN.out[5] --> RayleighChannel --> Node_22.in[1];
        OBN.out[6] --> RayleighChannel --> Node_21.in[1];
        OBN.out[7] --> RayleighChannel --> Node_12.in[1];
        OBN.out[8] --> RayleighChannel --> Node_11.in[1];

        
        // 1) OBN -> Hub_1 AND Hub_2 AND Hub_3
        OBN.out[0] --> RayleighChannel --> Hub_1.in[2];
        OBN.out[1] --> RayleighChannel --> Hub_2.in[2];
        OBN.out[2] --> RayleighChannel --> Hub_3.in[2];
        
                // Receiving messages from Nodes to Hubs
        // 3) Node11 AND Node12 -> Hub_1
        Node_11.out --> RayleighChannel --> Hub_1.in[0];
        Node_12.out --> RayleighChannel --> Hub_1.in[1];

        // 3) Node21 AND Node22 -> Hub_2
        Node_21.out --> RayleighChannel --> Hub_2.in[0];
        Node_22.out --> RayleighChannel --> Hub_2.in[1];

        // 3) Node31 AND Node32 -> Hub_3
        Node_31.out --> RayleighChannel --> Hub_3.in[0];
        Node_32.out --> RayleighChannel --> Hub_3.in[1];
        

        //Forwarding from Hubs to Nodes
        // 2) Hub_1 -> Node11 AND Node12
        Hub_1.out[0] --> RayleighChannel  --> Node_11.in[0];
        Hub_1.out[1] --> RayleighChannel  --> Node_12.in[0];

        // 2) Hub_2 -> Node21 AND Node22
        Hub_2.out[0] --> RayleighChannel  --> Node_21.in[0];
        Hub_2.out[1] --> RayleighChannel  --> Node_22.in[0];

        // 2) Hub_3 -> Node31 AND Node32
        Hub_3.out[0] --> RayleighChannel  --> Node_31.in[0];
        Hub_3.out[1] --> RayleighChannel  --> Node_32.in[0];



        //Forwarding from Hubs to OBN
        // 4) Hub_1 AND Hub_2 AND Hub_3 -> OBN
        Hub_1.out[2] --> RayleighChannel --> OBN.in[0];
        Hub_2.out[2] --> RayleighChannel --> OBN.in[1];
        Hub_3.out[2] --> RayleighChannel --> OBN.in[2];

}
