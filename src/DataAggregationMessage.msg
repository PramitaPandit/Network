//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
// DataAggregationMessage.msg

message DataAggregationMessage {
    int sourceNodeId;   			// ID of the source node
    double aggregatedValueNode11;	// Aggregated value from node 11
    double aggregatedValueNode12;	// Aggregated value from node 12
    double aggregatedValueNode21;	// Aggregated value from node 21
    double aggregatedValueNode22;	// Aggregated value from node 22
    double aggregatedValueNode31;	// Aggregated value from node 31
    double aggregatedValueNode32;	// Aggregated value from node 32
}
