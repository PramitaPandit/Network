/*
 * KalmanFilter.h
 *
 *  Created on: Mar 8, 2024
 *      Author: pramita
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>

class KalmanFilter {
public:
    KalmanFilter(const std::vector<std::vector<double>>& F,
                 const std::vector<std::vector<double>>& H,
                 const std::vector<std::vector<double>>& Q,
                 const std::vector<std::vector<double>>& R,
                 const std::vector<std::vector<double>>& P,
                 const std::vector<double>& x0);

    void predict();
    double update(double z);
    double getState() const;
    double updateEstimate(double z);  // Add this line

private:
    std::vector<std::vector<double>> F, H, Q, R, P;
    std::vector<double> x;
};

#endif // KALMANFILTER_H
