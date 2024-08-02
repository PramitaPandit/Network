/*
 * KalmanFilter.cc
 *
 *  Created on: Mar 8, 2024
 *      Author: pramita
 */

#include "KalmanFilter.h"
#include <cassert>
#include <cmath>
#include <iostream>

KalmanFilter::KalmanFilter(const std::vector<std::vector<double>>& F,
                           const std::vector<std::vector<double>>& H,
                           const std::vector<std::vector<double>>& Q,
                           const std::vector<std::vector<double>>& R,
                           const std::vector<std::vector<double>>& P,
                           const std::vector<double>& x0)
    : F(F), H(H), Q(Q), R(R), P(P), x(x0) {}

void KalmanFilter::predict() {
    // Perform matrix multiplication F * x
    std::vector<double> x_pred(x.size(), 0);
    for (size_t i = 0; i < F.size(); ++i) {
        for (size_t j = 0; j < x.size(); ++j) {
            x_pred[i] += F[i][j] * x[j];
        }
    }
    x = x_pred;

    // Update the estimate error covariance P
    std::vector<std::vector<double>> P_temp(F.size(), std::vector<double>(F.size(), 0));
    for (size_t i = 0; i < F.size(); ++i) {
        for (size_t j = 0; j < F.size(); ++j) {
            for (size_t k = 0; k < F.size(); ++k) {
                P_temp[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    for (size_t i = 0; i < F.size(); ++i) {
        for (size_t j = 0; j < F.size(); ++j) {
            P[i][j] = P_temp[i][j] + Q[i][j];
        }
    }
}

double KalmanFilter::update(double z) {
    // Calculate y = z - H * x
    if (H[0].size() != x.size()) {
        std::cerr << "Error: Dimension mismatch between H and x." << std::endl;
        return x[0];
    }

    double y = z;
    for (size_t i = 0; i < H[0].size(); ++i) {
        y -= H[0][i] * x[i];
    }

    // Calculate S = H * P * H^T + R
    double S = 0;
    for (size_t i = 0; i < H[0].size(); ++i) {
        for (size_t j = 0; j < H[0].size(); ++j) {
            S += H[0][i] * P[i][j] * H[0][j];
        }
    }
    S += R[0][0];

    // Calculate K = P * H^T * inv(S)
    std::vector<double> K(P.size(), 0);
    for (size_t i = 0; i < P.size(); ++i) {
        K[i] = P[i][0] * H[0][i] / S;
    }

    // Update the state estimate x
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] += K[i] * y;
    }

    // Update the estimate error covariance P
    std::vector<std::vector<double>> I(P.size(), std::vector<double>(P.size(), 0));
    for (size_t i = 0; i < P.size(); ++i) {
        I[i][i] = 1;
    }

    std::vector<std::vector<double>> K_H(P.size(), std::vector<double>(H[0].size(), 0));
    for (size_t i = 0; i < K.size(); ++i) {
        for (size_t j = 0; j < H[0].size(); ++j) {
            K_H[i][j] = K[i] * H[0][j];
        }
    }

    std::vector<std::vector<double>> I_KH(I.size(), std::vector<double>(I[0].size(), 0));
    for (size_t i = 0; i < I.size(); ++i) {
        for (size_t j = 0; j < I[i].size(); ++j) {
            I_KH[i][j] = I[i][j] - K_H[i][j];
        }
    }

    std::vector<std::vector<double>> P_new(P.size(), std::vector<double>(P[0].size(), 0));
    for (size_t i = 0; i < P.size(); ++i) {
        for (size_t j = 0; j < P[i].size(); ++j) {
            for (size_t k = 0; k < P[i].size(); ++k) {
                P_new[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }
    P = P_new;

    // Return the filtered state estimate (assuming x[0] is the filtered value)
    return x[0];
}

double KalmanFilter::getState() const {
    return x[0]; // Assuming single-dimensional state
}

// Add the definition of updateEstimate
double KalmanFilter::updateEstimate(double z) {
    predict();
    return update(z);
}



