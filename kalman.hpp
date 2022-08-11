#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    /**
     * Create a Kalman filter with the specified matrices.
     * @param dt time-step increment.
     * @param A System dynamics matrix.
     * @param C Output matrix.
     * @param Q Process noise covariance.
     * @param R Measurement noise covariance.
     * @param P Estimate error covariance.
     */
    KalmanFilter(Eigen::MatrixXd A, Eigen::MatrixXd C,
                 Eigen::MatrixXd Q, Eigen::MatrixXd R,
                 Eigen::MatrixXd P);

    /// Initialize the filter with initial states as zero.
    void init();

    /// Initialize the filter with a guess for initial states.
    void init(const Eigen::VectorXd &x0);

    /**
     * Update the estimated state based on measured values,
     * using the given time step.
     */
    void update(const Eigen::VectorXd &y, double dt);

    /// Get current estimated state (a-posteriori).
    [[nodiscard]] Eigen::VectorXd state() { return x_hat; };

    /// Get current estimated covariance (a-posteriori).
    [[nodiscard]] Eigen::MatrixXd covariance() { return P; };

    /// Get current time.
    [[nodiscard]] double time() const { return t; };

private:
    Eigen::MatrixXd A, C, Q, R, P, K, I;

    Eigen::VectorXd x_hat; /// A-posteriori estimated state

    int n; /// State-space dimension

    double t{0}; /// Current time

    bool initialized{false}; /// Is the filter initialized?
};
