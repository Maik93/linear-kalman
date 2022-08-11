#include "kalman.hpp"

KalmanFilter::KalmanFilter(Eigen::MatrixXd A, Eigen::MatrixXd C,
                           Eigen::MatrixXd Q, Eigen::MatrixXd R,
                           Eigen::MatrixXd P0)
        : A(std::move(A)), C(std::move(C)), Q(std::move(Q)), R(std::move(R)), P(std::move(P0)),
          n(static_cast<int>(this->A.rows())), x_hat(n) {
    I = Eigen::MatrixXd(n, n).setIdentity();
}

void KalmanFilter::init() {
    x_hat.setZero();
    initialized = true;
}

void KalmanFilter::init(const Eigen::VectorXd &x0) {
    init();
    x_hat = x0;
}

void KalmanFilter::update(const Eigen::VectorXd &y, double dt) {
    if (!initialized)
        throw std::runtime_error("Filter is not initialized!");

    // prediction phase
    x_hat = A * x_hat; // a-priori estimated state
    P = A * P * A.transpose() + Q;

    // correction phase
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (y - C * x_hat); // a-posteriori estimated state
    P = (I - K * C) * P;

    t += dt;
}
