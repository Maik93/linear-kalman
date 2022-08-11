#include "kalman.hpp"

#include <iostream>
#include <fstream>

#include <vector>
#include <random>
#include <Eigen/Dense>

int main() {
    // Problem statement: uniformly accelerated motion, measuring position only

    double dt = 1. / 30; // Time step

    // --------------- generate noisy measurements ---------------
    std::vector<double> measurements(50);
    std::default_random_engine generator;
    std::normal_distribution<double> measure_noise(0., .2);
    auto t = 0.;
    for (int i = 0; i < 50; ++i) {
        measurements[i] = 10. - 9.81 * pow(t, 2) + measure_noise(generator);
        t += dt;
    }

    // -------------------- setup the filter ---------------------
    int n = 3; // Number of states
    int m = 1; // Number of measurements

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Discrete LTI for states [position, velocity, acceleration]
    A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    C << 1, 0, 0;

    // Reasonable covariance matrices
    Q << .01, 0., 0., 0., .01, 0., 0., 0., .01;
    R << .2;
    P << .1, .1, 0., .1, .1, 0., 0., 0., .1;

    std::cout << "A: " << std::endl << A << std::endl;
    std::cout << "C: " << std::endl << C << std::endl;
    std::cout << "Q: " << std::endl << Q << std::endl;
    std::cout << "R: " << std::endl << R << std::endl;
    std::cout << "P: " << std::endl << P << std::endl;

    // Construct the filter
    KalmanFilter kf(A, C, Q, R, P);

    // Best guess of initial states
    Eigen::VectorXd x0(n);
    x0 << measurements[0], 0, -9.81;
    kf.init(x0);

    // --------------------- run the filter ----------------------

    std::ofstream log;
    log.open("log.csv");
    log << "time,measure,estimate" << std::endl;

    // Feed measurements into filter, output estimated states
    Eigen::VectorXd y(m);
    std::cout << "t = " << kf.time() << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
    for (int i = 0; i < measurements.size(); i++) {
        y << measurements[i];
        kf.update(y, dt);
        std::cout << "t = " << kf.time() << ", " << "y[" << i << "] = " << y << std::endl;

        std::cout << "x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
        std::cout << "P: " << std::endl << kf.covariance() << std::endl << std::endl;

        // store measures and estimates in the csv file
        log << kf.time() << ',' << y << ',' << kf.state()[0] << std::endl;
    }

    log.close();
    return 0;
}
