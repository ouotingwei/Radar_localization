#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <random>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(double x = 0, double y = 0, double yaw = 0) {
        // Define what state to be estimate
        // Ex.
        //   only pose -> Eigen::Vector3d(x, y, yaw)
        //   with velocity -> Eigen::Vector6d(x, y, yaw, vx, vy, vyaw)
        //   etc...
        pose << x, y, yaw;   // only pose

        // Transition matrix
        A = Eigen::Matrix3d::Identity(); // jacobian matrix
        B = Eigen::Matrix3d::Identity(); // motion transition matrix

        // State covariance matrix
        S = Eigen::Matrix3d::Identity() * 1;

        // Observation matrix
        C << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

        // State transition error
        R = Eigen::Matrix3d::Identity() * 1;

        // Measurement error
        Q = Eigen::Matrix3d::Identity() * 1;
        
        std::cout << "Initialize Kalman Filter" << std::endl;
    }

    void predict(const Eigen::Vector3d& u) {
        // Base on the Kalman Filter design in Assignment 3
        // Implement a linear or nonlinear motion model for the control input
        // Calculate Jacobian matrix of the model as A
        // u = [del_x, del_y, del_yaw]

        //setting the random noise R
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> distribution(0.0, std::sqrt(2.25));

        for (int i = 0; i < R.rows(); ++i) {
            for (int j = 0; j < R.cols(); ++j) {
                R(i, j) = distribution(gen);
            }
        }

        B << std::cos(pose[2]), -std::sin(pose[2]), 0,
             std::sin(pose[2]), std::cos(pose[2]), 0,
             0, 0, 1;  // setting the motion transition matrix

        A << 1, 0, -std::sin(pose[2]) * u[0] - std::cos(pose[2]) * u[1],
             0, 1, std::cos(pose[2]) * u[0] - std::sin(pose[2]) * u[1],
             0, 0, 1;  // setting the jacobian matrix

        pose += B * u; // motion model
        S = A * S * A.transpose() + R;    // state
    }

    Eigen::Vector3d update(const Eigen::Vector3d& z) {
        // Base on the Kalman Filter design in Assignment 3
        // Implement a linear or nonlinear observation matrix for the measurement input
        // Calculate Jacobian matrix of the matrix as C
        // z = [x, y, yaw]

        //setting the random noise S
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> distribution(0.0, std::sqrt(2.25));

        for (int i = 0; i < S.rows(); ++i) {
            for (int j = 0; j < S.cols(); ++j) {
                S(i, j) = distribution(gen);
            }
        }

        // I choose the linear model to update the pose & state

        Eigen::Matrix3d K = S * C.transpose() * (C * S * C.transpose() + Q).inverse();
        pose = pose + K * (z - C * pose);
        S = (Eigen::Matrix3d::Identity() - K * C) * S;

        return pose;
    }

private:
    Eigen::Vector3d pose;
    Eigen::Matrix3d A, B, S, R, Q;
    Eigen::Matrix3d C;
};