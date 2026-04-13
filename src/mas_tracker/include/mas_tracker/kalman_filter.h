#pragma once

#include <Eigen/Dense>
// #include <iostream> // Was commented out, remove if not used

namespace mas_tracker {

// Linear Kalman filtering class
class KF {
public:
    // Constructor
    KF(unsigned int num_states, unsigned int num_measurements)
    : num_states_(num_states),
      num_measurements_(num_measurements),
      sampling_time_(1.0) // Default sampling time
    {
        // Initialize matrices and vectors with zeros
        F_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
        H_ = Eigen::MatrixXd::Zero(num_measurements_, num_states_);
        K_ = Eigen::MatrixXd::Zero(num_states_, num_measurements_);

        init_state_cov_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
        state_cov_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
        predicted_state_cov_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
        process_noise_cov_ = Eigen::MatrixXd::Zero(num_states_, num_states_);
        measurement_cov_ = Eigen::MatrixXd::Zero(num_measurements_, num_measurements_);

        state_ = Eigen::VectorXd::Zero(num_states_);
        predicted_state_ = Eigen::VectorXd::Zero(num_states_);
        measurement_ = Eigen::VectorXd::Zero(num_measurements_);
    }

    virtual ~KF() = default;

    /**
     * @brief Predicts the next state and state covariance.
     * @return The predicted state vector.
     */
    Eigen::VectorXd predict() {
        // Predict state: x_hat_k|k-1 = F * x_hat_k-1|k-1
        predicted_state_ = F_ * state_;
        // Predict state covariance: P_k|k-1 = F * P_k-1|k-1 * F^T + Q
        predicted_state_cov_ = F_ * state_cov_ * F_.transpose() + process_noise_cov_;

        // For the next cycle, the current state and covariance become the predicted ones
        state_ = predicted_state_;
        state_cov_ = predicted_state_cov_;
        return state_;
    }

    /**
     * @brief Updates the state and state covariance with a new measurement.
     * @param z The measurement vector.
     * @return The updated (corrected) state vector.
     */
    Eigen::VectorXd update(const Eigen::VectorXd& z) {
        measurement_ = z; // Store the measurement

        // Calculate Kalman Gain: K = P_k|k-1 * H^T * (H * P_k|k-1 * H^T + R)^-1
        // Note: state_cov_ here is P_k|k-1 (predicted_state_cov_) after predict() step
        Eigen::MatrixXd S = H_ * state_cov_ * H_.transpose() + measurement_cov_;
        K_ = state_cov_ * H_.transpose() * S.inverse(); // Using .inverse(), ensure S is invertible

        // Update state estimate: x_hat_k|k = x_hat_k|k-1 + K * (z - H * x_hat_k|k-1)
        // Note: state_ here is x_hat_k|k-1 (predicted_state_)
        state_ = state_ + K_ * (measurement_ - H_ * state_);

        // Update state covariance: P_k|k = (I - K * H) * P_k|k-1
        state_cov_ = (Eigen::MatrixXd::Identity(num_states_, num_states_) - K_ * H_) * state_cov_;

        return state_;
    }

    // State vector (x_hat)
    Eigen::VectorXd state_;
    // Predicted state vector (x_hat_k|k-1)
    Eigen::VectorXd predicted_state_;
     // Measurement vector (z)
    Eigen::VectorXd measurement_;

    // Process noise covariance matrix (Q)
    Eigen::MatrixXd process_noise_cov_;
    // State covariance matrix (P_k|k or P_k-1|k-1)
    Eigen::MatrixXd state_cov_;
    // Predicted state covariance matrix (P_k|k-1)
    Eigen::MatrixXd predicted_state_cov_;
    // Initial state covariance matrix
    Eigen::MatrixXd init_state_cov_;
    // Measurement noise covariance matrix (R)
    Eigen::MatrixXd measurement_cov_;

    unsigned int num_states_;
    unsigned int num_measurements_;
    double sampling_time_; // Sampling time (dt), if used explicitly in F

    // System matrices
    Eigen::MatrixXd F_; // State transition matrix
    Eigen::MatrixXd H_; // Measurement matrix
    Eigen::MatrixXd K_; // Kalman gain
};

} // namespace mas_tracker
