#include "kalman.h"

using namespace IntentionDetection;

Kalman::Kalman(const float& dt, const Eigen::Vector2f& target_delta, const float& x, const float& y, const float& vx, const float& vy, const Eigen::Matrix2f& _R)
{
    //TRANSITION MATRIX
    A << 1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt,
        0, 0, 0, 1;

    //process noise covariance matrix 
    Q = _R,
        //NOISE EVOLUTION
        G = Eigen::MatrixXf(4, 2);

    G << std::pow(dt, 2) / 2, 0,
        dt, 0,
        0, std::pow(dt, 2) / 2,
        0, dt;

    //STATE OBSERVATION MATRIX
    C = Eigen::MatrixXf(2, 4);
    C << 1, 0, 0, 0,
        0, 0, 1, 0;

    //INITIAL COVARIANCE MATRIX
    P << std::pow(target_delta(0), 2), 0, 0, 0,
        0, .1, 0, 0,
        0, 0, std::pow(target_delta(1), 2), 0,
        0, 0, 0, .1;
    //GAIN     
    K = Eigen::MatrixXf(4, 2);

    //measurement noise covariance matrix
    R = _R;

    last_prediction = Eigen::Vector2f(x, y);
    last_speed = Eigen::Vector2f(vx, vy);
    first = true;

    history_prediction = last_prediction;
}


Eigen::Vector2f Kalman::predict()
{
    history_prediction = last_prediction;

    if (first)
    {
        x_predict << last_prediction(0), last_speed(0), last_prediction(1), last_speed(1);
        first = false;
    }
    else
    {
        x_predict = A * x_filter;
    }

    //Covariance Matrix predicted error
    P_predict = A * P * A.transpose() + G * Q * G.transpose();


    //Predicted Measurement
    z_predict = C * x_predict;

    //Error Measurement Covariance Matrix
    S = C * P_predict * C.transpose() + R;

    //Compute Entropy
    entropy = k + .5 * log10(P.determinant());

    last_prediction_eigen = z_predict;
    last_prediction = Eigen::Vector2f(z_predict(0), z_predict(1));
    return last_prediction;
}

void Kalman::gainUpdate(const float& beta)
{
  K = P_predict * C.transpose() * S.inverse();
  P = P_predict - (1 - beta) * K * S * K.transpose();
}

Eigen::Vector4f Kalman::update(const std::vector< Eigen::Vector2f >& selected_detections, const Eigen::VectorXf& beta, const float& last_beta)
{
    Eigen::Vector4f a;
    a.setZero();

    x_filter.setZero();
    unsigned int i = 0;

    for (const auto& det : selected_detections)
    {
        a = x_predict + K * (det - z_predict);
        x_filter = x_filter + beta(i) * a;
        ++i;
    }

    x_filter = last_beta * x_predict + x_filter;

    a.setZero();
    Eigen::Matrix4f Pk = Eigen::Matrix4f::Zero();
    Eigen::Vector2f det;

    for (i = 0; i < selected_detections.size() + 1; ++i)
    {
        if (i == selected_detections.size())
        {
            a = x_predict;
        }
        else
        {
            a = x_predict + K * (selected_detections.at(i) - z_predict);
        }

        Pk = Pk + beta(i) * (a * a.transpose() - x_filter * x_filter.transpose());
    }
    P += Pk;

    return x_filter;
}
