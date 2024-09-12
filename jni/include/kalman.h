#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <iostream>
#include <vector>

#include <Eigen/Dense>

namespace IntentionDetection
{
    class Kalman
    {
    public:
        Kalman() { ; }
        Kalman(const float& dt, const Eigen::Vector2f& target_delta, const float& x, const float& y, const float& vx, const float& vy, const Eigen::Matrix2f& _R);
        Eigen::Vector2f predict();
        void gainUpdate(const float& beta);
        Eigen::Vector4f update(const std::vector<Eigen::Vector2f >& selected_detections, const Eigen::VectorXf& beta, const float& last_beta);
    public:
        inline const Eigen::Matrix2f getS() const
        {
            return S;
        }
        const Eigen::Vector2f getHistoryPrediction() const { return history_prediction; }
        const Eigen::Vector2f getLastPrediction() const
        {
            return last_prediction;
        }
        inline const Eigen::Vector2f getLastPredictionEigen() const
        {
            return last_prediction_eigen;
        }
        inline const float getEntropy() const
        {
            return entropy;
        }
        void setDt(const double& dt)
        {
            A(4) = A(14) = dt;
        }
        const Eigen::Vector4f getUpdate()
        {
            return x_filter;
        }
    private:
        Eigen::Matrix4f A; //Evolution state matrix
        Eigen::Matrix2f Q; //Covariance Matrix associated to the evolution process
        Eigen::MatrixXf G; //Evolution Noise Matrix
        Eigen::Matrix4f P; //Covariance Matrix
        Eigen::MatrixXf C;
        Eigen::Matrix2f R; //Proces measurement Covariance matrix
        Eigen::Matrix2f S;
        Eigen::MatrixXf K; //Gain
        Eigen::Matrix4f P_predict; //Covariance Matrix predicted error
        Eigen::Vector4f x_predict;
        Eigen::Vector4f x_filter;
        Eigen::Vector2f z_predict;
        Eigen::Vector2f last_prediction;
        Eigen::Vector2f last_prediction_eigen;
        Eigen::Vector2f last_speed;
        bool first;
        float entropy;
    private:
        static constexpr float k = 5.0620; // n/2 * log(4*PI) where n is the state dimention (x, y, x', y')
        //static constexpr float k = 2.1984;

        Eigen::Vector2f history_prediction;
    };
}

#endif