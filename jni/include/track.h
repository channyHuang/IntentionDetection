#ifndef _TRACK_H_
#define _TRACK_H_

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "kalman.h"

namespace IntentionDetection
{
    class TrackerParam {
    public:
        float fProbDetection = 1;
        float fProbGate = 0.4;
        float fDt = 0.4;
        float fSigmaLocal = 15;
        float fSigmaGlobal = 0.1;
        float fAssociationCostLocal = 40;
        float fAssociationCostGlobal = 50;
        float fLambda = 2;
        float fGamma;
        int nMinAcceptRate = 10;
        int nMaxMissRate = 9;
        Eigen::Matrix2f matCovarance;
        Eigen::Vector2f vDelta = Eigen::Vector2f(10, 10);

        TrackerParam() {
            matCovarance << 100 , 0 , 0 , 100;
            fGamma = 0.000001 * fLambda;
        }
    };

    class Track
    {
    public:
        enum TrackState { 
            NONE, 
            ACCEPT, 
            DISCARD 
        };

        Track() { ; }
        Track(const float& dt, const Eigen::Vector2f& target_delta, const float& x, const float& y,
            const float& vx, const float& vy, const float& g_sigma, const float& gamma, const Eigen::Matrix2f& _R);
        
        Eigen::Vector2f predict();
        
        void setId(const int& _id) {
            id = _id;
        }
        int getId() {
            return id;
        }
        const Eigen::Vector2f getLastPrediction() const {
            return KF->getLastPrediction();
        }
        Eigen::Vector2f getVelocity() const {
            return KF->getLastPrediction() - KF->getHistoryPrediction();
        }
        const Eigen::Vector2f getHistoryPrediction() const {
            return KF->getHistoryPrediction();
        }
        const Eigen::Vector2f getLastPredictionEigen() {
            return KF->getLastPredictionEigen();
        }
        const Eigen::Matrix2f S() const {
            return KF->getS();
        }
        const float getEllipseVolume() const {
            return ellipse_volume;
        }
        const void gainUpdate(const float& beta) {
            KF->gainUpdate(beta);
        }
        const Eigen::Vector4f update(const std::vector< Eigen::Vector2f >& selected_detections, const Eigen::VectorXf& beta, const float& last_beta) {
            life_time++;
            nodetections = 0;
            return KF->update(selected_detections, beta, last_beta);
        }
        void increaseLifetime() {
            life_time++;
        }
        void notDetected() {
            nodetections++;
        }
        bool isAlive()  {
            return nodetections < maxNotDetection;
        }
        
        const TrackState getEntropy() const {
            return entropy_sentinel;
        }
        void setDt(const double& _dt) {
            KF->setDt(_dt);
        }
        const Eigen::Vector4f getUpdate() {
            return KF->getUpdate();
        }

    private:
        int id;
        int maxNotDetection;
        std::shared_ptr<Kalman> KF;
        float ellipse_volume;
        int number_returns;
        float side;
        float g_sigma;
        float gamma;
        int life_time;
        int nodetections;
        float initial_entropy;
        TrackState entropy_sentinel;
        Eigen::Vector2f last_prediction;
    };
}

#endif