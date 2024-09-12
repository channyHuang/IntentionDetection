#ifndef _GLOBAL_TRACKER_H_
#define _GLOBAL_TRACKER_H_

#include <iostream>
#include <numeric>
#include <unordered_map>

#include <Eigen/Dense>

#include "local_tracker.h"
#include "ObjectAI.h"

namespace IntentionDetection
{
    class GlobalTracker : public Tracker
    {
    public:
        GlobalTracker(const TrackerParam& _param);
        virtual void track(const std::unordered_map<int, ObjectDetected>& detections);

    private:
        std::vector<std::shared_ptr<Tracker>> m_vLocalTrackers;

        void delete_tracks();
        void manage_new_tracks();
        void associate(std::vector<Eigen::Vector2f>& _selected_detections, Eigen::MatrixXi& _q, const std::unordered_map<int, ObjectDetected>& detections);
    };
}

#endif