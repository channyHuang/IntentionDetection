#ifndef _LOCAL_TRACKER_
#define _LOCAL_TRACKER_

#include "tracker.h"

namespace IntentionDetection
{
    class LocalTracker : public Tracker
    {
    public:
        LocalTracker(const TrackerParam& _param);
        virtual void track(const std::unordered_map<int, ObjectDetected>& detections, std::vector<bool>& _isAssoc, unsigned int& _trackID);
        
    private:
        void associate(std::vector<Eigen::Vector2f>& _selected_detections, Eigen::MatrixXi& _q, const std::unordered_map<int, ObjectDetected>& detections, std::vector<bool>& _isAssoc);
        virtual void delete_tracks();
    };
}

#endif