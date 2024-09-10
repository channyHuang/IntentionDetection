#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <iostream>
#include <Eigen/Dense>
#include <numeric>

#include "track.h"
#include "hungarianAlg.h"
#include "ObjectDetected.h"

namespace IntentionDetection
{
    class Tracker
    {
    public:
        Tracker(const TrackerParam& param) : m_stParam(param) { ; }

        // track
        virtual void track(const std::unordered_map<int, ObjectDetected>& detections) { ; }
        virtual void track(const std::unordered_map<int, ObjectDetected>& detections, std::vector<bool>& _isAssoc, unsigned int& _trackID) { ; }
        virtual inline void push_back(const std::shared_ptr<Track>& _track) { tracks_.push_back(_track); }

        inline const unsigned int size() const { return tracks_.size(); }

        std::vector<std::shared_ptr<Track>> getTracks() { return tracks_; }

        inline std::vector<Eigen::Vector2f> getNotAssociated() const { return not_associated_; }

    protected:
        constexpr static unsigned int MAX_ASSOC = 10000;

        unsigned int m_nGlobalId = 0;
        bool m_bInited = false;
        bool m_bStartTracking = false;
        TrackerParam m_stParam;
        std::vector<std::shared_ptr<Track>> tracks_;
        std::vector<Eigen::Vector2f> prev_detections_;
        std::vector<Eigen::Vector2f> not_associated_;
        Eigen::MatrixXf beta_;
        Eigen::VectorXf last_beta_;
        std::vector<bool> analyze_tracks(const Eigen::MatrixXi& _q);
        //std::vector<bool> analyze_tracks(const cv::Mat& _q);
        std::vector<Eigen::MatrixXi> generate_hypothesis(const std::vector<Eigen::Vector2f>& _selected_detections, const Eigen::MatrixXi& _q);
        Eigen::MatrixXf joint_probability(const std::vector<Eigen::MatrixXi>& _association_matrices, const std::vector<Eigen::Vector2f>& _selected_detections);

    private:
        virtual void delete_tracks() { ; }
        virtual void manage_new_tracks() { ; }
        virtual void associate(std::vector<Eigen::Vector2f>& _selected_detections, Eigen::MatrixXi& _q, const std::vector<ObjectDetected>& detections) { ; }
        virtual void associate(std::vector<Eigen::Vector2f>& _selected_detections, Eigen::MatrixXi& _q, const std::vector<ObjectDetected>& detections, std::vector<bool>& _isAssoc) { ; }
    };
}

#endif
