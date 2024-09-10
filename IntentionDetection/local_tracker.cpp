#include "local_tracker.h"

using namespace IntentionDetection;

LocalTracker::LocalTracker(const TrackerParam& _param)
  : Tracker(_param)
{ 
}

void LocalTracker::track(const std::unordered_map<int, ObjectDetected>& detections, std::vector<bool>& _isAssoc, unsigned int& _trackID)
{
    unsigned int j = 0;
    for (auto& track : tracks_) {
        track->predict();
        if (track->getId() == -1 && track->isAlive() && track->getEntropy() == Track::TrackState::ACCEPT) {
            track->setId(_trackID++);
        }
        ++j;
    }

    Eigen::MatrixXi q = Eigen::MatrixXi::Zero(detections.size(), tracks_.size());
    //cv::Mat_<int> q(cv::Size(tracks_.size(), detections.size()), int(0));
    std::vector<Eigen::Vector2f> selected_detections;

    //ASSOCIATION
    std::vector<bool> not_associate;
    associate(selected_detections, q, detections, _isAssoc);

    //NO ASSOCIATIONS
    if ((q.rows() * q.cols()) == 0) {
        for (const auto& track : tracks_) {
            track->notDetected();
        }
    }
    else {
        //CHECK ASSOCIATIONS
        not_associate = analyze_tracks(q); //ASSIGN ALL THE NOT ASSOCIATED TRACKS

        //HYPOTHESIS
        const std::vector<Eigen::MatrixXi>& association_matrices = generate_hypothesis(selected_detections, q);

        //COMPUTE JOINT PROBABILITY
        beta_ = joint_probability(association_matrices, selected_detections);
        last_beta_ = beta_.row(beta_.rows() - 1);

        //KALMAN PREDICT STEP
        unsigned int i = 0, j = 0;

        for (const auto& track : tracks_) {
            if (not_associate.at(j)) {
                track->gainUpdate(last_beta_(i));
            }
            j++;
            i++;
        }

        //UPDATE AND CORRECT
        i = 0, j = 0;
        for (const auto& track : tracks_) {
            if (not_associate.at(j)) {
                track->update(selected_detections, beta_.col(i), beta_(beta_.rows() - 1, i));
            }
            else {
                track->notDetected();
            }
            ++j;
            ++i;
        }
    }
    delete_tracks();
}

void LocalTracker::delete_tracks() {
    for (int i = tracks_.size() - 1; i >= 0; --i) {
        if (!tracks_.at(i)->isAlive() && tracks_.at(i)->getId() != -1) {
            tracks_.erase(tracks_.begin() + i);
        }
    }
}

void LocalTracker::associate(std::vector<Eigen::Vector2f >& _selected_detections, Eigen::MatrixXi& _q,
    const std::unordered_map<int, ObjectDetected>& _detections, std::vector<bool>& _isAssoc) {

    _q = Eigen::MatrixXi::Zero(_detections.size(), tracks_.size() + 1);
    unsigned int validationIdx = 0;
    not_associated_.clear();
    unsigned int j = 0;

    auto euclideanDist = [](const Eigen::Vector2f& _p1, const Eigen::Vector2f& _p2)
    {
        const Eigen::Vector2f& tmp = _p1 - _p2;
        return sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1));
    };

    auto mahalanobis = [](const Eigen::Matrix2f& R, const Eigen::Vector2f& a, const Eigen::Vector2f& b) -> float {
        float res = 0;
        for (int r = 0; r < R.rows(); ++r) {
            for (int c = 0; c < R.cols(); ++c) {
                res += R(r, c) * (a(r) - b(r)) * (a(c) - b(c));
            }
        }
        return std::sqrt(res);
    };

    for (auto itr = _detections.begin(); itr != _detections.end(); itr++)
    {
        Eigen::Vector2f det;
        det << itr->second.position[0], itr->second.position[1];
        unsigned int i = 1;
        bool found = false;

        for (auto& track : tracks_)
        {
            const Eigen::Vector2f& tr = track->getLastPredictionEigen();

            const int& id = track->getId();
            const Eigen::Matrix2f& S = track->S().inverse();

            const float& mah = mahalanobis(S, tr, det);
            const float& eucl = euclideanDist(det, tr);
            if (mah <= m_stParam.fSigmaLocal && eucl <= m_stParam.fAssociationCostLocal)
            {
                _q(validationIdx, 0) = 1;
                _q(validationIdx, i) = 1;
                found = true;
            }
            ++i;
        }
        if (found)
        {
            _selected_detections.push_back(det);
            _isAssoc.at(j) = true;
            validationIdx++;
        }
        else
        {
            not_associated_.push_back(det);
        }
        ++j;
    }

    Eigen::MatrixXi newq = Eigen::MatrixXi(validationIdx, tracks_.size() + 1);
    newq = _q.block(0, 0, validationIdx, tracks_.size() + 1);
    _q = Eigen::MatrixXi(validationIdx, tracks_.size() + 1);
    _q = newq;
}
