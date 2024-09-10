#include "global_tracker.h"

using namespace IntentionDetection;

GlobalTracker::GlobalTracker(const TrackerParam& _param)
    : Tracker(_param) {
    m_nGlobalId = 0;
    m_bInited = true;
    m_bStartTracking = false;
}

void GlobalTracker::track(const std::unordered_map<int, ObjectDetected>& detections)
{
    if (m_bInited) {
        prev_detections_.clear();
        for (auto itr = detections.begin(); itr != detections.end(); itr++) {
            prev_detections_.push_back(Eigen::Vector2f(itr->second.position[0], itr->second.position[1]));
        }
        m_bInited = false;
    }
    else if (!m_bInited && !m_bStartTracking) {
        not_associated_.clear();
        for (auto itr = detections.begin(); itr != detections.end(); itr++) {
            not_associated_.push_back(Eigen::Vector2f(itr->second.position[0], itr->second.position[1]));
        }
        manage_new_tracks();
        if (m_vLocalTrackers.size() > 0) {
            tracks_.clear();
            tracks_ = m_vLocalTrackers.at(0)->getTracks();
            m_bStartTracking = true;
        }
        else {
            prev_detections_ = not_associated_;
        }
    }
    else {
        not_associated_.clear();
        std::vector<bool> isAssoc(detections.size(), false); //all the detections are not associated
        unsigned int i = 0;
        for (const auto& tracker : m_vLocalTrackers) {
            tracker->track(detections, isAssoc, m_nGlobalId);
        }
        for (const auto& ass : isAssoc) {
            if (!ass) {
                not_associated_.push_back(Eigen::Vector2f(detections.at(i).position[0], detections.at(i).position[1]));
            }
            ++i;
        }

        delete_tracks();

        tracks_.clear();
        for (const auto& tracker : m_vLocalTrackers) {
            std::vector<std::shared_ptr<Track>> tr = tracker->getTracks();
            for (const auto& t : tr) {
                tracks_.push_back(t);
            }
        }
        Eigen::MatrixXi q = Eigen::MatrixXi::Zero(detections.size(), tracks_.size());

        std::vector<Eigen::Vector2f> selected_detections;

        //ASSOCIATION
        std::vector<bool> not_associate;
        associate(selected_detections, q, detections);

        if (q.rows() * q.cols() > 0) {
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
                ++j;
                ++i;
            }
        }
        manage_new_tracks();
    }

}

void GlobalTracker::delete_tracks() {
    for (int i = m_vLocalTrackers.size() - 1; i >= 0; --i) {
        if (m_vLocalTrackers.at(i)->size() == 0) {
            m_vLocalTrackers.erase(m_vLocalTrackers.begin() + i);
        }
    }
}

void GlobalTracker::manage_new_tracks() {
    const unsigned int & prevDetSize = prev_detections_.size();
    const unsigned int & deteSize = not_associated_.size();
    if (prevDetSize == 0) {
        prev_detections_ = not_associated_;
    }
    else if (deteSize == 0) {
        prev_detections_.clear();
    }
    else {
        Eigen::MatrixXi assigmentsBin = Eigen::MatrixXi::Zero(prevDetSize, deteSize);
        Eigen::MatrixXf costMat = Eigen::MatrixXf::Zero(prevDetSize, deteSize);

        auto euclideanDist = [](const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
        {
            const Eigen::Vector2f& tmp = p1 - p2;
            return sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1));
        };

        assignments_t assignments;
        distMatrix_t costs(deteSize * prevDetSize);

        for (unsigned int i = 0; i < prevDetSize; ++i) {
            for (unsigned int j = 0; j < deteSize; ++j) {
                costs.at(i + j * prevDetSize) = euclideanDist(not_associated_.at(j), prev_detections_.at(i));
                costMat(i, j) = costs.at(i + j * prevDetSize);
            }
        }

        AssignmentProblemSolver APS;
        APS.Solve(costs, prevDetSize, deteSize, assignments, AssignmentProblemSolver::optimal);

        const unsigned int & assSize = assignments.size();

        for (unsigned int i = 0; i < assSize; ++i) {
            if (assignments[i] != -1 && costMat(i, assignments[i]) < m_stParam.fAssociationCostLocal) {
                assigmentsBin(i, assignments[i]) = 1;
            }
        }

        const unsigned int & rows = assigmentsBin.rows();
        const unsigned int & cols = assigmentsBin.cols();

        std::shared_ptr<Tracker> tracker = std::make_shared<LocalTracker>(m_stParam);

        for (unsigned int i = 0; i < rows; ++i) {
            for (unsigned int j = 0; j < cols; ++j) {
                if (assigmentsBin(i, j)) {
                    const float& vx = not_associated_.at(j).x() - prev_detections_.at(i).x();
                    const float& vy = not_associated_.at(j).y() - prev_detections_.at(i).y();
                    std::shared_ptr<Track> tr(new Track(m_stParam.fDt, m_stParam.vDelta, not_associated_.at(j).x(), not_associated_.at(j).y(),
                        vx, vy, m_stParam.fSigmaLocal, m_stParam.fGamma, m_stParam.matCovarance));
                    tracker->push_back(tr);
                }
            }
        }
        if (tracker->size() > 0) {
            m_vLocalTrackers.push_back(tracker);
        }

        Eigen::MatrixXi notAssignedDet = Eigen::MatrixXi::Zero(1, assigmentsBin.cols());
        for (unsigned int i = 0; i < assigmentsBin.rows(); ++i)
        {
            notAssignedDet += assigmentsBin.row(i);
        }

        prev_detections_.clear();
        for (unsigned int i = 0; i < notAssignedDet.cols(); ++i) {
            if (notAssignedDet(0, i) != 0) continue;
            const unsigned int & idx = i;
            prev_detections_.push_back(not_associated_.at(idx));
        }
    }
}

void GlobalTracker::associate(std::vector<Eigen::Vector2f >& _selected_detections, Eigen::MatrixXi& _q, const std::unordered_map<int, ObjectDetected>& detections)
{

    _q = Eigen::MatrixXi::Zero(detections.size(), tracks_.size() + 1);
    unsigned int validationIdx = 0;
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

    for (auto itr = detections.begin(); itr != detections.end(); itr++) {
        Eigen::Vector2f det;
        det << itr->second.position[0], itr->second.position[1];
        unsigned int i = 1;
        bool found = false;

        for (auto& track : tracks_) {
            const Eigen::Vector2f& tr = track->getLastPredictionEigen();

            const int& id = track->getId();
            const Eigen::Matrix2f& S = track->S().inverse();

            const float& mah = mahalanobis(S, tr, det);
            const float& eucl = euclideanDist(det, tr);
            if (mah <= m_stParam.fSigmaGlobal && eucl <= m_stParam.fAssociationCostGlobal) {
                _q(validationIdx, 0) = 1;
                _q(validationIdx, i) = 1;
                found = true;
            }
            ++i;
        }
        if (found) {
            _selected_detections.push_back(det);
            validationIdx++;
        }
        ++j;
    }


    Eigen::MatrixXi newq = Eigen::MatrixXi(validationIdx, tracks_.size() + 1);
    newq = _q.block(0, 0, validationIdx, tracks_.size() + 1);
    _q = Eigen::MatrixXi(validationIdx, tracks_.size() + 1);
    _q = newq;
}
