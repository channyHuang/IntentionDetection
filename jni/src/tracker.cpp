#include "tracker.h"

using namespace IntentionDetection;

#include "common.h"

Eigen::MatrixXf Tracker::joint_probability(const std::vector<Eigen::MatrixXi>& _association_matrices,
    const std::vector<Eigen::Vector2f>& selected_detections)
{
    auto mahalanobis = [](const Eigen::Matrix2f& R, const Eigen::Vector2f& a, const Eigen::Vector2f& b) -> float {
        float res = 0;
        for (int r = 0; r < R.rows(); ++r) {
            for (int c = 0; c < R.cols(); ++c) {
                res += R(r, c) * (a(r) - b(r)) * (a(c) - b(c));
            }
        }
        return std::sqrt(res);
    };

    unsigned int hyp_num = _association_matrices.size();
    Eigen::VectorXf Pr(_association_matrices.size());
    unsigned int validationIdx = _association_matrices.at(0).rows();
    unsigned int tracksize = tracks_.size();
    float prior;

    //Compute the total volume
    float V = 0.;
    for (const auto& track : tracks_)
    {
        V += track->getEllipseVolume();
    }

    for (unsigned int i = 0; i < hyp_num; ++i)
    {
        //I assume that all the measurments can be false alarms
        int false_alarms = validationIdx;
        float N = 1.;
        //For each measurement j: I compute the measurement indicator ( tau(j, X) ) 
        // and the target detection indicator ( lambda(t, X) ) 
        for (unsigned int j = 0; j < validationIdx; ++j)
        {
            //Compute the MEASURAMENT ASSOCIATION INDICATOR      
            const Eigen::MatrixXi& A_matrix = _association_matrices.at(i).block(j, 1, 1, tracksize);
            const int& mea_indicator = A_matrix.sum();
            ///////////////////////////////////////////////

            if (mea_indicator == 1)
            {
                //Update the total number of wrong measurements in X
                --false_alarms;

                //Detect which track is associated to the measurement j 
                //and compute the probability
                for (unsigned int notZero = 0; notZero < tracksize; ++notZero)
                {
                    if (A_matrix(0, notZero) == 1)
                    {
                        const Eigen::Vector2f& z_predict = tracks_.at(notZero)->getLastPredictionEigen();
                        const Eigen::Matrix2f& S = tracks_.at(notZero)->S();
                        const Eigen::Vector2f& diff = selected_detections.at(j) - z_predict;
                        //cv::Mat S_cv;
                        //cv::eigen2cv(S, S_cv);
                        ////const float& b = diff.transpose() * S.inverse() * diff;
                        //cv::Mat z_cv(cv::Size(2, 1), CV_32FC1);
                        //cv::Mat det_cv(cv::Size(2, 1), CV_32FC1);
                        //z_cv.at<float>(0) = z_predict(0);
                        //z_cv.at<float>(1) = z_predict(1);
                        //det_cv.at<float>(0) = selected_detections.at(j)(0);
                        //det_cv.at<float>(1) = selected_detections.at(j)(1);
                        //const float& b = cv::Mahalanobis(z_cv, det_cv, S_cv.inv());

                        float b = mahalanobis(S.inverse(), z_predict, selected_detections.at(j));
                        N = N / sqrt((2 * EIGEN_PI * S).determinant()) * exp(-b);
                    }
                }
            }

        }

        const float& likelyhood = N / float(std::pow(V, false_alarms));

        if (m_stParam.fProbDetection == 1)
        {
            prior = 1.;
        }
        else
        {
            //Compute the TARGET ASSOCIATION INDICATOR
            prior = 1.;
            for (unsigned int j = 0; j < tracksize; ++j)
            {
                const Eigen::MatrixXi& target_matrix = _association_matrices.at(i).col(j + 1);
                const int& target_indicator = target_matrix.sum();
                prior = prior * std::pow(m_stParam.fProbDetection, target_indicator) * std::pow((1 - m_stParam.fProbDetection), (1 - target_indicator));
            }
        }

        //Compute the number of events in X for which the same target 
        //set has been detected
        int a = 1;
        for (int j = 1; j <= false_alarms; ++j)
        {
            a = a * j;
        }

        Pr(i) = a * likelyhood * prior;
    }

    const float& prSum = Pr.sum();

    if (prSum != 0.)
        Pr = Pr / prSum; //normalization

      //Compute Beta Coefficients
    Eigen::MatrixXf beta(validationIdx + 1, tracksize);
    beta = Eigen::MatrixXf::Zero(validationIdx + 1, tracksize);


    Eigen::VectorXf sumBeta(tracksize);
    sumBeta.setZero();


    for (unsigned int i = 0; i < tracksize; ++i)
    {
        for (unsigned int j = 0; j < validationIdx; ++j)
        {
            for (unsigned int k = 0; k < hyp_num; ++k)
            {
                beta(j, i) = beta(j, i) + Pr(k) * _association_matrices.at(k)(j, i + 1);
            }
            sumBeta(i) += beta(j, i);
        }
        sumBeta(i) = 1 - sumBeta(i);
    }


    beta.row(validationIdx) = sumBeta;

    return beta;
}


std::vector<Eigen::MatrixXi> Tracker::generate_hypothesis(const std::vector<Eigen::Vector2f>& _selected_detections,
    const Eigen::MatrixXi& _q)
{
    unsigned int validationIdx = _q.rows();
    //All the measurements can be generated by the clutter track
    Eigen::MatrixXi A_Matrix(_q.rows(), _q.cols());
    A_Matrix = Eigen::MatrixXi::Zero(_q.rows(), _q.cols());
    A_Matrix.col(0).setOnes();
    std::vector<Eigen::MatrixXi> tmp_association_matrices(MAX_ASSOC, A_Matrix);

    unsigned int hyp_num = 0;
    //Generating all the possible association matrices from the possible measurements

    if (validationIdx != 0)
    {
        for (unsigned int i = 0; i < _q.rows(); ++i)
        {
            for (unsigned int j = 1; j < _q.cols(); ++j)
            {
                if (_q(i, j) == 1) // == 1
                {
                    tmp_association_matrices.at(hyp_num)(i, 0) = 0;
                    tmp_association_matrices.at(hyp_num)(i, j) = 1;
                    ++hyp_num;
                    if (j == _q.cols() - 1) continue;
                    for (unsigned int l = 0; l < _q.rows(); ++l)
                    {
                        if (l != i)
                        {
                            for (unsigned int m = j + 1; m < _q.cols(); ++m) // CHECK Q.COLS - 1
                            {
                                if (_q(l, m))
                                {
                                    tmp_association_matrices.at(hyp_num)(i, 0) = 0;
                                    tmp_association_matrices.at(hyp_num)(i, j) = 1;
                                    tmp_association_matrices.at(hyp_num)(l, 0) = 0;
                                    tmp_association_matrices.at(hyp_num)(l, m) = 1;
                                    ++hyp_num;
                                } //if(q.at<int>(l, m))
                            }// m
                        } // if l != i
                    } // l
                } // if q(i, j) == 1
            } // j
        } // i
    }
    /////////////////////////////////////////////////////////////////////////////////
    std::vector<Eigen::MatrixXi> association_matrices(hyp_num + 1);
    std::copy(tmp_association_matrices.begin(), tmp_association_matrices.begin() + hyp_num + 1,
        association_matrices.begin());
    return association_matrices;
}

std::vector<bool> Tracker::analyze_tracks(const Eigen::MatrixXi& _q) {
    const Eigen::MatrixXi& m_q = _q.block(0, 1, _q.rows(), _q.cols() - 1);
    Eigen::MatrixXi col_sum = Eigen::MatrixXi::Zero(1, m_q.cols());

    std::vector<bool> not_associate(m_q.cols(), true);
    for (int i = 0; i < m_q.rows(); ++i) {
        col_sum += m_q.row(i);
    }
    
    for (int i = 0; i < col_sum.cols(); ++i) {
        if (col_sum(0, i) != 0) continue;
        not_associate.at(i) = false;
    }

    return not_associate;
}

//std::vector<bool> Tracker::analyze_tracks(const cv::Mat& _q) {
//    const cv::Mat& m_q = _q(cv::Rect(1, 0, _q.cols - 1, _q.rows));
//    cv::Mat col_sum(cv::Size(m_q.cols, 1), _q.type(), cv::Scalar(0));
//
//    std::vector<bool> not_associate(m_q.cols, true); //ALL TRACKS ARE ASSOCIATED
//    for (unsigned int i = 0; i < m_q.rows; ++i)
//    {
//        col_sum += m_q.row(i);
//    }
//    cv::Mat nonZero;
//    col_sum.convertTo(col_sum, CV_8UC1);
//
//
//    cv::Mat zero = col_sum == 0;
//    cv::Mat zeroValues;
//    cv::findNonZero(zero, zeroValues);
//
//    for (unsigned int i = 0; i < zeroValues.total(); ++i)
//    {
//        not_associate.at(zeroValues.at<cv::Point>(i).x) = false;
//    }
//    return not_associate;
//}