#include "common.h"

#include <Eigen/Dense>

namespace IntentionDetection
{
    bool validPos(const Eigen::Vector2f& p, int row, int col) {
        if (p[0] < 0 || p[0] >= col) return false;
        if (p[1] < 0 || p[1] >= row) return false;
        return true;
    }

    bool roundPos(Eigen::Vector2f& p, int row, int col) {
        bool valid = true;
        if (p[0] < 0) {
            p[0] = 0;
            valid = false;
        }
        if (p[0] >= col) {
            p[0] = col - 1;
            valid = false;
        }
        if (p[1] < 0) {
            p[1] = 0;
            valid = false;
        }
        if (p[1] >= row) {
            p[1] = row - 1;
            valid = false;
        }
        return valid;
    }

    float normalize(const Eigen::Vector2f& p) {
        return std::sqrt(p[0] * p[0] + p[1] * p[1]);
    }

    // cos(theta)
    float calcAngle(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        return a.dot(b) / (normalize(a) * normalize(b));
    }

    bool isSamePoint(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
        return ((std::fabs(a[0] - b[0]) <= FLOAT_EPSILON) && (std::fabs(a[1] - b[1]) <= FLOAT_EPSILON));
    }

    std::vector<std::string> splitString(const std::string& line, const char delimiter) {
        std::vector<std::string> res;
        if (line.length() <= 0) return res;
        size_t pos = 0;
        size_t npos = line.find_first_of(delimiter, pos);
        while (npos != std::string::npos) {
            res.push_back(line.substr(pos, npos - pos));
            pos = npos + 1;
            npos = line.find_first_of(delimiter, pos);
        }
        res.push_back(line.substr(pos));
        return res;
    }

    float cross(const Point& O, const Point& A, const Point& B) {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }
    std::vector<Point> convexHull(std::vector<Point> vPts) {
        int n = vPts.size();
        int k = 0;
        std::vector<Point> H(2 * n);
        std::sort(vPts.begin(), vPts.end());
        for (int i = 0; i < n; ++i) {
            while (k >= 2 && cross(H[k - 2], H[k - 1], vPts[i]) <= 0) k--;
            H[k++] = vPts[i];
        }
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && cross(H[k - 2], H[k - 1], vPts[i]) <= 0) k--;
            H[k++] = vPts[i];
        }
        H.resize(k - 1);
        return H;
    }

    float normalize(const Point& p) {
        return std::sqrt(p.x * p.x + p.y * p.y);
    }
    float calcAngle(const Point& a, const Point& b) {
        return a.dot(b) / (normalize(a) * normalize(b));
    }
    bool isSamePoint(const Point& a, const Point& b) {
        return ((std::fabs(a.x - b.x) <= FLOAT_EPSILON) && (std::fabs(a.y - b.y) <= FLOAT_EPSILON));
    }
    bool pointInLine(const Point& target, const Point& line1, const Point& line2) {
        if (std::fabs(line1.x - line2.x) < LINE_EPSILON) { // horizontal line
            if (std::fabs(target.x - line1.x) < LINE_EPSILON || std::fabs(target.x - line2.x) < LINE_EPSILON) {
                return true;
            }
            return false;
        }
        float k = (line1.y - line2.y) / (line1.x - line2.x);
        float b = line1.y - k * line1.x;
        return (std::fabs(target.y - k * target.x - b) < LINE_EPSILON);
    }
    bool pointInHull(const Point& target, const std::vector<Point>& hulls) {
        if (hulls.size() <= 1) return true;
        for (int i = 0; i < hulls.size() - 1; ++i) {
            if (pointInLine(target, hulls[i], hulls[i + 1])) return true;
        }
        if (pointInLine(target, hulls[hulls.size() - 1], hulls[0])) return true;
        return false;
    }

    bool isLine(const std::vector<Point>& hulls) {
        int len = hulls.size();
        for (int i = 1; i < len - 1; ++i) {
            if (!pointInLine(hulls[i], hulls[0], hulls[len - 1])) return false;
        }
        return true;
    }

    bool isHumanType(const std::vector<Point>& hulls) {
        int len = hulls.size();
        if (len >= 7) return false;
        std::vector<int> count(len, 0);
        for (int i = 0; i < len - 1; ++i) {
            for (int j = 0; j < len; ++j) {
                if (pointInLine(hulls[j], hulls[i], hulls[i + 1])) count[i]++;
            }
        }
        int n = 0;
        for (int i = 0; i < len; ++i) {
            if (count[i] > 0) n++;
        }
        return (n <= 4);
    }

#ifdef USE_OPENCV
    cv::Mat combineImage(const cv::Mat& img1, const cv::Mat& img2) {
        cv::Mat image(cv::Size(img1.cols * 2, img1.rows), CV_8UC3);
        img1.copyTo(image(cv::Rect(0, 0, img1.cols, img1.rows)));
        img2.copyTo(image(cv::Rect(img1.cols, 0, img2.cols, img2.rows)));
        return image;
    }

    void draw(cv::Mat& img) {
        cv::Point2f pos(0, 0);
        //for (int i = 0; i < colors.size(); ++i) {
        //    cv::circle(img, pos + cv::Point2f(PADDING, i * PADDING + PADDING), 8, cv::Scalar(colors[i][0], colors[i][1], colors[i][2]), -1);
        //    cv::putText(img, sIntentions[i], pos + cv::Point2f(PADDING * 2, i * PADDING + PADDING), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(colors[i][0], colors[i][1], colors[i][2]), 2, cv::LINE_AA);
        //}
    }
#endif
}
