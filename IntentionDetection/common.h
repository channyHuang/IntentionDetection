#pragma once

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#define ANSI_COLOR_RED "\x1b[0;31m"
#define ANSI_COLOR_GREEN "\x1b[0;32m"
#define ANSI_COLOR_YELLOW "\x1b[0;33m"
#define ANSI_COLOR_BLUE "\x1b[0;34m"
#define ANSI_COLOR_MAGENTA "\x1b[0;35m"
#define ANSI_COLOR_CYAN "\x1b[0;36m"
#define ANSI_COLOR_WHITE "\x1b[0;37m"
#define ANSI_COLOR_BLACK "\x1b[0;30m"

#define ANSI_DELETE_LAST_LINE "\033[A\33[2K\r"
#define ANSI_DELETE_CURRENT_LINE "\33[2K\r"
#define ANSI_SCREEN_FLUSH std::fflush(stdout);

#ifndef EPSILON
#define FLOAT_EPSILON 1e-4
#define LINE_EPSILON 10
#define PADDING 20
#endif

#include <vector>
#include <string>

#include <Eigen/Dense>

namespace IntentionDetection
{
    extern bool validPos(const Eigen::Vector2f& p, int row, int col);
    extern bool roundPos(Eigen::Vector2f& p, int row, int col);
    extern float normalize(const Eigen::Vector2f& p);
    extern float calcAngle(const Eigen::Vector2f& a, const Eigen::Vector2f& b);
    extern bool isSamePoint(const Eigen::Vector2f& a, const Eigen::Vector2f& b);
    extern std::vector<std::string> splitString(const std::string& line, const char delimiter);

    class Point {
    public:
        Point(float _x = 0, float _y = 0) : x(_x), y(_y) {}

        float dot(const Point& p) const { return x * p.x + y * p.y; }

        bool operator < (const Point& p) const {
            return x < p.x || (x == p.x && y < p.y);
        }
    public:
        float x, y;
    };
    extern float normalize(const Point& p);
    extern float calcAngle(const Point& a, const Point& b);
    extern std::vector<Point> convexHull(std::vector<Point> vPts);
    extern bool isSamePoint(const Point& a, const Point& b);
    extern bool pointInLine(const Point& target, const Point& line1, const Point& line2);
    extern bool pointInHull(const Point& target, const std::vector<Point>& hulls);
    extern bool isLine(const std::vector<Point>& hulls);
    extern bool isHumanType(const std::vector<Point>& hulls);

#ifdef USE_OPENCV
    extern cv::Mat combineImage(const cv::Mat& img1, const cv::Mat& img2);
#endif
}
