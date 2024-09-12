#pragma once 

#include <iostream>
#include <unordered_map>

#include "common.h"
using namespace IntentionDetection;
#include "ObjectAI.h"
#include "ObjectPlatform.h"

class GroupDetector {
public:
    GroupDetector() {}

    static std::unordered_map<int, std::shared_ptr<ObjectAI>> genObjects(Object::FLEETType eType, std::shared_ptr<ObjectPlatform> target, int row, int col) {
        std::unordered_map<int, std::shared_ptr<ObjectAI>> objects;
        
        Eigen::Vector2f pos;
        Eigen::Vector2f dist;
        int i;
        switch (eType) {
        case Object::FLEETType::Fleet_Circle:
        {
            int n = 8;
            float step = 360 / n;
            float angle = 0;
            for (i = 0; i < n; ++i) {
                pos = Eigen::Vector2f(rand() % col, rand() % row);

                std::shared_ptr<ObjectAI> object = std::make_shared<ObjectAI>(i, row, col);

                object->setCurPos(pos);
                object->setTarget(target);
                object->setSimulateIntention(Object::Intention_Surround);
                object->setSurroundAngle(angle);
                objects[i] = object;

                angle += step;
            }
        }
        break; 
        case Object::FLEETType::Fleet_Line:
        {
            for (i = 0; i < 5; ++i) {
                pos = Eigen::Vector2f(rand() % col, rand() % row);

                while (!validPos(pos + dist, row, col)) {
                    pos = Eigen::Vector2f(rand() % col, rand() % row);
                    break;
                }
                std::shared_ptr<ObjectAI> object = std::make_shared<ObjectAI>(i, row, col);
                object->setCurPos(pos);
                object->setOffsetPos(Eigen::Vector2f(i, i) * PADDING);
                object->setTarget(target);
                object->setSimulateIntention(Object::Intention_Attack);
                objects[i] = object;
            }
        }
            break;
        case Object::FLEETType::Fleet_Wedge:
        {
            int n = 5;
            for (int i = 0; i < n; ++i) {
                for (int j = i; j < n; ++j) {

                    pos = Eigen::Vector2f(rand() % col, rand() % row);

                    std::shared_ptr<ObjectAI> object = std::make_shared<ObjectAI>(i * n + j, row, col);
                    object->setCurPos(pos);
                    object->setOffsetPos(Eigen::Vector2f(i * PADDING, j * PADDING));
                    object->setTarget(target);
                    object->setSimulateIntention(Object::Intention_Attack);

                    objects[object->getId()] = object;
                }
            }
        }
            break;
        case Object::FLEETType::Fleet_Formation:
        {
            pos = Eigen::Vector2f(rand() % col, rand() % row);

            int n = 5;

            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {

                    pos = Eigen::Vector2f(rand() % col, rand() % row);

                    std::shared_ptr<ObjectAI> object = std::make_shared<ObjectAI>(i * n + j, row, col);
                    object->setCurPos(pos);
                    object->setOffsetPos(Eigen::Vector2f(i * PADDING, j * PADDING));
                    object->setTarget(target);
                    object->setSimulateIntention(Object::Intention_Attack);
                   
                    objects[object->getId()] = object;
                }
            }
        }
            break;
        case Object::FLEETType::Fleet_Herringborn:
        {
            int n = 5;
            float dist = 0;

            for (int i = 1; i <= n; ++i) {
                pos = Eigen::Vector2f(rand() % col, rand() % row);

                dist = 10 + i;

                std::shared_ptr<ObjectAI> object = std::make_shared<ObjectAI>(i, row, col);
                object->setCurPos(pos);
                object->setOffsetPos(Eigen::Vector2f(i* PADDING, dist));
                object->setTarget(target);
                object->setSimulateIntention(Object::Intention_Attack);

                objects[object->getId()] = object;

                std::shared_ptr<ObjectAI> object2 = std::make_shared<ObjectAI>(i + n, row, col);
                object2->setCurPos(pos);
                object2->setOffsetPos(Eigen::Vector2f(i* PADDING, -dist));
                object2->setTarget(target);
                object2->setSimulateIntention(Object::Intention_Attack);

                objects[object2->getId()] = object2;
            }
        }
            break;
        case Object::FLEETType::Fleet_Unknow:
        default:
            break;
        }
        return objects;
    }

    static Object::FLEETType detectFleetType(std::unordered_map<int, std::shared_ptr<Object>>& mapObjects) {
        if (mapObjects.size() <= 0) return Object::FLEETType::Fleet_Unknow;

        std::vector<Point> points;
        for (auto itr = mapObjects.begin(); itr != mapObjects.end(); itr++) {
            if (itr->second->isPlatform()) continue;
            points.push_back(Point(itr->second->getCurPos()[0], itr->second->getCurPos()[1]));
        }
        if (points.size() <= 0) return Object::FLEETType::Fleet_Unknow;

        std::vector<int> nEdgeCount;

        while (1) {
            std::vector<Point> hulls = convexHull(points);

            int len = points.size(), i;
            for (i = 0; i < len; ++i) {
                if (pointInHull(points[i], hulls)) {
                    std::swap(points[i], points[len - 1]);
                    len--;
                    i--;
                }
            }
            nEdgeCount.push_back(points.size() - len);

            if (nEdgeCount.size() == 1) { // if all in hulls
                if (nEdgeCount[0] == points.size()) {
                    if (isLine(points)) return Object::FLEETType::Fleet_Line;
                    if (isHumanType(hulls)) return Object::FLEETType::Fleet_Herringborn;
                    return Object::FLEETType::Fleet_Circle;
                }
            }
            if (len < 5 || points.size() == hulls.size()) {
                if (len > 0 && len < 5) nEdgeCount.push_back(len);
                break;
            }
            points.resize(len);
        }
        if (nEdgeCount.size() <= 1) {
            return Object::FLEETType::Fleet_Circle;
        }
        int d = nEdgeCount[0] - nEdgeCount[1], i;
        for (i = 1; i < nEdgeCount.size() - 1; ++i) {
            if (std::fabs(nEdgeCount[i] - nEdgeCount[i + 1] - d) > 3) break;
        }
        if (i == nEdgeCount.size() - 1) {
            if (nEdgeCount[nEdgeCount.size() - 1] >= 5) return Object::FLEETType::Fleet_Circle;
            if (nEdgeCount[nEdgeCount.size() - 1] == 3) return Object::FLEETType::Fleet_Formation;
            return Object::FLEETType::Fleet_Formation;
        }
        return Object::FLEETType::Fleet_Unknow;
    }

    static void moveCooperate(std::unordered_map<int, std::shared_ptr<Object>>& objects) {
        for (auto itr = objects.begin(); itr != objects.end(); itr++) {
            if (itr->second->isPlatform()) continue;

            Eigen::Vector2f curPos = itr->second->getCurPos();
            Eigen::Vector2f tarPos = itr->second->getTargetPos();

            int v = itr->second->getMaxVelocity();
            Eigen::Vector2f nextDir = (tarPos - curPos);
            Eigen::Vector2f finalDir = nextDir / normalize(nextDir);
            for (auto nitr = objects.begin(); nitr != objects.end(); nitr++) {
                if (nitr->second->getId() == itr->second->getId()) continue;
                if (normalize(nitr->second->getCurPos() - itr->second->getCurPos()) > itr->second->getMaxVelocity()) continue;
                Eigen::Vector2f forceDir = itr->second->getCurPos() - nitr->second->getCurPos();
                float len = normalize(forceDir);
                if (std::fabs(len) <= FLOAT_EPSILON) len = 1;
                finalDir += forceDir / len;
            }

            finalDir = finalDir / normalize(finalDir) * v;
            if (normalize(tarPos - curPos) < v) {
                itr->second->setCurPos(tarPos);
            }
            else {
                itr->second->setCurPos(curPos + finalDir);
            }
        }
    }

private:
};
