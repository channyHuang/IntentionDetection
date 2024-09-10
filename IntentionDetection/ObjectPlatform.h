#pragma once 

#include "Object.h"

class ObjectPlatform : public Object {
public:
    ObjectPlatform(int id_ = -1, int row_ = 540, int col_ = 960)
        : Object(id_, row_, col_) {
        eObjectType = Platform;

        minVelocity = 5;
        maxVelocity = 12;
    }

    virtual void updateTargetPos() {
        if (isSamePoint(curPos, targetPos)) {
            targetPos = Eigen::Vector2f(rand() % col, rand() % row);
        }
    }

    virtual void move() {
        updateTargetPos();

        int v = rand() % (maxVelocity - minVelocity) + minVelocity;
        Eigen::Vector2f dir = targetPos - curPos;
        float len = normalize(dir);

        Eigen::Vector2f vDir = dir * v / normalize(dir);

        if (len < std::fabs(v)) {
            curPos = targetPos;
        }
        else {
            curPos += vDir;
            roundPos(curPos, row, col);
        }

        if (dqPosBuffer.size() >= nBufferLen) {
            dqPosBuffer.pop_front();
        }
        dqPosBuffer.push_back(curPos);
        if (dqVelocityBuffer.size() >= nBufferLen) {
            dqVelocityBuffer.pop_front();
        }
        dqVelocityBuffer.push_back(vDir);
    }

protected:
    
};
