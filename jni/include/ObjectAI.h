#pragma once 

#include "Object.h"

class ObjectAI : public Object {
public:
    ObjectAI(int id_ = -1, int row_ = 540, int col_ = 960)
        : Object(id_, row_, col_) {
        corners.push_back(Eigen::Vector2f(nCornerPadding, nCornerPadding));
        corners.push_back(Eigen::Vector2f(col - nCornerPadding, nCornerPadding));
        corners.push_back(Eigen::Vector2f(col - nCornerPadding, row - nCornerPadding));
        corners.push_back(Eigen::Vector2f(nCornerPadding, row - nCornerPadding));

        eObjectType = AI;
    }

    virtual void updateTargetPos() {
        switch (eSimulateIntention) {
        case Intention_Track:
        case Intention_Attack:
        case Intention_Cover:
            if (pTargetObject != nullptr) {
                targetPos = pTargetObject->getCurPos() + offsetPos;
            }
            break;
        case Intention_Escape:
            if (pTargetObject != nullptr) {
                targetPos = 2 * getCurPos() - pTargetObject->getCurPos();
            }
            break;
        case Intention_Surround:
            if (pTargetObject != nullptr) {
                targetPos = pTargetObject->getCurPos() + Eigen::Vector2f(std::cos(fSurroundAngle * EIGEN_PI / 180), -std::sin(fSurroundAngle * EIGEN_PI / 180)) * fTrackPadding;
                roundPos(targetPos, row, col);
            }
            break;
        case Intention_Patrol:
            if (isSamePoint(curPos, targetPos + offsetPos)) {
                targetPos = corners[nCornerIdx++] - offsetPos;
                if (nCornerIdx >= corners.size()) nCornerIdx = 0;
            }
            break;
        default:
            if (isSamePoint(curPos, targetPos + offsetPos)) {
                Eigen::Vector2f p(rand() % col, rand() % row);
                targetPos = p;
            }
            break;
        }
    }

    virtual void move(bool bCoop = false) {
        if (pParentObject != nullptr) { return; }

        updateTargetPos();

        if (bCoop) return;

        int v = rand() % (maxVelocity - minVelocity) + minVelocity;
        Eigen::Vector2f dir = targetPos + offsetPos - curPos;
        float len = normalize(dir);

        switch (eSimulateIntention) {
        case Intention_Track:
            if (len < fTrackPadding) v = 0;
            break;
        default:
            break;
        }

        Eigen::Vector2f vDir = dir * v / normalize(dir);

        if (len < std::fabs(v)) {
            curPos = targetPos + offsetPos;
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

    void subIntention() {
        if (pTargetObject == nullptr) {
            eSubIntention = Intention_Unknow;
            return;
        }
        float cosAngle = calcAngle(getVelocity(), pTargetObject->getCurPos() - getCurPos());
        float distance = normalize(pTargetObject->getCurPos() - getCurPos());
        if (((cosAngle > 0) && (normalize(getVelocity()) >= normalize(pTargetObject->getVelocity())))
            || (distance < 20)) { // closer
            eSubIntention = Object::Intention_Attack;
        }
        else if ((distance >= 20 && cosAngle < 0 && normalize(getVelocity()) >= normalize(pTargetObject->getVelocity()))
            || (distance >= 100)) { // far far away
            eSubIntention = Object::Intention_Escape;
        }
        else {
            eSubIntention = Object::Intention_Track;
        }
    }

private:
};
