#pragma once 

#include <iostream>
#include <deque>

#include <Eigen/Dense>

#include <magic_enum/magic_enum.hpp>

#include "common.h"
#include "json.hpp"
using namespace IntentionDetection;

class Object {
public:
    enum ObjectType {
        AI = 0,
        Platform
    };

    enum class FLEETType : int {
        Fleet_Line,
        Fleet_Herringborn,
        Fleet_Formation,
        Fleet_Wedge,
        Fleet_Circle,
        Fleet_Unknow
    };

    enum DrawType {
        Draw_Origin,
        Draw_SubIntention,
        Draw_Intention
    };

    enum Intention {
        // sub intention
        Intention_Attack,
        Intention_Escape,
        Intention_Track,
        
        Intention_Patrol,

        // parent intention
        Intention_Surround,
        Intention_Breakout,
        Intention_Cover,
        Intention_Unknow
    };


public:
    Object(int id_ = -1, int row_ = 540, int col_ = 960) 
        : id(id_), row(row_), col(col_) {
        curPos[0] = rand() % col;
        curPos[1] = rand() % row;

        targetPos = curPos;
    }

    virtual void updateTargetPos() {}
    virtual void move(bool bCoop = false) {}

    // get - set
    int getId() { return id; }

    bool isPlatform() { return (eObjectType == Platform); }

    long getTimeStamp() { return lTimeStamp; }
    void setTimeStamp(long stamp) { lTimeStamp = stamp; }

    Eigen::Vector2f getCurPos() { return curPos; }
    void setCurPos(const Eigen::Vector2f& pos) { curPos = pos; }

    Eigen::Vector2f getTargetPos() { return targetPos; }
    Eigen::Vector2f getOriginTargetPos() { return (pTargetObject == nullptr ? Eigen::Vector2f(0, 0) : pTargetObject->getCurPos()); }

    void setOffsetPos(const Eigen::Vector2f& pos = Eigen::Vector2f(0, 0)) { offsetPos = pos; }

    void setSurroundAngle(float angle) { fSurroundAngle = angle; }

    void setVelocity(int maxVelocity_ = 10, int minVelocity_ = 0) {
        maxVelocity = maxVelocity_;
        minVelocity = minVelocity_;
    }
    float getMaxVelocity() { return maxVelocity; }

    void setFleetType(FLEETType type) { eFleetType = type; }
    FLEETType getFleetType() { return eFleetType; }

    void setSubIntention(Intention inten) { eSubIntention = inten; }
    Intention getSubIntention() { return eSubIntention; }

    void setIntention(Intention inten) { eIntention = inten; }
    Intention getIntention() { return eIntention; }

    void setPreIntention(Intention inten) { ePreIntention = inten; }
    Intention getPreIntention() { return ePreIntention; }

    void setSimulateIntention(Intention eSimulateIntention_ = Intention_Patrol) { eSimulateIntention = eSimulateIntention_; }

    void setTarget(std::shared_ptr<Object> obj = nullptr) { pTargetObject = obj; }
    std::shared_ptr<Object> getTarget() { return pTargetObject; }

    void setParent(std::shared_ptr<Object> obj = nullptr) { pParentObject = obj; }
    std::shared_ptr<Object> getParent() { return pParentObject; }

    Eigen::Vector3i getColor() { return vColorBGR; }
    void setColor(Eigen::Vector3i color_ = Eigen::Vector3i(0, 0, 0)) { vColorBGR = color_; }
    // end of get - set

    Eigen::Vector2f getVelocity() { return (dqVelocityBuffer.size() <= 0 ? Eigen::Vector2f(0, 0) : dqVelocityBuffer.back()); }

    Eigen::Vector3i getIntentionColor() {
        return colors[eIntention];
    }

    Eigen::Vector3i getSubIntentionColor() {
        Eigen::Vector3i color = colors[eSubIntention];
        return color;
    }

    virtual nlohmann::json to_json() {
        nlohmann::json param;
        param["id"] = id;
        param["position"] = { curPos[0], curPos[1] };
        param["fleettype"] = eFleetType;
        param["fleetid"] = 0;
        param["subintention"] = eSubIntention;
        param["intention"] = eIntention;
        return param;
    }

//#ifdef USE_SIMULATE
    Eigen::Vector2f getTextPadding() { return (id < 0 ? Eigen::Vector2f(8, 8) : Eigen::Vector2f(-8, -8)); }
//#endif

public:
    std::vector<Eigen::Vector3i> colors = {
        Eigen::Vector3i(0, 0, 255),
        Eigen::Vector3i(0, 255, 255),
        Eigen::Vector3i(0, 255, 0),
        Eigen::Vector3i(255, 255, 255),
        Eigen::Vector3i(255, 255, 0),
        Eigen::Vector3i(255, 0, 255),
        Eigen::Vector3i(255, 0, 0),
        Eigen::Vector3i(128, 128, 128)
    };

protected:
    // common feature
    ObjectType eObjectType;
    int id;
    int row, col;
    long lTimeStamp;
    int maxVelocity = 10, minVelocity = 1;
    int nBufferLen = 10;
    Eigen::Vector3i vColorBGR = Eigen::Vector3i(0, 0, 0);
    Eigen::Vector2f curPos, targetPos, offsetPos = Eigen::Vector2f(0, 0);
    std::shared_ptr<Object> pTargetObject = nullptr;
    std::deque<Eigen::Vector2f> dqPosBuffer, dqVelocityBuffer;
    // platform feature

    // ai feature
    int nCornerIdx = 0;
    const int nCornerPadding = 20;
    float fTrackPadding = 50;
    float fSurroundAngle = -30;
    Intention eSimulateIntention = Intention_Surround;
    FLEETType eFleetType = FLEETType::Fleet_Unknow;
    Intention eSubIntention = Intention_Unknow, eIntention = Intention_Unknow, ePreIntention = Intention_Unknow;
    std::shared_ptr<Object> pParentObject = nullptr;
    std::vector<Eigen::Vector2f> corners;

private:
};
