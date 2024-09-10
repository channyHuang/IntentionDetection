#pragma once 


#include <unordered_set>
#include <memory>
#include <queue>

#include "json.hpp"
#include "ObjectAI.h"
#include "ObjectPlatform.h"
#include "global_tracker.h"
#include "groupDetecter.h"

extern class IntentionDetector;

class ObjectControler {
public:
	static std::shared_ptr<ObjectControler> getInstance() {
		if (instance == nullptr) {
			instance = std::make_shared<ObjectControler>();
		}
		return instance;
	}

	ObjectControler(int row_ = 540, int col_ = 960)
		: row(row_), col(col_) {}

	void reset();
	void init();
	void updatePlatforms(float fTimeStamp, std::unordered_map<int, ObjectDetected> msgs);
	void updateMsgs(float fTimeStamp, std::unordered_map<int, ObjectDetected> msgs);
	nlohmann::json to_json();
	std::queue<std::vector<ObjectDetected>> g_quObjectMsg;
	void autoMove();
	void predict(int nTargetId = 0);

	void setGenData(std::unordered_map<int, std::shared_ptr<ObjectAI>> objs);
	void setPlatformData(std::unordered_map<int, std::shared_ptr<ObjectPlatform>> plats);

private:
	static std::shared_ptr<ObjectControler> instance;

	std::unordered_map<int, std::shared_ptr<Object>> m_mapObjects;
	std::unordered_set<int> m_setObjectsId;
	std::unordered_set<int> m_setPlatformsId;
	std::unordered_map<int, std::shared_ptr<ObjectAI>> objects;
	std::unordered_map<int, std::shared_ptr<ObjectPlatform>> platforms;
	bool m_bInited = false;
	int row, col;
	float alpha = 0.2;
	float omiga[4] = { 0.1, 0.3, 0.3, 0.3 };
	float bias[4] = { 1, 1, 1, 1 };
	Object::FLEETType eFleetType;
	float m_fTimestamp = 0;
	// association
	std::shared_ptr<IntentionDetection::Tracker> m_pGlobalTracker;
};
