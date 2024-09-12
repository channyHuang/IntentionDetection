#include "ObjectControler.h"

#include <thread> 

#include "common.h"
#include "Intention.h"

std::shared_ptr<ObjectControler> ObjectControler::instance = nullptr;

void ObjectControler::init() {
#ifdef USE_SIMULATE
	std::shared_ptr<ObjectPlatform> player = std::make_shared<ObjectPlatform>(1, row, col);
	platforms[player->getId()] = player;
	objects = GroupDetector::genObjects(Array_Circle, player, row, col);
#endif

	IntentionDetection::TrackerParam params;
	m_pGlobalTracker = std::make_shared<IntentionDetection::GlobalTracker>(params);
	m_bInited = true;
}

void ObjectControler::reset() {
	objects.clear();
}

void ObjectControler::setGenData(std::unordered_map<int, std::shared_ptr<ObjectAI>> objs) {
	for (auto itr = m_setObjectsId.begin(); itr != m_setObjectsId.end(); itr++) {
		m_mapObjects.erase(*itr);
	}
	m_setObjectsId.clear();
	for (auto itr = objs.begin(); itr != objs.end(); itr++) {
		m_mapObjects[itr->first] = itr->second;
		m_setObjectsId.insert(itr->first);
	}
}

void ObjectControler::setPlatformData(std::unordered_map<int, std::shared_ptr<ObjectPlatform>> plats) {
	for (auto itr = m_setPlatformsId.begin(); itr != m_setPlatformsId.end(); itr++) {
		m_mapObjects.erase(*itr);
	}
	m_setPlatformsId.clear();
	for (auto itr = plats.begin(); itr != plats.end(); itr++) {
		m_mapObjects[itr->first] = itr->second;
		m_setPlatformsId.insert(itr->first);
	}
}

void ObjectControler::updatePlatforms(float fTimeStamp, std::unordered_map<int, ObjectDetected> msgs) {
	for (auto itr = msgs.begin(); itr != msgs.end(); itr++) {
		if (m_setPlatformsId.find(itr->first) == m_setPlatformsId.end()) {
			m_mapObjects[itr->first] = std::make_shared<ObjectPlatform>(itr->first);
			m_setPlatformsId.insert(itr->first);
		}
		m_mapObjects[itr->first]->setCurPos(Eigen::Vector2f(itr->second.position[0], itr->second.position[1]));
		m_mapObjects[itr->first]->setTimeStamp(fTimeStamp);
	}
	for (auto itr = m_setPlatformsId.begin(); itr != m_setPlatformsId.end();) {
		if (m_mapObjects[*itr]->getTimeStamp() < fTimeStamp) {
			m_mapObjects.erase(*itr);
			itr = m_setPlatformsId.erase(itr);
		}
		else {
			itr++;
		}
	}
}

void ObjectControler::updateMsgs(float fTimeStamp, std::unordered_map<int, ObjectDetected> msgs) {
	if (!m_bInited) {
		init();
		m_bInited = true;
	}

	// association
	m_pGlobalTracker->track(msgs);
	std::vector<std::shared_ptr<Track>> tracks = m_pGlobalTracker->getTracks();
	
	for (auto track : tracks) {
		int id = track->getId();
		if (id == -1) continue;

		auto itr = m_mapObjects.find(id);
		if (itr != m_mapObjects.end()) {
			itr->second->setCurPos(track->getLastPrediction());
			itr->second->setTimeStamp(fTimeStamp);
		}
		else {
			m_mapObjects[id] = std::make_shared<ObjectAI>(id);
			m_mapObjects[id]->setCurPos(track->getLastPrediction());
			m_mapObjects[id]->setTimeStamp(fTimeStamp);
			m_setObjectsId.insert(id);
		}
	}

	for (auto itr = m_mapObjects.begin(); itr != m_mapObjects.end(); ) {
		if (itr->second->isPlatform()) {
			itr++;
			continue;
		}
		if (itr->second->getTimeStamp() < fTimeStamp) {
			m_setObjectsId.erase(itr->second->getId());
			itr = m_mapObjects.erase(itr);
		}
		else {
			itr++;
		}
	}
}

nlohmann::json ObjectControler::to_json() {
	nlohmann::json plats, objs;
	nlohmann::json output;
	for (auto itr = m_mapObjects.begin(); itr != m_mapObjects.end(); itr++) {
		nlohmann::json obj = itr->second->to_json();
		if (itr->second->isPlatform()) {
			plats.emplace_back(obj);
		}
		else {
			objs.emplace_back(obj);
		}
	}
	output["objects"] = objs;
	output["platforms"] = plats;
	return output;
}

void ObjectControler::autoMove() {
	for (auto itr = m_mapObjects.begin(); itr != m_mapObjects.end(); itr++) {
		if (itr->second->isPlatform()) {
			itr->second->move();
		}
		else {
			itr->second->move(true);
		}
	}

	GroupDetector::moveCooperate(m_mapObjects);
}

void ObjectControler::predict(int nTargetId) {
	// group detect
	Object::FLEETType eFleetType = GroupDetector::detectFleetType(m_mapObjects);
	for (auto itr = m_mapObjects.begin(); itr != m_mapObjects.end(); itr++) {
		if (itr->second->isPlatform()) continue;
		itr->second->setFleetType(eFleetType);
	}
	// sub intention
	Intention::getInstance()->detectSubIntention(nTargetId, m_mapObjects, m_setObjectsId);
	// intention
	if (m_mapObjects.find(nTargetId) != m_mapObjects.end()) {
		Intention::getInstance()->detectIntention(nTargetId, m_mapObjects, m_setObjectsId, m_setPlatformsId);
	}
}
