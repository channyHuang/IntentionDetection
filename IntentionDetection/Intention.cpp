#include "Intention.h"

#include "common.h"

#include <Eigen/Dense>

std::shared_ptr<Intention> Intention::instance = nullptr;

std::shared_ptr<Intention> Intention::getInstance() {
	if (instance == nullptr) {
		instance = std::shared_ptr<Intention>(new Intention);
	}
	return instance;
}

Intention::Intention() {}
Intention::~Intention() {}

void Intention::reset() {
	mapPlatformDist2Object.clear();
}

void Intention::step() {
	for (auto itr = mapPlatformDist2Object.begin(); itr != mapPlatformDist2Object.end(); itr++) {
		std::unordered_map<int, std::deque<float>> map = itr->second;
		for (auto ditr = map.begin(); ditr != map.end(); ditr++) {
			if (ditr->second.size() < nSlideWindow) continue;

			ditr->second.pop_front();
			if (ditr->second.empty()) {
				ditr = map.erase(ditr);
				ditr--;
			}
		}
	}
}

void Intention::detectSubIntention(int nTargetId, std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId) {
	if (mapObjects.find(nTargetId) == mapObjects.end()) {
		std::cout << "target id not found!!!" << std::endl;
		return;
	}

	for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
		float cosAngle = calcAngle(mapObjects.find(*oitr)->second->getVelocity(), mapObjects.find(nTargetId)->second->getCurPos() - mapObjects.find(*oitr)->second->getCurPos());
		float distance = normalize(mapObjects.find(nTargetId)->second->getCurPos() - mapObjects.find(*oitr)->second->getCurPos());
		if (((cosAngle > 0) && (normalize(mapObjects.find(*oitr)->second->getVelocity()) >= normalize(mapObjects.find(nTargetId)->second->getVelocity())))
			|| (distance < 20)) { // closer
			mapObjects.find(*oitr)->second->setSubIntention(Object::Intention_Attack);
		}
		else if ((distance >= 20 && cosAngle < 0 && normalize(mapObjects.find(*oitr)->second->getVelocity()) >= normalize(mapObjects.find(*oitr)->second->getVelocity()))
			|| (distance >= 100)) { // far far away
			mapObjects.find(*oitr)->second->setSubIntention(Object::Intention_Escape);
		}
		else {
			mapObjects.find(*oitr)->second->setSubIntention(Object::Intention_Track);
		}
	}
}

void Intention::update(std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId, std::unordered_set<int> setPlatformsId) {
	for (auto pitr = setPlatformsId.begin(); pitr != setPlatformsId.end(); pitr++) {
		auto ditr = mapPlatformDist2Object.find(*pitr);
		// new platform
		if (ditr == mapPlatformDist2Object.end()) {
			std::unordered_map<int, std::deque<float>> mapSinglePlatformDist2Object;
		
			for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
				float fDist = IntentionDetection::normalize(mapObjects.find(*oitr)->second->getCurPos() - mapObjects.find(*pitr)->second->getCurPos());
				
				std::deque<float> quDist;
				quDist.push_back(fDist);
				mapSinglePlatformDist2Object[*oitr] = quDist;
			}

			mapPlatformDist2Object[*pitr] = mapSinglePlatformDist2Object;
		}
		else { // exist platform
			std::unordered_map<int, std::deque<float>> &mapSinglePlatformDist2Object = ditr->second;

			for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
				float fDist = IntentionDetection::normalize(mapObjects.find(*oitr)->second->getCurPos() - mapObjects.find(*pitr)->second->getCurPos());
				
				auto itr = mapSinglePlatformDist2Object.find(*oitr);

				if (itr == mapSinglePlatformDist2Object.end()) {
					std::deque<float> quDist;
					quDist.push_back(fDist);

					mapSinglePlatformDist2Object[*oitr] = quDist;
				}
				else {
					if (itr->second.size() >= nSlideWindow) {
						itr->second.pop_front();
					}
					itr->second.push_back(fDist);
				}
			}
		}	
	}
}

bool Intention::isSurround(std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId, std::unordered_set<int> setPlatformsId, int id, bool bObjectBeSurround) {
	int nSurround = 0;
	if (bObjectBeSurround) {
		if (setObjectsId.find(id) == setObjectsId.end()) return false;
		for (auto itr = setPlatformsId.begin(); itr != setPlatformsId.end(); itr++) {
			if (normalize(mapObjects.find(*itr)->second->getCurPos() - mapObjects.find(id)->second->getCurPos()) > fMaxDist * 2) continue;
			float angle = calcAngle(mapObjects.find(*itr)->second->getCurPos() - mapObjects.find(id)->second->getCurPos(), Eigen::Vector2f(1, 0));
			if (mapObjects.find(*itr)->second->getCurPos().y() > mapObjects.find(id)->second->getCurPos().y()) {
				if (angle >= cos(EIGEN_PI * 2 / 5)) nSurround |= (1);
				else if (angle >= cos(EIGEN_PI * 4 / 5)) nSurround |= (1 << 1);
				else nSurround |= (1 << 2);
			}
			else {
				if (angle >= cos(EIGEN_PI * 8 / 5)) nSurround |= (1 << 3);
				else if (angle >= cos(EIGEN_PI * 6 / 5)) nSurround |= (1 << 4);
				else nSurround |= (1 << 2);
			}
		}
	}
	else {
		if (setPlatformsId.find(id) == setPlatformsId.end()) return false;
		for (auto itr = setObjectsId.begin(); itr != setObjectsId.end(); itr++) {
			if (normalize(mapObjects.find(*itr)->second->getCurPos() - mapObjects.find(id)->second->getCurPos()) > fMaxDist) continue;
			float angle = calcAngle(mapObjects.find(*itr)->second->getCurPos() - mapObjects.find(id)->second->getCurPos(), Eigen::Vector2f(1, 0));
			if (mapObjects.find(*itr)->second->getCurPos().y() > mapObjects.find(id)->second->getCurPos().y()) {
				if (angle >= cos(EIGEN_PI * 2 / 5)) nSurround |= (1);
				else if (angle >= cos(EIGEN_PI * 4 / 5)) nSurround |= (1 << 1);
				else nSurround |= (1 << 2);
			}
			else {
				if (angle >= cos(EIGEN_PI * 8 / 5)) nSurround |= (1 << 3);
				else if (angle >= cos(EIGEN_PI * 6 / 5)) nSurround |= (1 << 4);
				else nSurround |= (1 << 2);
			}
		}
	}
	return (nSurround == ((1 << 5) - 1));
}

bool Intention::detectIntention(int nTargetId, std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId, std::unordered_set<int> setPlatformsId) {
	step();
	update(mapObjects, setObjectsId, setPlatformsId);

	auto itr = mapPlatformDist2Object.find(nTargetId);
	if (itr == mapPlatformDist2Object.end()) {
		if (isSurround(mapObjects, setObjectsId, setPlatformsId, nTargetId, false)) {
			for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
				if (normalize(mapObjects.find(*oitr)->second->getCurPos() - mapObjects.find(nTargetId)->second->getCurPos()) > fMaxDist) continue;
				mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Surround);
			}
		}

		for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
			if (isSurround(mapObjects, setObjectsId, setPlatformsId, *oitr, true)) {
				mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Breakout);
			}
		}

		return false;
	}
	int nSurround = 0;
	for (auto oitr = itr->second.begin(); oitr != itr->second.end(); oitr++) {
		mapObjects[oitr->first]->setPreIntention(mapObjects[oitr->first]->getIntention());
		mapObjects[oitr->first]->setIntention(Object::Intention::Intention_Patrol);

		auto quDist = oitr->second;
		if (quDist.size() < nSlideWindow) continue;

		Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nSlideWindow, 2);	
		Eigen::VectorXf b = Eigen::VectorXf::Zero(nSlideWindow);
		int rowidx = 0;
		float average = 0;
		for (auto qitr = quDist.begin(); qitr != quDist.end(); qitr++) {
			A(rowidx, 0) = rowidx;
			A(rowidx, 1) = (*qitr);
			average += (*qitr);

			b(rowidx) = -1;
			rowidx++;
		}
		average /= quDist.size();
		Eigen::Vector2f x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

		float k = x(0) / x(1);

		if (average > 50 && k <= -0.1765) mapObjects[oitr->first]->setIntention(Object::Intention::Intention_Escape);
		else if (average <= 20 && k >= 0.1765) mapObjects[oitr->first]->setIntention(Object::Intention::Intention_Attack);
		else if (average > 20 && average <= 50 && k > -0.1765 && k < 0.1765) mapObjects[oitr->first]->setIntention(Object::Intention::Intention_Track);
	}

	if (isSurround(mapObjects, setObjectsId, setPlatformsId, nTargetId, false)) {
		for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
			if (normalize(mapObjects.find(*oitr)->second->getCurPos() - mapObjects.find(nTargetId)->second->getCurPos()) > fMaxDist) continue;
			mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Surround);
		}
	}

	for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
		if (isSurround(mapObjects, setObjectsId, setPlatformsId, *oitr, true)) {
			mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Breakout);
		}
	}

	for (auto oitr = setObjectsId.begin(); oitr != setObjectsId.end(); oitr++) {
		for (auto nitr = oitr; nitr != setObjectsId.end(); nitr++) {
			if (*oitr == *nitr) continue;

			if (mapObjects.find(*oitr)->second->getPreIntention() == Object::Intention::Intention_Attack && mapObjects.find(*nitr)->second->getPreIntention() == Object::Intention::Intention_Track) {
				if (mapObjects.find(*oitr)->second->getIntention() == Object::Intention::Intention_Track && mapObjects.find(*nitr)->second->getIntention() == Object::Intention::Intention_Attack) {
					mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Cover);
					mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Cover);
				}
			}
			else if (mapObjects.find(*oitr)->second->getPreIntention() == Object::Intention::Intention_Track && mapObjects.find(*nitr)->second->getPreIntention() == Object::Intention::Intention_Attack) {
				if (mapObjects.find(*oitr)->second->getIntention() == Object::Intention::Intention_Attack && mapObjects.find(*nitr)->second->getIntention() == Object::Intention::Intention_Track) {
					mapObjects.find(*oitr)->second->setIntention(Object::Intention::Intention_Cover);
					mapObjects.find(*nitr)->second->setIntention(Object::Intention::Intention_Cover);
				}
			}
		}
	}
}
