#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <deque>

#include "ObjectAI.h"
#include "ObjectPlatform.h"

class Intention {
public:
	static std::shared_ptr<Intention> getInstance();

	~Intention();

	void reset();
	void step();
	void detectSubIntention(int nTargetId, std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId);
	void update(std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId, std::unordered_set<int> setPlatformsId);
	bool detectIntention(int nTargetId, std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId, std::unordered_set<int> setPlatformsId);
	bool isSurround(std::unordered_map<int, std::shared_ptr<Object>> mapObjects, std::unordered_set<int> setObjectsId, std::unordered_set<int> setPlatformsId, int id, bool bObjectBeSurround);

protected:
	Intention();

	static std::shared_ptr<Intention> instance;
	
	std::unordered_map<int, std::unordered_map<int, std::deque<float>>> mapPlatformDist2Object;
	int nSlideWindow = 10;
	float fMinDist = 20, fMaxDist = 50;
};