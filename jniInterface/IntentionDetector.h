#pragma once

#include "export.h"

typedef void(* DetectorCallback)(const char*);

class INTENTIONDETECTOR_EXPORT IntentionDetector
{
public:
	IntentionDetector();
	~IntentionDetector();

	void setCallback(DetectorCallback pCallback = nullptr);
	bool sendMessage(const char* pData);
	const char* getCurrentSituation();

	void autoGenerateData();

public:
	DetectorCallback m_pCallback = nullptr;
};
