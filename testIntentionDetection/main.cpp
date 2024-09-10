
#pragma comment(lib, "./../../bin/Release/IntentionDetection.lib")

#include "./../IntentionDetection/IntentionDetector.h"
#include "./../IntentionDetection/json.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <future>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <magic_enum/magic_enum.hpp>




enum class FLEETType : int {
	Fleet_Line,
	Fleet_Herringborn,
	Fleet_Formation,
	Fleet_Wedge,
	Fleet_Circle,
	Fleet_Unknow
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

struct Object {
	int id;
	float position[2];
	float tx, ty;
	int row, col;
	FLEETType eFleetType;
	Intention eSubIntention;
	Intention eIntention;

	nlohmann::json to_json() {
		nlohmann::json param;
		param["id"] = id;
		param["position"] = { position[0], position[1] };
		param["velocity"] = { position[0], position[1] };
		return param;
	}

	bool from_json(nlohmann::json& param) {
		if (param.contains("id")) {
			id = param.at("id");
		}
		if (param.contains("position")) {
			position[0] = param.at("position")[0];
			position[1] = param.at("position")[1];
		}
		else return false;
		if (param.contains("fleettype")) {
			eFleetType = param.at("fleettype");
		}
		if (param.contains("subintention")) {
			eSubIntention = param.at("subintention");
		}
		if (param.contains("intention")) {
			eIntention = param.at("intention");
		}

		return true;
	}
};

cv::VideoWriter output_cap;

void callback(const char* data) {

	//printf("callback %s\n", data);
	cv::Mat image = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);

	std::vector<Object> vFinalObjects;
	nlohmann::json params = nlohmann::json::parse(data);
	if (params.contains("platforms") && params.at("platforms").is_array()) {
		nlohmann::json param = params.at("platforms");
		for (nlohmann::json par : param) {
			Object obj;
			obj.from_json(par);

			cv::circle(image, cv::Point(obj.position[0], obj.position[1]) - cv::Point(20, 20), 8, cv::Scalar(255, 0, 0), -1);
		}
	}

	if (params.contains("objects") && params.at("objects").is_array()) {
		nlohmann::json param = params.at("objects");
		for (auto par : param) {
			Object obj;
			obj.from_json(par);
			vFinalObjects.push_back(obj);
		}
	}

	for (int i = 0; i < vFinalObjects.size(); ++i) {
		cv::Scalar color = cv::Scalar(255, 0, 255);
		switch (vFinalObjects[i].eSubIntention) {
		case Intention_Attack:
			color = cv::Scalar(0, 0, 255);
			break;
		case Intention_Escape:
			color = cv::Scalar(0, 255, 0);
			break;
		case Intention_Track:
			color = cv::Scalar(0, 255, 255);
			break;
		default:
			break;
		}
		cv::circle(image, cv::Point2f(vFinalObjects[i].position[0], vFinalObjects[i].position[1]), 8, color, -1);
		cv::putText(image, std::to_string(vFinalObjects[i].id).c_str(), cv::Point2f(vFinalObjects[i].position[0], vFinalObjects[i].position[1]), cv::FONT_HERSHEY_SIMPLEX, 0.50, color, 2, cv::LINE_AA);
	}

	if (vFinalObjects.size() > 0) {
		cv::putText(image, magic_enum::enum_name(vFinalObjects[0].eFleetType).data(), cv::Point2f(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

		cv::putText(image, magic_enum::enum_name(vFinalObjects[0].eIntention).data(), cv::Point2f(10, 450), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
	}
	cv::imshow("Fleet&SubIntention", image);

	output_cap << image;

	cv::waitKey(100);
}

nlohmann::json parse(std::vector<Object>& objects, std::vector<Object> platforms = std::vector<Object>()) {
	std::chrono::time_point<std::chrono::system_clock> tpNow = std::chrono::system_clock::now();
	auto nsec = std::chrono::time_point_cast<std::chrono::nanoseconds>(tpNow).time_since_epoch().count();
	auto sec = nsec / std::pow(10, 9);
	
	nlohmann::json params;
	params["timestamp"] = sec;
	params["messagetype"] = 1;

	nlohmann::json jPlatforms;
	for (int i = 0; i < platforms.size(); ++i) {
		nlohmann::json platform = platforms[0].to_json();
		jPlatforms.emplace_back(platform);
	}
	if (platforms.size() > 0) {
		params["platforms"] = jPlatforms;
	}

	nlohmann::json jObjects;
	for (int i = 0; i < objects.size(); ++i) {
		nlohmann::json object = objects[i].to_json();
		jObjects.emplace_back(object);
	}
	params["objects"] = jObjects;

	return params;
}

void simulatePlatform(Object &platform) {
	if (std::abs(platform.tx - platform.position[0]) <= 8 && std::abs(platform.ty - platform.position[1]) <= 8) {
		platform.tx = rand() % platform.col;
		platform.ty = rand() % platform.row;
	}

	int v = 1;
	int dirx = platform.tx - platform.position[0], diry = platform.ty - platform.position[1];
	float len = std::sqrt(dirx * dirx + diry * diry);
	if (std::fabs(v * dirx / len + platform.position[0] - platform.tx) <= 8) platform.position[0] = platform.tx;
	else platform.position[0] += dirx / len;
	if (std::fabs(v * diry / len + platform.position[1] - platform.ty) <= 8) platform.position[1] = platform.ty;
	else platform.position[1] += diry / len;
}

std::vector<Object> objects;
std::vector<Object> platforms;
IntentionDetector detector;

#include <filesystem>
namespace fs = std::filesystem;
#include <fstream>
#include <iostream>
#include <unordered_map>

std::vector<std::string> splitString(const std::string& line, const char delimiter) {
	std::vector<std::string> res;
	if (line.length() <= 0) return res;
	size_t pos = 0;
	size_t npos = line.find_first_of(delimiter, pos);
	while (npos != std::string::npos) {
		res.push_back(line.substr(pos, npos - pos));
		pos = npos + 1;
		npos = line.find_first_of(delimiter, pos);
	}
	res.push_back(line.substr(pos));
	return res;
}

std::unordered_map<int, std::vector< cv::Rect>> petsReading(const std::string& _gt) {
	std::ifstream file(_gt);
	if (!file.is_open()) {
		std::cerr << "Error: cannot read the file: " << _gt << std::endl;
	}
	std::unordered_map<int, std::vector< cv::Rect > > pets;

	std::string line;
	char delimiter = ',';
	while (std::getline(file, line))
	{
		if (line.size() == 0) continue;
		std::vector<std::string> res = splitString(line, delimiter);
		if (res.size() <= 1) {
			std::cerr << "Bad data " << line << std::endl;
			continue;
		}
		int timeId = std::stoi(res[0]);
		int count = std::stoi(res[1]);
		if (res.size() < count * 4 + 2) {
			std::cerr << "Bad data " << line << std::endl;
			continue;
		}
		std::vector< cv::Rect> rects;
		for (int i = 0; i < count; ++i) {
			int idx = i * 4 + 2;
			cv::Rect rect(std::stof(res[idx]), std::stof(res[idx + 1]), std::stof(res[idx + 2]), std::stof(res[idx + 3]));
			rects.push_back(rect);
		}
		pets.insert(std::make_pair(timeId, rects));
	}
	return pets;
}

void sendThread() {
	for (int i = -5; i <= 5; ++i) {
		Object platform;
		platform.id = i;
		platform.row = platform.col = 500;
		platform.position[0] = rand() % platform.col;
		platform.position[1] = rand() % platform.row;
		platform.tx = rand() % platform.col;
		platform.ty = rand() % platform.row;

		if (i > 0) {
			objects.push_back(platform);
		} 
		else {
			platforms.push_back(platform);
		}
	}

	while (1) {
		for (int i = 0; i < objects.size(); ++i) {
			simulatePlatform(objects[i]);
		}
		for (int i = 0; i < platforms.size(); ++i) {
			simulatePlatform(platforms[i]);
		}

		nlohmann::json params = parse(objects, platforms);
		//std::cout << params.dump().c_str() << std::endl;
		detector.sendMessage(params.dump().c_str());

		cv::Mat image = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);;
		for (int i = 0; i < platforms.size(); ++i) {
			cv::circle(image, cv::Point2f(platforms[i].position[0], platforms[i].position[1]), 8, cv::Scalar(255, 0, 0), -1);
		}
		for (int i = 0; i < objects.size(); ++i) {
			cv::circle(image, cv::Point2f(objects[i].position[0], objects[i].position[1]), 8, cv::Scalar(0, 255, 0), -1);
		}
		cv::imshow("origin", image);
		cv::waitKey(10);

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		std::this_thread::yield();
	}
}

struct SendThread {
	explicit SendThread() {	}

	void run(int type) {
		mThread = std::thread([this, type]() {
			this->threadLoop(this->mExitSignal.get_future(), type);
			});
		mThread.detach();
	};

	void stop() {
		mExitSignal.set_value();
	}

private:
	std::thread mThread;
	std::promise<void> mExitSignal;

	void threadLoop(std::future<void> exitListener, int nCurType) {
		detector.reset();
		int index = 1;
		std::unordered_map<int, std::vector< cv::Rect>> detections;
		Object platform;

		switch (nCurType) {
		case 0:
			detector.autoGenerateData(true);
			break;
		case 1:
		{
			detections = petsReading("E:/projects/tracking/PETS09-S2L1/detection.txt");

			platform.id = -1;
			platform.row = platform.col = 500;
			platform.position[0] = rand() % platform.col;
			platform.position[1] = rand() % platform.row;
			platform.tx = rand() % platform.col;
			platform.ty = rand() % platform.row;

			platforms.push_back(platform);
			
		}
		break;
		case 2:
			detector.autoSurroundData(true);
			break;
		default:
			break;
		};

		do {
			switch (nCurType) {
			case 1:
			{
				simulatePlatform(platform);
				platforms.clear();
				platforms.push_back(platform);

				objects.clear();
				auto itr = detections.find(index);
				if (itr != detections.end()) {
					objects.resize(itr->second.size());
					for (int i = 0; i < itr->second.size(); ++i) {
						objects[i].id = i;
						objects[i].position[0] = (float)(itr->second[i].x + detections[index].at(i).width / 2);
						objects[i].position[1] = (float)(itr->second[i].y + detections[index].at(i).height / 2);
						objects[i].tx = objects[i].position[0];
						objects[i].ty = objects[i].position[0];
					}
				}
				else {
					break;
				}
				index++;

				nlohmann::json params = parse(objects, platforms);

				detector.sendMessage(params.dump().c_str());

				cv::Mat image = cv::Mat::zeros(cv::Size(768, 576), CV_8UC3);;
				for (int i = 0; i < objects.size(); ++i) {
					cv::circle(image, cv::Point2f(objects[i].position[0], objects[i].position[1]), 8, cv::Scalar(0, 255, 0), -1);
				}
				cv::imshow("origin", image);
				cv::waitKey(1);
			}
				break;
			case 2:
			{
				
			}
				break;
			default:
				break;
			}
		} while (exitListener.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);

		switch (nCurType) {
		case 0:
			detector.autoGenerateData(false);
			break;
		case 2:
			detector.autoSurroundData(false);
			break;
		default:
			break;
		};
	}
};

struct ThreadCallBack {
	explicit ThreadCallBack() {
		mThreadId = std::this_thread::get_id();
	}

	void run() {
		mThread = std::thread([this]() {
			this->threadLoop(this->mExitSignal.get_future());
			});
		mThread.detach();
	}

	void stop() {
		mExitSignal.set_value();
	};

	virtual ~ThreadCallBack() {}

private:
	std::thread::id mThreadId;
	std::thread mThread;
	std::promise<void> mExitSignal;

	void threadLoop(std::future<void> exitListener) {
		detector.setCallback(callback);
		do {
		} while (exitListener.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);
		detector.setCallback(nullptr);
	}
};

int main() {
	output_cap = cv::VideoWriter("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, cv::Size(500, 500));

	ThreadCallBack tCallback;
	tCallback.run();

	int c = -1;
	while (1) {
		SendThread tSendMessage;
		tSendMessage.run(c);

		std::cin >> c;
		if (c == -1) break;
		
		tSendMessage.stop();
	}

	tCallback.stop();
	output_cap.release();
	return 0;
}