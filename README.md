# IntentionDetection
Simple Intention Detection

# Input-Output
## 参数设置输入数据json字段
| 字段 | 类型 | 说明 | 默认值 |
|:---:|:---:|:---:|:---:|
| messagetype | int | 0-更新;1-参数设置 | 0 |
| timestamp | float | 时间戳 | 0 |
| padding | int | 跟踪距离阈值，平台和目标距离小于该阈值一律视为进攻 | 20 |
## 数据更新输入数据json字段
| 序号 | 字段 | 类型 | 说明 | 
|:---:|:---:|:---:|:---:|
| 1 | messagetype | int | 0-更新;1-参数设置 |
| 2 | timestamp | float | 时间戳 |
| 3 | platforms | 数组 | 平台信息 |
| 3.1 | id | int | | 
| 3.2| position | float[2] | 平台位置 | 
| 3.3| velocity | float[2] | 平台速度 |
| 4| objects | 数组 | 目标信息 | 
| 4.1 | id | int | | 
| 4.2 | position | float[2] | 目标位置 | 
| 4.3 | velocity | float[2] | 目标速度 |
## 输出数据json格式
| 序号 | 字段 | 类型 | 说明 | 
|:---:|:---:|:---:|:---:|
| 1 | timestamp | float | 时间戳 |
| 2 | objects | 数组 | 目标信息 | 
| 2.1 | id | int | | 
| 2.2 | position | float[2] | 目标位置 | 
| 2.3 | array | int | 列队队形（一字形、人字形/雁群形、方阵、楔形、多环形/蜂群形、其它） | 
| 2.4 | arrayid | int | 列队队形id |
| 2.5 | subintention | int | 子意图（靠近、停留、离开） |
| 2.6 | intention | int | 父意图（进攻、逃跑、跟踪、掩护/突围、巡逻、包围、其它） |

# 外部接口
| 接口名称 | 参数 | 说明 | 
|:---:|:---:|:---:|
| IntentionDetector | - | 对外导出的类 |
| DetectorCallback | void(char*) | 回调函数 | 
| setCallback | void(DetectorCallback) | 设置回调 |
| sendMessage | void(char*) | 发送输入数据 |
| getCurrentSituation | const char*() | 获取离当前时刻最近的输出数据 |

# Example
## c++/windows
```c++
#include "IntentionDetector.h"
#pragma comment(lib, "IntentionDetection.lib")

void callback(const char* outputJson) {
    // do anything here
}

int main() {
    IntentionDetector detector;
    detector.setCallback(callback);
    // parse input data as json
    char* inputJson;
    detector.sendMessage(inputJson);

    return 0;
}
```
## Java/windows
```java
```
