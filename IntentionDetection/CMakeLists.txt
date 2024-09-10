CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

set(projectName IntentionDetection)
project(${projectName})

find_package(JNI REQUIRED)
find_package(Eigen3 REQUIRED)

FILE(GLOB FILES_C "*.cpp")
FILE(GLOB FILES_H "*.h" "*.hpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(MAGIC_ENUM_DIR "" CACHE PATH "")
set(CMAKE_CXX_STANDARD 17)

include_directories(${projectName}
	${EIGEN3_INCLUDE_DIR}
	${MAGIC_ENUM_DIR}
	${JNI_INCLUDE_DIRS}
	)

link_directories(${projectName}
	)

add_library(${projectName} STATIC
${FILES_C}
${FILES_H}
)

target_link_libraries(${projectName}
)
