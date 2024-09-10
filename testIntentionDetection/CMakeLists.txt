CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

set(projectName testIntentionDetection)
project(${projectName})

FILE(GLOB FILES_C "*.cpp")

find_package(OpenCV REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(MAGIC_ENUM_DIR "" CACHE PATH "")
set(CMAKE_CXX_STANDARD 17)

include_directories(${projectName}
	${OpenCV_INCLUDE_DIRS}
	${MAGIC_ENUM_DIR}
)

link_directories(${projectName}
	)

add_executable(${projectName} 
${FILES_C}
)

target_link_libraries(${projectName}
	${OpenCV_LIBS}
)