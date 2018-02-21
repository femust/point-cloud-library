project(StairsDetection)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.5 REQUIRED)
find_package( OpenCV REQUIRED )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(${PROJECT_NAME} main.cpp CloudHandler.cpp)
target_link_libraries (${PROJECT_NAME}  ${PCL_LIBRARIES})

add_executable(tests tests.cpp)
target_link_libraries (tests  ${PCL_LIBRARIES})

add_executable(visual visual.cpp)
target_link_libraries (visual  ${PCL_LIBRARIES})

add_executable(fouriertransform fouriertransform.cpp)
target_link_libraries (fouriertransform  ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(recognition recognition.cpp)
target_link_libraries (recognition  ${OpenCV_LIBS} ${PCL_LIBRARIES})



#find_package(Qt5Core REQUIRED)
#qt5_use_modules( ${PROJECT_NAME} Core )








