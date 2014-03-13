cmake_minimum_required(VERSION 2.8)
project(kinectLib_2.4)

find_package( OpenCV 2.4 REQUIRED  )

include_directories(
/usr/include/
/usr/include/ni/
./
${OpenCV_INCLUDE_DIRS}
)

#include_directories(
#/usr/include/
#/usr/include/ni/
#${OpenCV_INCLUDE_DIRS}
#)

set(src
kinectInterface.h
kinectInterface.cpp
kinectException.h
kinectException.cpp
viewerOpenCV.h
viewerOpenCV.cpp
calibrateOpenCV.cpp
calibrateOpenCV.h
)

link_directories(
/usr/lib/
)

add_definitions( -DCMAKE_FULL_SKEL )

set(EXECUTABLE_OUTPUT_PATH ../)

add_executable(
	kinectRecord
	./bin/record.cpp
	${src}
)

target_link_libraries(
	kinectRecord
	OpenNI
    ${OpenCV_LIBS}
    boost_filesystem
    boost_system
)

add_executable(
	kinectPlayBack
	./bin/playBack.cpp
	${src}
)

target_link_libraries(
	kinectPlayBack
	OpenNI
    ${OpenCV_LIBS}
    boost_filesystem
    boost_system
)

add_executable(
	kinectCalibrate
	./bin/calibrate.cpp
	${src}
)

target_link_libraries(
	kinectCalibrate
	OpenNI
    ${OpenCV_LIBS}
    boost_filesystem
    boost_system
)

add_executable(
        playBackSkel
        ./bin/playBackSkel.cpp
        ${src}
)

target_link_libraries(
        playBackSkel
        OpenNI
    ${OpenCV_LIBS}
    boost_filesystem
    boost_system
)


