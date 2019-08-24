cmake_minimum_required(VERSION 2.8)
PROJECT(main)
find_package(OpenCV 3.4.3)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(main /home/tromac/program/hero/main/main.cpp 
/home/tromac/program/hero/camera/DVP.cpp 
/home/tromac/program/hero/camera/DVP.h 
/home/tromac/program/hero/camera/DVPCamera.h
/home/tromac/program/hero/detector/ArmorDetector.cpp
/home/tromac/program/hero/detector/ArmorDetector.h
/home/tromac/program/hero/main/ThreadControl.cpp
/home/tromac/program/hero/main/ThreadControl.h
/home/tromac/program/hero/fusion/CoordinatesFusion.cpp
/home/tromac/program/hero/fusion/CoordinatesFusion.h
/home/tromac/program/hero/uart/serial.cpp
/home/tromac/program/hero/uart/serial.hpp
/home/tromac/program/hero/tools/MathTools.cpp
/home/tromac/program/hero/tools/MathTools.h)
target_link_libraries(main ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(main ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(main ${X11_LIBRARIES})