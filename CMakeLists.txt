cmake_minimum_required(VERSION 2.8.11)
project(efmi-omni-sinusoid)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

link_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Ceres REQUIRED)
find_package(Qt5 REQUIRED
    Core
    Widgets)
find_package(GSL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_library(FMIBase src/fftMath.cc src/fmiImage.cc src/fmiRegistrationInit.cc src/fmiRegistration.cc include/misc_0.h)
target_link_libraries(FMIBase Qt5::Widgets Qt5::Core GSL::gsl)

set(SOURCE_FILES
        src/CameraModel.cpp
        src/OCamModel.cpp
        src/PCamModel.cpp
        src/CCamModel.cpp
        src/Frame.cpp
        src/CFrame.cpp
        src/Tracking.cpp
        src/CTracking.cpp
        src/CSystem.cpp
        src/misc_1.cpp
        )

add_library(OmniSineBase ${SOURCE_FILES})
target_link_libraries(OmniSineBase FMIBase ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(EOS_2imgs test/main.cpp)
target_link_libraries(EOS_2imgs OmniSineBase)

add_executable(TransFlow test/main_transflow.cpp)
target_link_libraries(TransFlow OmniSineBase)

# add_executable(OmniSinusoidOdom test/main_seq.cpp)
# target_link_libraries(OmniSinusoidOdom OmniSineBase)

# add_executable(OdomZ test/test_z_rpy.cpp)
# target_link_libraries(OdomZ OmniSineBase)

# add_executable(OdomVideo test/main_video.cpp)
# target_link_libraries(OdomVideo OmniSineBase)

# add_executable(OdomSeqVideo test/main_seq_video.cpp)
# target_link_libraries(OdomSeqVideo OmniSineBase)

# add_executable(FittingTest test/fitting_test.cpp)
# target_link_libraries(FittingTest ${CERES_LIBRARIES} ${OpenCV_LIBS})

# add_executable(Fish2Pano test/fish2pano.cpp)
# target_link_libraries(Fish2Pano ${OpenCV_LIBS})

# add_executable(Omni2Pano test/omni2pano.cpp)
# target_link_libraries(Omni2Pano ${OpenCV_LIBS})

# add_executable(MiscTest test/misc_test.cpp)
# target_link_libraries(MiscTest OmniSineBase)
