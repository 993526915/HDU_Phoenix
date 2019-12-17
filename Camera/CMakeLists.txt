cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
project(hello)

#只需要修改路径
set(INC_DIRS /home/nuc/HDU_Phoenix/Camera/include)
set(LIB_DIRS /home/nuc/HDU_Phoenix/Camera/lib
/home/nuc/HDU_Phoenix/Camera
)
#链接库和.h
include_directories(
        ${INC_DIRS}
)
link_libraries(
        ${LIB_DIRS}
)
find_package(Threads REQUIRED)

find_package(OpenCV REQUIRED)
add_executable(hello test.cpp camera.cpp)

#链接
target_link_libraries(hello ${LIB_DIRS} libGCBase_gcc421_v3_0.so
libGenApi_gcc421_v3_0.so  libLog_gcc421_v3_0.so  liblog4cpp_gcc421_v3_0.so
libMathParser_gcc421_v3_0.so  libMediaProcess.so  libMvCameraControl.so
libMVGigEVisionSDK.so  libMVRender.so  libMvUsb3vTL.so 
libNodeMapData_gcc421_v3_0.so  libXmlParser_gcc421_v3_0.so
)
target_link_libraries(hello ${OpenCV_LIBS})
target_link_libraries(hello Threads::Threads)
