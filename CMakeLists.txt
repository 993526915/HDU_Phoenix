cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
project(RM_CV)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

#只需要修改路径
set(INC_DIRS # /home/wzx/HDU_Phoenix/buff/Inc
             # /home/wzx/HDU_Phoenix/camera/cam_inc
             # /home/wzx/HDU_Phoenix/camera/Inc
             # /home/wzx/HDU_Phoenix/armor/Inc
             # /home/wzx/HDU_Phoenix/main/Inc
             # /home/wzx/HDU_Phoenix/serials/Inc
             # /home/wzx/HDU_Phoenix/general/Inc
             # /home/wzx/HDU_Phoenix/utilities/Inc
             buff/Inc
             camera/cam_inc
             camera/Inc
             armor/Inc
             main/Inc
             serials/Inc
             general/Inc
             angleSolve/Inc
             utilities/Inc)

set(LIB_DIRS ${PROJECT_SOURCE_DIR}/camera/lib)



#链接库和.h
include_directories(${INC_DIRS})
link_libraries(${LIB_DIRS})

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)

add_executable(RM_CV main2.cpp armor/Src/ArmorDector.cpp
                            camera/Src/camera_wrapper.cpp
                            angleSolve/Src/PNPSolver.cpp
                            main/Src/img_buffer.cpp
                            main/Src/ImgProdCons.cpp
                            serials/Src/serial.cpp
                            utilities/Src/opencv_extend.cpp
                            camera/Src/camera.cpp
                            buff/Src/Buff.cpp)

#链接
target_link_libraries(RM_CV ${LIB_DIRS}   libGCBase_gcc421_v3_0.so
                                        libGenApi_gcc421_v3_0.so  
                                        libLog_gcc421_v3_0.so  
                                        liblog4cpp_gcc421_v3_0.so
                                        libMathParser_gcc421_v3_0.so  
                                        libMediaProcess.so  
                                        libMvCameraControl.so
                                        libMVGigEVisionSDK.so  
                                        libMVRender.so  
                                        libMvUsb3vTL.so 
                                        libNodeMapData_gcc421_v3_0.so  
                                        libXmlParser_gcc421_v3_0.so)


target_link_libraries(RM_CV ${OpenCV_LIBS} ${GLOG_LIBRARY})
target_link_libraries(RM_CV Threads::Threads)
