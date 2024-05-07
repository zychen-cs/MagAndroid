# Sets the minimum version of CMake required to build the native library.
cmake_minimum_required(VERSION 3.4.1)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})

add_library( # 设置生成库的名字
        mynativeso
        # 生成动态库
        SHARED
        # 指定源码文件，这里指定test.cpp文件
        ../jni/jninative.cpp)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../jni/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/ceres
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/ceres/internal
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/ceres/internal/miniglog
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/eigen3/)

#依赖的add库
#add_library(ceres SHARED IMPORTED)
#set_target_properties(ceres
#        PROPERTIES IMPORTED_LOCATION
#        ${CMAKE_CURRENT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libceres.so)

#find_library( # log库的别名
#        log-lib
        #log库
#        log )

target_link_libraries( # Specifies the target library.
        mynativeso
        ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libceres.so
        log)


##要求支持c++11
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#
##设置第三方库目录路径参数
#set(libs_path ${CMAKE_SOURCE_DIR}/../../../libs)
#
##引入glog头文件
#include_directories( ${libs_path}/include )
#
###########添加Ceres静态库 begin#########
#add_library(
#        ceres       #库名字 去掉了 lib 与 .a
#        SHARED      #必须的
#        IMPORTED    #必须的
#)
##指定Ceres静态库位置
#set_target_properties(
#        ceres
#        PROPERTIES IMPORTED_LOCATION
#        ${libs_path}/arm64-v8a/libceres.so
#)
##引入Ceres的头文件
#include_directories( ${libs_path}/include )
###########添加Ceres静态库 end#########
#
###########添加Eigen库 begin#########
##引入Eigen的头文件
#include_directories( ${libs_path}/include )
###########添加Eigen库 begin#########
#
###########添加添加native-lib动态库 begin#########
#add_library(
#        native-lib
#        SHARED
#        native-lib.cpp
#)
##查找要链接的android log库
#find_library(
#        log-lib
#        log )
##链接库
#target_link_libraries(
#        native-lib
#        ${log-lib}
#        ceres
#        #openmap库
#        #若编译Ceres时使用了 -DCERES_USE_OPENMP
#        #则在使用Ceres时必须链接上OpenMap库
#        #omp
#)
##########添加添加native-lib动态库 end#########





#cmake_minimum_required(VERSION 3.22.1)
#
## Declares and names the project.
#
#project("magandorid")
#
## Creates and names a library, sets it as either STATIC
## or SHARED, and provides the relative paths to its source code.
## You can define multiple libraries, and CMake builds them for you.
## Gradle automatically packages shared libraries with your APK.
#
#add_library( # Sets the name of the library.
#        magandorid
#
#        # Sets the library as a shared library.
#        SHARED
#
#        # Provides a relative path to your source file(s).
#        native-lib.cpp)
#
## Searches for a specified prebuilt library and stores the path as a
## variable. Because CMake includes system libraries in the search path by
## default, you only need to specify the name of the public NDK library
## you want to add. CMake verifies that the library exists before
## completing its build.
#
find_library( # Sets the name of the path variable.
        log-lib

        # Specifies the name of the NDK library that
        # you want CMake to locate.
        log)
#
## Specifies libraries CMake should link to your target library. You
## can link multiple libraries, such as libraries you define in this
## build script, prebuilt third-party libraries, or system libraries.
#
#target_link_libraries( # Specifies the target library.
#        magandorid
#
#        # Links the target library to the log library
#        # included in the NDK.
#        ${log-lib})