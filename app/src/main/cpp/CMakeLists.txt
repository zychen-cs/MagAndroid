# Sets the minimum version of CMake required to build the native library.
cmake_minimum_required(VERSION 3.4.1)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})

add_library(
        mynativeso

        SHARED

        ../jni/jninative.cpp)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../jni/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/ceres
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/ceres/internal
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/ceres/internal/miniglog
        ${CMAKE_CURRENT_SOURCE_DIR}/../jni/include/eigen3/)



target_link_libraries( # Specifies the target library.
        mynativeso
        ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libceres.so
        log)


#add_library(
#        native-lib
#        SHARED
#        native-lib.cpp
#)

#find_library(
#        log-lib
#        log )
##Link Libraary
#target_link_libraries(
#        native-lib
#        ${log-lib}
#        ceres
#        #openmap

#)



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