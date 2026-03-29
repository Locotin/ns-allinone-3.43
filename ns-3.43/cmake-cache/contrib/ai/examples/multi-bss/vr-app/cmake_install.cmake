# Install script for directory: /home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "default")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so"
         RPATH "/usr/local/lib:$ORIGIN/:$ORIGIN/../lib:/usr/local/lib64:$ORIGIN/:$ORIGIN/../lib64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/build/lib/libns3.43-vr-app-default.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so"
         OLD_RPATH "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/build/lib:::::"
         NEW_RPATH "/usr/local/lib:$ORIGIN/:$ORIGIN/../lib:/usr/local/lib64:$ORIGIN/:$ORIGIN/../lib64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.43-vr-app-default.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ns3" TYPE FILE FILES
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/burst-generator.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/burst-sink.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/bursty-application.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/my-random-variable-stream.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/seq-ts-size-frag-header.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/simple-burst-generator.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/trace-file-burst-generator.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/model/vr-burst-generator.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/helper/bursty-helper.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/helper/burst-sink-helper.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/contrib/ai/examples/multi-bss/vr-app/helper/bursty-app-stats-calculator.h"
    "/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/build/include/ns3/vr-app-module.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/angel/Documents/estocasticos/Taller/ns-allinone-3.43/ns-3.43/cmake-cache/contrib/ai/examples/multi-bss/vr-app/examples/cmake_install.cmake")

endif()

