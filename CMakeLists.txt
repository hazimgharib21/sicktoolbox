cmake_minimum_required(VERSION 2.8.3)
project(sicktoolbox)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SickLD SickLMS1xx SickLMS2xx SickNAV350
  CATKIN_DEPENDS
  DEPENDS Threads
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Driver libraries
add_library(SickLD c++/drivers/ld/sickld/SickLD.cc c++/drivers/ld/sickld/SickLDBufferMonitor.cc c++/drivers/ld/sickld/SickLDMessage.cc)
target_link_libraries(SickLD ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickLMS1xx c++/drivers/lms1xx/sicklms1xx/SickLMS1xx.cc c++/drivers/lms1xx/sicklms1xx/SickLMS1xxBufferMonitor.cc c++/drivers/lms1xx/sicklms1xx/SickLMS1xxMessage.cc)
target_link_libraries(SickLMS1xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickLMS2xx c++/drivers/lms2xx/sicklms2xx/SickLMS2xx.cc c++/drivers/lms2xx/sicklms2xx/SickLMS2xxBufferMonitor.cc c++/drivers/lms2xx/sicklms2xx/SickLMS2xxMessage.cc)
target_link_libraries(SickLMS2xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickNAV350 c++/drivers/nav350/sicknav350/SickNAV350.cc c++/drivers/nav350/sicknav350/SickNAV350BufferMonitor.cc c++/drivers/nav350/sicknav350/SickNAV350Message.cc)
target_link_libraries(SickNAV350 ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS SickLD SickLMS1xx SickLMS2xx SickNAV350  
  #  ld_more_config ld_multi_sector ld_single_sector lms1xx_simple_app 
  #lms2xx_config lms2xx_mean_values lms2xx_partial_scan lms2xx_real_time_indices 
  #lms2xx_set_variant lms2xx_simple_app lms2xx_stream_range_and_reflect lms2xx_subrange
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## TODO Move headers that aren't needed externally back into the source
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

## TODO There are readmes, etc, create install rules for these
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
