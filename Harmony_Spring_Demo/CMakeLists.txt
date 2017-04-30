#-----------------------------------------------------------------------------------------------
#  Run with "ccmake ." at the root directory, or use it as a template for 
#   starting your own programs
#-----------------------------------------------------------------------------------------------
SET(sampleName Harmony_Spring)
SET(PRJ_NAME "EXAMPLE_${sampleName}")

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
PROJECT(${PRJ_NAME})


SET(EXECUTABLE_OUTPUT_PATH ".")

#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED  base obs gui hmtslam slam kinematics nav hwdrivers maps)

#INCLUDE_DIRECTORIES()




# UWB src directory:
SET(UWB_INCLUDE_DIR_APP modules/sensors/src/uwb/application)
SET(UWB_INCLUDE_DIR_DEV modules/sensors/src/uwb)
SET(UWB_INCLUDE_DIR_UTIL modules/sensors/src/uwb/utils)
SET(UWB_INCLUDE_DIR_UWBLOC modules/sensors/src/uwb/uwb_loc)
SET(UWB_SRC_DIR modules/sensors/src/uwb)

# Sonar src directory:
SET(Sonar_INCLUDE_DIR modules/sensors/src/sonar)
SET(Sonar_SRC_DIR modules/sensors/src/sonar)

# motor drive src directory:
SET(Motor_INCLUDE_DIR modules/sensors/src/motor_driver)
SET(Motor_SRC_DIR modules/sensors/src/motor_driver)

#localization
SET(Localization_INCLUDE_DIR modules/localization/include)
SET(Localization_SRC_DIR modules/localization/src)

#planner
SET(Planner_INCLUDE_DIR modules/planner/include)
SET(Planner_SRC_DIR modules/planner/src)

#maps
SET(Maps_INCLUDE_DIR modules/maps/include)
SET(Maps_SRC_DIR modules/maps/src)



INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIRS} ${UWB_INCLUDE_DIR_APP} ${UWB_INCLUDE_DIR_DEV} ${UWB_INCLUDE_DIR_UTIL} ${UWB_INCLUDE_DIR_UWBLOC} ${Localization_INCLUDE_DIR} ${Planner_INCLUDE_DIR} ${Maps_INCLUDE_DIR}  ${Sonar_INCLUDE_DIR}  ${Motor_INCLUDE_DIR})



# use sse3 instruction set
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "-msse3")
endif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fpermissive -std=c++11 ")
#/*-std=c++11*/


# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${sampleName} src/main.cpp #../
	${UWB_SRC_DIR}/utils/utils.cpp
        ${UWB_SRC_DIR}/device/dw1000.cpp 
        ${UWB_SRC_DIR}/application/two_way_ranging.cpp 
	${UWB_SRC_DIR}/uwb_loc/uwb_hs.cpp        
	#${UWB_SRC_DIR}/network_utils.cpp ${UWB_INCLUDE_DIRS}
        ${Localization_SRC_DIR}/uwb_imu.cpp
        ${Localization_SRC_DIR}/ekf_localization.cpp
        ${Localization_SRC_DIR}/wheel_odometry.cpp
        ${Planner_SRC_DIR}/planner_robot.cpp
        ${Maps_SRC_DIR}/maps_hs.cpp
        ${Sonar_SRC_DIR}/HcSr04.cxx
        ${Sonar_SRC_DIR}/Servo.cxx
        ${Motor_SRC_DIR}/l298.cxx
   ) 

SET_TARGET_PROPERTIES(
	${sampleName} 
	PROPERTIES 
	PROJECT_LABEL "(EXAMPLE) ${sampleName}")

# Add special defines needed by this example, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${sampleName}  ${MRPT_LINKER_LIBS}
	${MRPT_LIBS} mraa  # This is filled by FIND_PACKAGE(MRPT ...)
	""  # Optional extra libs...
	)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


# -------------------------------------------------------------------------
# This part can be removed if you are compiling this program outside of 
#  the MRPT tree:
# -------------------------------------------------------------------------
IF(${CMAKE_PROJECT_NAME} STREQUAL "MRPT") # Fails if build outside of MRPT project.
	DeclareAppDependencies(${sampleName} ) # Dependencies
ENDIF(${CMAKE_PROJECT_NAME} STREQUAL "MRPT")
# -------------------------------------------------------------------------

