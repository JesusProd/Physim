##########################################################
#################### Setup example #######################
##########################################################

set(EXAMPLE_NAME SimulatorLite_Cylinders)

##########################################################
################# Gather source files ####################
##########################################################

file(GLOB_RECURSE SRCFILES source/*.cpp)
file(GLOB_RECURSE INCFILES include/*.h)

source_group(TREE ${PROJECT_SOURCE_DIR}/examples/SimulatorLite_Cylinders/source/ PREFIX Source FILES ${SRCFILES})
source_group(TREE ${PROJECT_SOURCE_DIR}/examples/SimulatorLite_Cylinders/include/ PREFIX Source FILES ${INCFILES})

#########################################################
################## Create Executable ####################
#########################################################

add_executable(${EXAMPLE_NAME} ${SRCFILES} ${INCFILES} ${NVISFILES})
target_include_directories(${EXAMPLE_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${EXAMPLE_NAME} ${PROJECT_NAME})

foreach (LIBLINK_RELEASE ${LIBLINKS_RELEASE_LIST})
	target_link_libraries(${EXAMPLE_NAME} optimized ${LIBLINK_RELEASE})
endforeach()

foreach (LIBLINK_DEBUG ${LIBLINKS_DEBUG_LIST})
	target_link_libraries(${EXAMPLE_NAME} debug ${LIBLINK_DEBUG})
endforeach()

set_target_properties(${EXAMPLE_NAME} PROPERTIES FOLDER Examples)
set_target_properties(${EXAMPLE_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
