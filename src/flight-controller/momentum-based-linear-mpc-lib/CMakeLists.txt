
set(COMPILE_MPC_library ON CACHE BOOL "Compile the MPC library")

if(COMPILE_MPC_library)
	set(LIBRARY_TARGET_NAME linearMomentumBasedMPC)

	set(H_PREFIX include)

	set(MPC_LIBRARY_HDR include/IMPCProblem/IQPUtilsMPC.h
						include/IMPCProblem/IMPCProblem.h
						include/IMPCProblem/systemDynamic.h
						include/variableSamplingMPC/variableSamplingMPC.h
						include/variableSamplingMPC/costsVSMPC.h
						include/variableSamplingMPC/constraintsVSMPC.h
						include/variableSamplingMPC/systemDynamicsVSMPC.h)
	
	set(MPC_LIBRARY_SRC src/IMPCProblem/IQPUtilsMPC.cpp
						src/IMPCProblem/IMPCProblem.cpp
						src/IMPCProblem/systemDynamic.cpp
						src/variableSamplingMPC/variableSamplingMPC.cpp
						src/variableSamplingMPC/costsVSMPC.cpp
						src/variableSamplingMPC/constraintsVSMPC.cpp
						src/variableSamplingMPC/systemDynamicsVSMPC.cpp)
	
	add_library(${LIBRARY_TARGET_NAME} ${MPC_LIBRARY_HDR} ${MPC_LIBRARY_SRC})

	set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES PUBLIC_HEADER "${MPC_LIBRARY_HDR}")

	target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
															"$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

	target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC FlightControlLibrary)

	# Specify installation targets, typology and destination folders.
	install(TARGETS ${LIBRARY_TARGET_NAME}
	EXPORT  ${PROJECT_NAME}
	LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
	ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
	RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
	PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)

endif()

add_subdirectory(bindings)
