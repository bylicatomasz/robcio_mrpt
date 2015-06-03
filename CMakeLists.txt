INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  pf-localization
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(RobcioApp)

#MESSAGE(STATUS "Makefile for application: /apps/pf-localization ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(RobcioApp 
RobcioData.cpp RobcioData.h
RobcioLocalizationPF.cpp RobcioLocalizationPF.h
RobcioMapTools.cpp RobcioMapTools.h
RobcioTools.cpp RobcioTools.h
RobcioWebService.cpp RobcioWebService.h
RobcioWinPlot.cpp RobcioWinPlot.h
main.cpp
pf_localization_main.cpp)



INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

SET(TMP_TARGET_NAME "RobcioApp")

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrptwxthings)
	ADD_DEPENDENCIES(${TMP_TARGET_NAME} mrptwxthings)


# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-slam mrpt-gui mrpt-topography)

DeclareAppForInstall(${TMP_TARGET_NAME})
