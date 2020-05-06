cmake_minimum_required(VERSION 3.0)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

set(DISCREGRID_VERSION_MAJOR "0")
set(DISCREGRID_VERSION_MINOR "1")
set(DISCREGRID_VERSION_PATCH "1")
set(DISCREGRID_VERSION_EXTRA "")
set(DISCREGRID_VERSION "${DISCREGRID_VERSION_MAJOR}.${DISCREGRID_VERSION_MINOR}")
set(DISCREGRID_VERSION_FULL "${DISCREGRID_VERSION}.${DISCREGRID_VERSION_PATCH}${DISCREGRID_VERSION_EXTRA}")
set(DISCREGRID_LIB_NAME Discregrid)

project(Discregrid
  VERSION ${DISCREGRID_VERSION}
  LANGUAGES CXX
)

# Visual studio solution directories.
set_property(GLOBAL PROPERTY USE_FOLDERS on)

# Require C++11 compiler
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Enable simultaneous compilation of source files.
if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250")
endif(MSVC)

OPTION(BUILD_AS_SHARED_LIBS "Build all the libraries as shared" OFF)
if (BUILD_AS_SHARED_LIBS)
	set(BUILD_AS_SHARED_LIBS ON)
	if(WIN32)
		set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON CACHE BOOL "Export all symbols")
	endif(WIN32)	
endif (BUILD_AS_SHARED_LIBS)

if (NOT WIN32)
  # set build type as Release by default
  if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
    if (NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    endif()
  endif()
endif()
set(CMAKE_DEBUG_POSTFIX "_d")

# Put all executables and libraries into a common directory.
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH    "${PROJECT_BINARY_DIR}/bin")

add_subdirectory(discregrid)

option(BUILD_CMD_EXECUTABLE "Build command line executable" ON)
if(BUILD_CMD_EXECUTABLE)
	add_subdirectory(cmd)
endif(BUILD_CMD_EXECUTABLE)

include(GNUInstallDirs)
########## install cmake files
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/Discregrid)
install(EXPORT Discregrid-targets
  FILE
    Discregrid-targets.cmake
  NAMESPACE
    Discregrid::
  DESTINATION
    ${INSTALL_CONFIGDIR}
)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/DiscregridConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/DiscregridConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/DiscregridConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/DiscregridConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/DiscregridConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

install(
  TARGETS
    Discregrid
  EXPORT Discregrid-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY discregrid/include/Discregrid
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PATTERN "*.hpp")

set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Kenny Chu")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libeigen3-dev")
include(CPack)
