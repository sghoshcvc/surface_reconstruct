# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

project( surface_reconstruction_polygonal )


# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()
find_package( PCL 1.8 REQUIRED )
# include for local directory
if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
endif()

# include for local package


# Creating entries for target: surface_reconstruction_polygonal
# ############################

add_executable( surface_reconstruction_polygonal  cgal_pcl_interface.cpp surface_reconstruction_polygonal.cpp )

add_to_cached_list( CGAL_EXECUTABLE_TARGETS surface_reconstruction_polygonal )

# Link the executable to CGAL and third-party libraries
target_link_libraries(surface_reconstruction_polygonal PRIVATE CGAL::CGAL ${PCL_LIBRARIES})

