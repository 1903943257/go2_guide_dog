# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(actor_collisions_CONFIG_INCLUDED)
  return()
endif()
set(actor_collisions_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(actor_collisions_SOURCE_PREFIX /home/sanshiqi/project/DRL-robot-navigation/catkin_ws/src/actor_collisions)
  set(actor_collisions_DEVEL_PREFIX /home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/actor_collisions)
  set(actor_collisions_INSTALL_PREFIX "")
  set(actor_collisions_PREFIX ${actor_collisions_DEVEL_PREFIX})
else()
  set(actor_collisions_SOURCE_PREFIX "")
  set(actor_collisions_DEVEL_PREFIX "")
  set(actor_collisions_INSTALL_PREFIX /home/sanshiqi/project/DRL-robot-navigation/catkin_ws/install_isolated)
  set(actor_collisions_PREFIX ${actor_collisions_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'actor_collisions' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(actor_collisions_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/usr/include;/usr/include/gazebo-11;/usr/include/bullet;/usr/include/simbody;/usr/include/sdformat-9.10;/usr/include/ignition/math6;/usr/include/OGRE;/usr/include/OGRE/Terrain;/usr/include/OGRE/Paging;/usr/include/ignition/transport8;/usr/include/ignition/msgs5;/usr/include/ignition/common3;/usr/include/ignition/fuel_tools4 " STREQUAL " ")
  set(actor_collisions_INCLUDE_DIRS "")
  set(_include_dirs "/usr/include;/usr/include/gazebo-11;/usr/include/bullet;/usr/include/simbody;/usr/include/sdformat-9.10;/usr/include/ignition/math6;/usr/include/OGRE;/usr/include/OGRE/Terrain;/usr/include/OGRE/Paging;/usr/include/ignition/transport8;/usr/include/ignition/msgs5;/usr/include/ignition/common3;/usr/include/ignition/fuel_tools4")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Your Name <your@email.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${actor_collisions_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'actor_collisions' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'actor_collisions' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/src/actor_collisions/${idir}'.  ${_report}")
    endif()
    _list_append_unique(actor_collisions_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "BulletSoftBody;BulletDynamics;BulletCollision;LinearMath;/usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6;/usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6;/usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6;/usr/lib/x86_64-linux-gnu/libdart.so.6.9.2;/usr/lib/x86_64-linux-gnu/libgazebo.so;/usr/lib/x86_64-linux-gnu/libgazebo_client.so;/usr/lib/x86_64-linux-gnu/libgazebo_gui.so;/usr/lib/x86_64-linux-gnu/libgazebo_sensors.so;/usr/lib/x86_64-linux-gnu/libgazebo_rendering.so;/usr/lib/x86_64-linux-gnu/libgazebo_physics.so;/usr/lib/x86_64-linux-gnu/libgazebo_ode.so;/usr/lib/x86_64-linux-gnu/libgazebo_transport.so;/usr/lib/x86_64-linux-gnu/libgazebo_msgs.so;/usr/lib/x86_64-linux-gnu/libgazebo_util.so;/usr/lib/x86_64-linux-gnu/libgazebo_common.so;/usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so;/usr/lib/x86_64-linux-gnu/libgazebo_opcode.so;/usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0;/usr/lib/x86_64-linux-gnu/libprotobuf.so;-lpthread;/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.10.1;optimized;/usr/lib/x86_64-linux-gnu/libOgreMain.so;debug;/usr/lib/x86_64-linux-gnu/libOgreMain.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0;optimized;/usr/lib/x86_64-linux-gnu/libOgreTerrain.so;debug;/usr/lib/x86_64-linux-gnu/libOgreTerrain.so;optimized;/usr/lib/x86_64-linux-gnu/libOgrePaging.so;debug;/usr/lib/x86_64-linux-gnu/libOgrePaging.so;/usr/lib/x86_64-linux-gnu/libignition-math6.so.6.15.1;/usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.5.0;/usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.11.0;/usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.17.0;/usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.9.1")
foreach(library ${libraries})
  # keep build configuration keywords, generator expressions, target names, and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND actor_collisions_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND actor_collisions_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT actor_collisions_NUM_DUMMY_TARGETS)
      set(actor_collisions_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::actor_collisions::wrapped-linker-option${actor_collisions_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR actor_collisions_NUM_DUMMY_TARGETS "${actor_collisions_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::actor_collisions::wrapped-linker-option${actor_collisions_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND actor_collisions_LIBRARIES "${interface_target_name}")
  elseif(${library} MATCHES "^\\$<")
    list(APPEND actor_collisions_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND actor_collisions_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND actor_collisions_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/actor_collisions/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/z1_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/velodyne_simulator/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/velodyne_gazebo_plugins/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/velodyne_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_move_base/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_motor_ctrl/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_real/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_sdk/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_control/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_guide/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_gazebo/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_controller/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_msgs/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/odom/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/multi_robot_scenario/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/laikago_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/human_plugin/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/human_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/h1_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go2w_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go2_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go1_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/b2w_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/b2_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/b1_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/aliengo_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/aliengoZ1_description/lib;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/a1_description/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(actor_collisions_LIBRARY_DIRS ${lib_path})
      list(APPEND actor_collisions_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'actor_collisions'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND actor_collisions_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(actor_collisions_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${actor_collisions_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "gazebo_ros;roscpp")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 actor_collisions_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${actor_collisions_dep}_FOUND)
      find_package(${actor_collisions_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${actor_collisions_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(actor_collisions_INCLUDE_DIRS ${${actor_collisions_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(actor_collisions_LIBRARIES ${actor_collisions_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${actor_collisions_dep}_LIBRARIES})
  _list_append_deduplicate(actor_collisions_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(actor_collisions_LIBRARIES ${actor_collisions_LIBRARIES})

  _list_append_unique(actor_collisions_LIBRARY_DIRS ${${actor_collisions_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(actor_collisions_EXPORTED_TARGETS ${${actor_collisions_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${actor_collisions_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
