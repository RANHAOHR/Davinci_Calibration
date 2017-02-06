# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "davinci_calibration: 3 messages, 0 services")

set(MSG_I_FLAGS "-Idavinci_calibration:/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(davinci_calibration_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg" NAME_WE)
add_custom_target(_davinci_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "davinci_calibration" "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg" ""
)

get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg" NAME_WE)
add_custom_target(_davinci_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "davinci_calibration" "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg" ""
)

get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg" NAME_WE)
add_custom_target(_davinci_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "davinci_calibration" "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg" "davinci_calibration/points"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/davinci_calibration
)
_generate_msg_cpp(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/davinci_calibration
)
_generate_msg_cpp(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/davinci_calibration
)

### Generating Services

### Generating Module File
_generate_module_cpp(davinci_calibration
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/davinci_calibration
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(davinci_calibration_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(davinci_calibration_generate_messages davinci_calibration_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_cpp _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_cpp _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_cpp _davinci_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(davinci_calibration_gencpp)
add_dependencies(davinci_calibration_gencpp davinci_calibration_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS davinci_calibration_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/davinci_calibration
)
_generate_msg_eus(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/davinci_calibration
)
_generate_msg_eus(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/davinci_calibration
)

### Generating Services

### Generating Module File
_generate_module_eus(davinci_calibration
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/davinci_calibration
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(davinci_calibration_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(davinci_calibration_generate_messages davinci_calibration_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_eus _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_eus _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_eus _davinci_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(davinci_calibration_geneus)
add_dependencies(davinci_calibration_geneus davinci_calibration_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS davinci_calibration_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/davinci_calibration
)
_generate_msg_lisp(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/davinci_calibration
)
_generate_msg_lisp(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/davinci_calibration
)

### Generating Services

### Generating Module File
_generate_module_lisp(davinci_calibration
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/davinci_calibration
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(davinci_calibration_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(davinci_calibration_generate_messages davinci_calibration_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_lisp _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_lisp _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_lisp _davinci_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(davinci_calibration_genlisp)
add_dependencies(davinci_calibration_genlisp davinci_calibration_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS davinci_calibration_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration
)
_generate_msg_py(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration
)
_generate_msg_py(davinci_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration
)

### Generating Services

### Generating Module File
_generate_module_py(davinci_calibration
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(davinci_calibration_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(davinci_calibration_generate_messages davinci_calibration_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/points.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_py _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_py _davinci_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/davinci_calibration/msg/corners.msg" NAME_WE)
add_dependencies(davinci_calibration_generate_messages_py _davinci_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(davinci_calibration_genpy)
add_dependencies(davinci_calibration_genpy davinci_calibration_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS davinci_calibration_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/davinci_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/davinci_calibration
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(davinci_calibration_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/davinci_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/davinci_calibration
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(davinci_calibration_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/davinci_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/davinci_calibration
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(davinci_calibration_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/davinci_calibration
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(davinci_calibration_generate_messages_py std_msgs_generate_messages_py)
