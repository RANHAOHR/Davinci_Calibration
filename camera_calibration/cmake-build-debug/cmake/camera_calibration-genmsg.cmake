# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "camera_calibration: 3 messages, 0 services")

set(MSG_I_FLAGS "-Icamera_calibration:/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(camera_calibration_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg" NAME_WE)
add_custom_target(_camera_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "camera_calibration" "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg" "camera_calibration/points"
)

get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg" NAME_WE)
add_custom_target(_camera_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "camera_calibration" "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg" ""
)

get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg" NAME_WE)
add_custom_target(_camera_calibration_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "camera_calibration" "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/camera_calibration
)
_generate_msg_cpp(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/camera_calibration
)
_generate_msg_cpp(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_cpp(camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/camera_calibration
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(camera_calibration_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(camera_calibration_generate_messages camera_calibration_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_cpp _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_cpp _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_cpp _camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(camera_calibration_gencpp)
add_dependencies(camera_calibration_gencpp camera_calibration_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS camera_calibration_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/camera_calibration
)
_generate_msg_eus(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/camera_calibration
)
_generate_msg_eus(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_eus(camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/camera_calibration
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(camera_calibration_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(camera_calibration_generate_messages camera_calibration_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_eus _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_eus _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_eus _camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(camera_calibration_geneus)
add_dependencies(camera_calibration_geneus camera_calibration_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS camera_calibration_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/camera_calibration
)
_generate_msg_lisp(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/camera_calibration
)
_generate_msg_lisp(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_lisp(camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/camera_calibration
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(camera_calibration_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(camera_calibration_generate_messages camera_calibration_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_lisp _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_lisp _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_lisp _camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(camera_calibration_genlisp)
add_dependencies(camera_calibration_genlisp camera_calibration_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS camera_calibration_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg"
  "${MSG_I_FLAGS}"
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration
)
_generate_msg_py(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration
)
_generate_msg_py(camera_calibration
  "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration
)

### Generating Services

### Generating Module File
_generate_module_py(camera_calibration
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(camera_calibration_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(camera_calibration_generate_messages camera_calibration_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/corners.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_py _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/intrinsic_param.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_py _camera_calibration_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/msg/points.msg" NAME_WE)
add_dependencies(camera_calibration_generate_messages_py _camera_calibration_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(camera_calibration_genpy)
add_dependencies(camera_calibration_genpy camera_calibration_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS camera_calibration_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/camera_calibration
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(camera_calibration_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/camera_calibration
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(camera_calibration_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/camera_calibration)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/camera_calibration
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(camera_calibration_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/camera_calibration/.+/__init__.pyc?$"
  )
endif()
add_dependencies(camera_calibration_generate_messages_py std_msgs_generate_messages_py)
