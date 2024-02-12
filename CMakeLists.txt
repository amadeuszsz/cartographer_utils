cmake_minimum_required(VERSION 3.14)
project(cartographer_utils)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Pose Twist Fusion
ament_auto_add_library(pose_twist_fusion_node SHARED
  src/pose_twist_fusion_node.cpp
  include/cartographer_utils/pose_twist_fusion_node.hpp
  include/cartographer_utils/visibility_control.hpp
)

rclcpp_components_register_node(pose_twist_fusion_node
  PLUGIN "cartographer_utils::pose_twist_fusion::PoseTwistFusionNode"
  EXECUTABLE pose_twist_fusion_node_exe
)

# Pose Initializer
ament_auto_add_library(pose_initializer_node SHARED
  src/pose_initializer_node.cpp
  include/cartographer_utils/pose_initializer_node.hpp
  include/cartographer_utils/visibility_control.hpp
)

rclcpp_components_register_node(pose_initializer_node
  PLUGIN "cartographer_utils::pose_initializer::PoseInitializerNode"
  EXECUTABLE pose_initializer_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
