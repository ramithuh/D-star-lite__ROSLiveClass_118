# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "base_local_planner;costmap_2d;geometry_msgs;nav_core;nav_msgs;pluginlib;roscpp;rospy;tf2;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsrv_client_plugin".split(';') if "-lsrv_client_plugin" != "" else []
PROJECT_NAME = "srv_client_plugin"
PROJECT_SPACE_DIR = "/home/user/simulation_ws/install"
PROJECT_VERSION = "0.1.0"
