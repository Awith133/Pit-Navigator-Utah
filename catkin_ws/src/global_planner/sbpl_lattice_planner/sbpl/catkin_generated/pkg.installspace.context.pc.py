# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include".split(';') if "${prefix}/include;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;costmap_2d;nav_core;pluginlib;tf2;geometry_msgs;nav_msgs;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsbpl_lattice_planner;/usr/local/lib/libsbpl.so".split(';') if "-lsbpl_lattice_planner;/usr/local/lib/libsbpl.so" != "" else []
PROJECT_NAME = "sbpl_lattice_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.3.4"
