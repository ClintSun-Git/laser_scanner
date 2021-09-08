# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sensor_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lscanner_driver".split(';') if "-lscanner_driver" != "" else []
PROJECT_NAME = "bea_scanner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"
