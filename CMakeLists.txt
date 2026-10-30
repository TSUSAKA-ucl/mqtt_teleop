cmake_minimum_required(VERSION 3.8)
project(mqtt_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PahoMqttCpp REQUIRED)   # apt: libpaho-mqttpp-dev libpaho-mqtt-dev
# for venv_util.py
find_package(ament_cmake_python REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MetaworkControl.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(teleop_node src/teleop_node.cpp src/MQTT_Client.cpp)
target_include_directories(teleop_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(teleop_node rclcpp sensor_msgs)
target_link_libraries(teleop_node
	"${cpp_typesupport_target}"
	nlohmann_json::nlohmann_json
	PahoMqttCpp::paho-mqttpp3)

install(TARGETS teleop_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_python_install_package(venv_utils)
install(PROGRAMS
	scripts/piper_v2_cmd.py scripts/v2_both.sh
	scripts/piper_metawork.sh
	DESTINATION lib/${PROJECT_NAME})

ament_package()
