cmake_minimum_required(VERSION 3.0.2)
project(spot_gazebo)

## Enforce C++11 if needed (for future code)
# add_compile_options(-std=c++11)

## Encontre as dependências do catkin
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  spot_description
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS gazebo_ros roscpp rospy sensor_msgs std_msgs spot_description
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Exemplo para instalar arquivos de configuração e launch
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Caso use scripts Python futuramente
# catkin_install_python(PROGRAMS
#   scripts/controlador_exemplo.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

# Tests opcionais podem ser adicionados aqui no futuro
