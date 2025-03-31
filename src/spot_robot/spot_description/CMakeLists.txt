cmake_minimum_required(VERSION 3.0.2)
project(spot_description)

## Enforce C++11 if needed (caso adicione código C++ futuramente)
# add_compile_options(-std=c++11)

## Encontre dependências do catkin
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  urdf
  xacro
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs urdf xacro
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

# Instala todos os arquivos do diretório urdf, meshes e rviz
install(DIRECTORY urdf meshes rviz launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Caso tenha scripts Python para visualização ou geração de arquivos
# catkin_install_python(PROGRAMS
#   scripts/exemplo.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
