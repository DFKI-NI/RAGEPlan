project(rageplan)
cmake_minimum_required(VERSION 3.0)

set(SOURCE_FILES
src/beliefstate.cpp
src/cellar.cpp
src/coord.cpp
src/drone.cpp
src/experiment.cpp
src/ftable.cpp
src/main.cpp
src/mcts.cpp
src/mobipick.cpp
src/node.cpp
src/rocksample.cpp
src/simulator.cpp
src/utils.cpp
)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(rage ${SOURCE_FILES})
TARGET_LINK_LIBRARIES( rage LINK_PUBLIC )

#set(LIB_DESTINATION "/lib")
#set(BIN_DESTINATION "/bin")

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${dir}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${dir}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${dir}/bin")

install(TARGETS rage
  ARCHIVE DESTINATION ${dir}/lib
  LIBRARY DESTINATION ${dir}/lib
  RUNTIME DESTINATION ${dir}/bin
)
