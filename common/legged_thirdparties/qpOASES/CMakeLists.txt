cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(qpoases)

find_package(ament_cmake REQUIRED)

###########
## Build ##
###########

include_directories(include)

#
# suppress any printline output from qpOASES
#
add_definitions(-D__SUPPRESSANYOUTPUT__)

#
# building qpOASES LIBRARY
#
set(SRCS
  src/BLASReplacement.cpp
  src/Bounds.cpp
  src/Constraints.cpp
  src/Flipper.cpp
  src/Indexlist.cpp
  src/LAPACKReplacement.cpp
  src/Matrices.cpp
  src/MessageHandling.cpp
  src/Options.cpp
  src/OQPinterface.cpp 
  src/QProblem.cpp
  src/QProblemB.cpp
  src/SolutionAnalysis.cpp
  src/SparseSolver.cpp
  src/SubjectTo.cpp
  src/SQProblem.cpp
  src/SQProblemSchur.cpp
  src/Utils.cpp)
add_library(${PROJECT_NAME} ${SRCS})

# set(qpOASES_PYTHON_SRC
#   interfaces/python/qpoases.pxd
#   interfaces/python/qpoases.pyx
# )

#
# building example applications
#
# set(EXAMPLES
#   example1
#   example1a
#   example1b
#   example2
#   example3
#   example3b
#   example4
#   example5
#   exampleLP
#   qrecipe
#   qrecipeSchur)
 
# foreach(EXAMPLE ${EXAMPLES})
#   add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
#   target_link_libraries(${EXAMPLE} ${PROJECT_NAME})
# endforeach(EXAMPLE)

################
## Installing ##
################

install(
  TARGETS ${PROJECT_NAME} # ${EXAMPLES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}  
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.ipp"
  PATTERN ".svn" EXCLUDE)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()