
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#ADD_EXECUTABLE(pmvs2 pmvs2.cc)
#
#TARGET_LINK_LIBRARIES(pmvs2 pmvs_lib image_lib numeric_lib ${PMVS_LIBRARIES} nlopt tinycthread ${OpenCV_LIBS})
#
#ADD_EXECUTABLE(genOption genOption.cc)
#
#ADD_EXECUTABLE(cmvs cmvs.cc)
#TARGET_LINK_LIBRARIES(cmvs cmvs_lib image_lib graclus_lib ${PMVS_LIBRARIES} tinycthread)

#add_library(dense_lib
#        pdense.cpp
#        )

add_executable(dense_node
        dense_node.cpp
        pdense.cpp
        )

target_link_libraries(dense_node
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        libGeographiccc
        ${PCL_LIBRARIES}
        ${YAML_LIBRARIES}
#        nlopt
        pmvs_lib image_lib numeric_lib ${PMVS_LIBRARIES} tinycthread ${OpenCV_LIBS} ${NLOPT_LIBRARIES}
#        dense_lib
        )

#INSTALL(TARGETS genOption DESTINATION bin/)
#INSTALL(TARGETS cmvs DESTINATION bin/)
#INSTALL(TARGETS pmvs2 DESTINATION bin/)
