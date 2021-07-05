#################################################################################
#                                                                               #
# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)           #
# All Rights Reserved.                                                          #
#                                                                               #
#################################################################################

# @authors: Leandro de Souza Rosa <leandro.desouzarosa@iit.it>

project(IMUDataDumper)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}
    ${source}
    ${header}
)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_math
                                              ev::event-driven
                                              stdc++fs)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
