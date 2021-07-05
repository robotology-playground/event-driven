#choose codec type
list(APPEND CodecTypes "CODEC_128x128" "CODEC_304x240_20" "CODEC_304x240_24")
list(GET CodecTypes 2 VLIB_CODEC_DEFAULT)
set(VLIB_CODEC_TYPE ${VLIB_CODEC_DEFAULT} CACHE STRING "select codec type")
set_property(CACHE VLIB_CODEC_TYPE PROPERTY STRINGS ${CodecTypes})

#choose clock rate of FPGA
set(VLIB_CLOCK_PERIOD_NS 80 CACHE STRING "event timestamp clock period (ns)")

#choose number of bits of the FPGA timer to use
set(VLIB_TIMER_BITS 30 CACHE STRING "event timestamp maximum = 2^TIMERBITS")
string(COMPARE GREATER ${VLIB_TIMER_BITS} 31 TOOMANYBITSINCOUNTER)
if(TOOMANYBITSINCOUNTER)
  message( FATAL_ERROR "Maximum bits useable is 31 (VLIB_TIMER_BITS)")
endif()

set(VLIB_DEPRECATED OFF CACHE BOOL "Also build old classes")

set(folder_source
  src/vCodec.cpp
  src/vPort.cpp
  src/vtsHelper.cpp
  src/codecs/codec_AddressEvent.cpp
  src/codecs/codec_FlowEvent.cpp
  src/codecs/codec_GaussianAE.cpp
  src/codecs/codec_IMUEvent.cpp
  src/codecs/codec_LabelledAE.cpp
  src/codecs/codec_SkinEvent.cpp
  src/codecs/codec_SkinSample.cpp
  src/codecs/codec_CochleaEvent.cpp
  src/codecs/codec_vEvent.cpp
  src/codecs/codec_NeuronEvent.cpp)

if(OpenCV_FOUND)
    list(APPEND folder_source src/vIPT.cpp
                              src/vDraw_basic.cpp
                              src/vDraw_ISO.cpp
                              src/vDraw_skin.cpp
                              src/imuPort.cpp)
endif()

if(VLIB_DEPRECATED)
  list(APPEND folder_source
    src/vWindow_adv.cpp
    src/vWindow_basic.cpp)
endif()

set(folder_header
  include/event-driven/vtsHelper.h
  include/event-driven/vCodec.h
  include/event-driven/vFilters.h
  include/event-driven/vPort.h
  include/event-driven/vCollectSend.h
  include/event-driven/all.h
  include/event-driven/imuPort.h
)

if(OpenCV_FOUND)
    list(APPEND folder_header include/event-driven/vIPT.h
                              include/event-driven/vDraw.h
                              include/event-driven/vDrawSkin.h)
endif()

if(VLIB_DEPRECATED)
  list(APPEND folder_header
    include/event-driven/vBottle.h
    include/event-driven/vWindow_adv.h
    include/event-driven/vWindow_basic.h
    include/event-driven/vSurfaceHandlerTh.h
    include/event-driven/deprecated.h)
endif()

# Create everything needed to build our library
add_library(${EVENTDRIVEN_LIBRARY} ${folder_source} ${folder_header})
add_library(ev::${EVENTDRIVEN_LIBRARY} ALIAS ${EVENTDRIVEN_LIBRARY})


set_target_properties(${EVENTDRIVEN_LIBRARY} PROPERTIES PUBLIC_HEADER "${folder_header}"
                                                          VERSION ${event-driven_VERSION}
                                                          SOVERSION 1)
target_include_directories(${EVENTDRIVEN_LIBRARY} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>") # FIXME INSTALL PATH

target_compile_definitions(${EVENTDRIVEN_LIBRARY} PRIVATE "${VLIB_CODEC_TYPE}"
                                                            CLOCK_PERIOD=${VLIB_CLOCK_PERIOD_NS}
                                                            TIMER_BITS=${VLIB_TIMER_BITS})

target_compile_options(${EVENTDRIVEN_LIBRARY} PRIVATE -Wall)

if(OpenCV_FOUND)
    target_link_libraries(${EVENTDRIVEN_LIBRARY} PUBLIC YARP::YARP_OS
                                                        YARP::YARP_sig
                                                        ${OpenCV_LIBRARIES})
else()
    target_link_libraries(${EVENTDRIVEN_LIBRARY} PUBLIC YARP::YARP_OS
                                                        YARP::YARP_sig)
endif()

install(TARGETS ${EVENTDRIVEN_LIBRARY}
        EXPORT eventdriven
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${EVENTDRIVEN_LIBRARY}" COMPONENT dev)

