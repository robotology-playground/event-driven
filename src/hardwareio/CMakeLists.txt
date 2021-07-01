option(ENABLE_zynqgrabber "Build zynqGrabber" OFF)
option(ENABLE_chronocamgrabber "Build chronocamGrabber" OFF)
option(ENABLE_binarydumper "Build binaryDumper" OFF)
option(ENABLE_qadIMUcal "Build qadIMUcalibrator" OFF)
option(ENABLE_atis3 "Build ATIS generation 3 bridge" OFF)
option(ENABLE_IMUCalibDuper "Build IMU dumper for calibration" OFF)

if(ENABLE_zynqgrabber)
    add_subdirectory(zynqGrabber)
endif()

if(ENABLE_chronocamgrabber)
    add_subdirectory(chronocamGrabber)
endif()

if(ENABLE_binarydumper)
    add_subdirectory(binaryDumper)
endif()

if(ENABLE_qadIMUcal)
    add_subdirectory(qadIMUcal)
endif()

if(ENABLE_IMUCalibDumper)
    add_subdirectory(IMUCalibDumper)
endif()


if(ENABLE_atis3 AND (MetavisionSDK_FOUND OR prophesee_core_FOUND))
    add_subdirectory(atis3-bridge)
elseif(ENABLE_atis3)
    message(FATAL_ERROR "prophesee_core/MetavisionSDK not found. Either install the SDK or turn off ENABLE_atis3 option")
endif()
