set(tusb_src "${CMAKE_CURRENT_SOURCE_DIR}/tinyusb-0.18.0/src")
idf_component_register(SRCS "usb_audio.c" "usb_descriptors.c" "usb_pcm.c" "usb_feedback.c" "usb_recovery.c" "usb_controls.c"
    "${tusb_src}/tusb.c" "${tusb_src}/common/tusb_fifo.c"
    "${tusb_src}/device/usbd.c" "${tusb_src}/device/usbd_control.c"
    "${tusb_src}/class/audio/audio_device.c" "${tusb_src}/class/cdc/cdc_device.c"
    "${tusb_src}/class/hid/hid_device.c"
    "${tusb_src}/portable/synopsys/dwc2/dcd_dwc2.c" "${tusb_src}/portable/synopsys/dwc2/dwc2_common.c"
    INCLUDE_DIRS "." "${tusb_src}" REQUIRES usb vfs esp_timer esp_driver_gpio esp_system esp_hw_support)
target_compile_definitions(${COMPONENT_LIB} PUBLIC CFG_TUSB_MCU=OPT_MCU_ESP32S3 CFG_TUSB_OS=OPT_OS_FREERTOS)
