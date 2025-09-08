# USB Stack for Zephyr OS with DWC3 Synopsys M31 and Qualcomm PHY Support
# Copyright (c) 2025

cmake_minimum_required(VERSION 3.20.0)

# Add subdirectories for different layers
add_subdirectory(hardware)
add_subdirectory(controller)
add_subdirectory(device_core)
add_subdirectory(function_driver)
add_subdirectory(adaptation)
add_subdirectory(application)
add_subdirectory(phy)
add_subdirectory(typec)

# Main USB stack library
zephyr_library()

# Include directories
zephyr_include_directories(
    include
    hardware/include
    controller/include
    device_core/include
    function_driver/include
    adaptation/include
    application/include
    phy/include
    typec/include
)

# Source files
zephyr_library_sources(
    src/usb_stack_init.c
    src/usb_stack_core.c
)

# Configuration options
zephyr_library_sources_ifdef(CONFIG_USB_DEVICE_STACK src/usb_device_stack.c)
zephyr_library_sources_ifdef(CONFIG_USB_DWC3_CONTROLLER controller/dwc3_controller.c)
zephyr_library_sources_ifdef(CONFIG_USB_QCOM_PHY phy/qcom_phy.c)
zephyr_library_sources_ifdef(CONFIG_USB_TYPEC_SUPPORT typec/typec_manager.c)
