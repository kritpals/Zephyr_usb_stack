# SPDX-License-Identifier: Apache-2.0

# USB Stack for Zephyr OS with DWC3 Synopsys M31 and Qualcomm PHY Support
# Copyright (c) 2025

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

# Core USB stack sources
zephyr_library_sources(
    src/usb_stack_core.c
    src/usb_stack_init.c
)
# Hardware layer
zephyr_library_sources_ifdef(CONFIG_USB_STACK_DWC3_CONTROLLER
    controller/dwc3_controller.c
)
# PHY layer
zephyr_library_sources_ifdef(CONFIG_USB_STACK_QCOM_PHY
    phy/qcom_phy.c
)
# Qualcomm PHY components
zephyr_library_sources_ifdef(CONFIG_USB_STACK_QCOM_RETIMER
    phy/qcom_retimer.c
)
zephyr_library_sources_ifdef(CONFIG_USB_STACK_QCOM_REDRIVER
    phy/qcom_redriver.c
)
zephyr_library_sources_ifdef(CONFIG_USB_STACK_QCOM_REPEATER
    phy/qcom_repeater.c
)

# Type-C support
zephyr_library_sources_ifdef(CONFIG_USB_STACK_TYPEC_SUPPORT
    typec/typec_manager.c
)
# Application layer
zephyr_library_sources_ifdef(CONFIG_USB_STACK_SERIAL
    application/usb_serial.c
)
# Device core layer
zephyr_library_sources_ifdef(CONFIG_USB_STACK_DEVICE_CORE
    device_core/usb_device_core.c
)
# Function driver layer
zephyr_library_sources_ifdef(CONFIG_USB_STACK_FUNCTION_DRIVER
    function_driver/usb_function_driver.c
)
# Adaptation layer
zephyr_library_sources_ifdef(CONFIG_USB_STACK_ADAPTATION
    adaptation/usb_adaptation.c
)
