# USB Stack for Zephyr OS with DWC3 Synopsys M31 and Qualcomm PHY Support

This is a comprehensive USB stack implementation for Zephyr OS that provides device mode operation with serial communication capabilities. The stack is specifically designed to work with DWC3 Synopsys M31 USB controllers and Qualcomm PHY components, including support for USB Type-C connectivity.

## Architecture Overview

The USB stack is organized in multiple layers following a clean architectural design:

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│                   (Serial Communication)                   │
├─────────────────────────────────────────────────────────────┤
│                   Adaptation Layer                         │
│              (USB Class Drivers - CDC ACM)                 │
├─────────────────────────────────────────────────────────────┤
│                  Function Driver Layer                     │
│            (USB Device Core & Endpoint Management)         │
├─────────────────────────────────────────────────────────────┤
│               Device Controller Interface                   │
│                 (USB Stack Core API)                       │
├─────────────────────────────────────────────────────────────┤
│                   Hardware Layer                           │
│  ┌─────────────────┬─────────────────┬─────────────────────┐ │
│  │ DWC3 Controller │  Qualcomm PHY   │   Type-C Manager    │ │
│  │   (Synopsys)    │ (QMP/QUSB2)     │    (Port Control)   │ │
│  └─────────────────┴─────────────────┴─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Features

### USB Standards Support
- USB 2.0, 3.0, and 3.1 compliance
- Device mode operation
- High-speed, SuperSpeed, and SuperSpeed+ support
- USB Type-C connector support with orientation detection

### Hardware Support
- **DWC3 Synopsys M31 USB Controller**
  - Complete register-level driver implementation
  - Event-driven architecture with interrupt handling
  - Transfer Request Block (TRB) management
  - Endpoint configuration and management

- **Qualcomm PHY Support**
  - QUSB2 PHY for USB 2.0 operation
  - QMP PHY for USB 3.0/3.1 SuperSpeed operation
  - Automatic PHY initialization and calibration
  - Power management and clock control

- **Type-C Port Management**
  - CC line detection and orientation sensing
  - VBUS monitoring and power delivery
  - Role detection (Source/Sink/DRP)
  - Connection state machine

### USB Device Classes
- **CDC ACM (Communication Device Class)**
  - Virtual serial port functionality
  - Line coding management
  - Control line state handling
  - Serial state notifications

### Key Capabilities
- Multi-threaded event processing
- DMA-coherent buffer management
- Comprehensive error handling and recovery
- Statistics and debugging support
- Modular and extensible design

## Directory Structure

```
usb_stack/
├── include/                    # Public API headers
│   ├── usb_stack.h            # Main API header
│   ├── usb_stack_types.h      # Type definitions
│   └── usb_stack_config.h     # Configuration constants
├── src/                       # Core implementation
│   └── usb_stack_core.c       # Main USB stack logic
├── hardware/                  # Hardware abstraction layer
│   └── include/
│       └── dwc3_regs.h        # DWC3 register definitions
├── controller/                # USB controller drivers
│   └── dwc3_controller.c      # DWC3 driver implementation
├── phy/                       # PHY drivers
│   └── qcom_phy.c            # Qualcomm PHY driver
├── typec/                     # Type-C management
│   └── typec_manager.c        # Type-C port controller
├── application/               # Application layer
│   └── usb_serial.c          # USB serial communication
└── CMakeLists.txt            # Build configuration
```

## Usage Example

### Basic USB Serial Communication

```c
#include "usb_stack/include/usb_stack.h"
#include "usb_stack/application/usb_serial.h"

/* Callback for received data */
void on_data_received(const uint8_t *data, size_t len, void *user_data)
{
    printk("Received %d bytes: %.*s\n", len, len, data);
    
    /* Echo back the data */
    usb_serial_simple_write(data, len);
}

/* Callback for line state changes */
void on_line_state_changed(bool dtr, bool rts, void *user_data)
{
    printk("Line state: DTR=%d, RTS=%d\n", dtr, rts);
}

int main(void)
{
    int ret;
    
    /* Initialize USB serial */
    ret = usb_serial_simple_init();
    if (ret) {
        printk("Failed to initialize USB serial: %d\n", ret);
        return ret;
    }
    
    /* Register callbacks */
    usb_serial_simple_register_rx_callback(on_data_received, NULL);
    
    /* Enable USB */
    ret = usb_serial_simple_enable();
    if (ret) {
        printk("Failed to enable USB serial: %d\n", ret);
        return ret;
    }
    
    printk("USB Serial device ready\n");
    
    /* Main application loop */
    while (1) {
        /* Send periodic data */
        const char *msg = "Hello from Zephyr USB!\n";
        usb_serial_simple_write((const uint8_t *)msg, strlen(msg));
        
        k_sleep(K_SECONDS(5));
    }
    
    return 0;
}
```

### Advanced Usage with Custom Device

```c
#include "usb_stack/include/usb_stack.h"

static struct usb_stack_device my_usb_device;

/* Custom event handler */
void my_event_handler(struct usb_stack_device *dev, 
                     usb_stack_event_type_t event, void *data)
{
    switch (event) {
    case USB_STACK_EVENT_CONNECT:
        printk("USB device connected\n");
        break;
    case USB_STACK_EVENT_DISCONNECT:
        printk("USB device disconnected\n");
        break;
    case USB_STACK_EVENT_RESET:
        printk("USB device reset\n");
        break;
    default:
        break;
    }
}

/* Custom setup handler */
int my_setup_handler(struct usb_stack_device *dev, 
                    struct usb_setup_packet *setup)
{
    /* Handle vendor-specific requests */
    if (setup->RequestType.type == USB_REQTYPE_TYPE_VENDOR) {
        /* Process vendor request */
        return usb_stack_control_response(dev, NULL, 0);
    }
    
    return -EINVAL;  /* Not handled */
}

int advanced_usb_init(void)
{
    /* Device configuration */
    struct usb_stack_device_config config = {
        .device_desc = &my_device_descriptor,
        .config_desc = &my_config_descriptor,
        .string_descs = my_string_descriptors,
        .num_strings = ARRAY_SIZE(my_string_descriptors),
        .event_callback = my_event_handler,
        .setup_callback = my_setup_handler,
        .self_powered = true,
        .remote_wakeup = false,
        .max_power = 100,  /* 200mA */
    };
    
    /* Initialize USB stack */
    int ret = usb_stack_init(&my_usb_device, &config);
    if (ret) {
        return ret;
    }
    
    /* Enable USB */
    return usb_stack_enable(&my_usb_device);
}
```

## Configuration

### Kconfig Options

Add these options to your `prj.conf`:

```
# Enable USB stack
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_DWC3_CONTROLLER=y
CONFIG_USB_QCOM_PHY=y
CONFIG_USB_TYPEC_SUPPORT=y

# USB stack configuration
CONFIG_USB_STACK_LOG_LEVEL=3
CONFIG_USB_STACK_INIT_PRIORITY=80

# Memory configuration
CONFIG_HEAP_MEM_POOL_SIZE=16384
CONFIG_MAIN_STACK_SIZE=2048

# Logging
CONFIG_LOG=y
CONFIG_USB_STACK_LOG_LEVEL=3
```

### Device Tree Configuration

Example device tree overlay:

```dts
/ {
    usb_controller: usb@a6f8800 {
        compatible = "snps,dwc3";
        reg = <0xa6f8800 0x400>;
        interrupts = <GIC_SPI 131 IRQ_TYPE_LEVEL_HIGH>;
        clocks = <&gcc GCC_USB30_PRIM_MASTER_CLK>;
        clock-names = "core";
        resets = <&gcc GCC_USB30_PRIM_BCR>;
        reset-names = "core";
        status = "okay";
    };
    
    usb_phy: phy@a784000 {
        compatible = "qcom,usb-phy";
        reg = <0xa784000 0x1000>, <0xa800000 0x1000>;
        reg-names = "qusb2", "qmp";
        clocks = <&gcc GCC_USB_PHY_CFG_AHB2PHY_CLK>;
        clock-names = "cfg_ahb";
        resets = <&gcc GCC_QUSB2PHY_PRIM_BCR>;
        reset-names = "phy";
        phy-type = <USB_STACK_PHY_TYPE_QUSB2>;
        status = "okay";
    };
    
    typec_manager: typec@c440000 {
        compatible = "qcom,typec-manager";
        reg = <0xc440000 0x1000>;
        cc1-gpios = <&tlmm 38 GPIO_ACTIVE_HIGH>;
        cc2-gpios = <&tlmm 39 GPIO_ACTIVE_HIGH>;
        vbus-gpios = <&tlmm 40 GPIO_ACTIVE_HIGH>;
        orientation-gpios = <&tlmm 41 GPIO_ACTIVE_HIGH>;
        pd-support;
        status = "okay";
    };
};
```

## Building

1. Add the USB stack to your Zephyr project:
```bash
cd your_zephyr_project
cp -r /path/to/usb_stack .
```

2. Update your `CMakeLists.txt`:
```cmake
add_subdirectory(usb_stack)
target_link_libraries(app PRIVATE usb_stack)
```

3. Build your project:
```bash
west build -b your_board
```

## API Reference

### Core Functions

- `usb_stack_init()` - Initialize the USB stack
- `usb_stack_enable()` - Enable USB device operation
- `usb_stack_disable()` - Disable USB device
- `usb_stack_configure_endpoint()` - Configure an endpoint
- `usb_stack_submit_transfer()` - Submit a data transfer
- `usb_stack_get_state()` - Get current device state

### Serial Communication Functions

- `usb_serial_init()` - Initialize USB serial device
- `usb_serial_enable()` - Enable serial communication
- `usb_serial_write()` - Send data over USB serial
- `usb_serial_register_callbacks()` - Register event callbacks

### Utility Functions

- `usb_stack_get_speed()` - Get current USB speed
- `usb_stack_get_statistics()` - Get transfer statistics
- `usb_stack_get_typec_state()` - Get Type-C connection state

## Debugging

### Log Levels

Set `CONFIG_USB_STACK_LOG_LEVEL` to control logging:
- 0: No logging
- 1: Errors only
- 2: Warnings and errors
- 3: Info, warnings, and errors
- 4: Debug, info, warnings, and errors

### Statistics

Use `usb_stack_get_statistics()` to monitor:
- Reset count
- Suspend/resume events
- Setup packet count
- Transfer statistics
- Error counts

## Troubleshooting

### Common Issues

1. **Device not enumerated**
   - Check PHY initialization
   - Verify Type-C connection
   - Check device descriptors

2. **Transfer failures**
   - Verify endpoint configuration
   - Check buffer alignment
   - Monitor DMA coherency

3. **Type-C issues**
   - Check CC line connections
   - Verify VBUS detection
   - Monitor orientation detection

### Debug Commands

Enable debug logging and monitor:
```
uart:~$ log enable usb_stack 4
uart:~$ usb status
uart:~$ usb stats
```

## Contributing

This USB stack is designed to be modular and extensible. To add new features:

1. Follow the existing architectural patterns
2. Add appropriate error handling
3. Include comprehensive logging
4. Update documentation
5. Add test cases

## License

SPDX-License-Identifier: Apache-2.0

Copyright (c) 2025 Zephyr USB Stack Contributors
