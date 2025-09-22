# USB Stack Sequence Diagrams and Initialization Analysis

This document contains comprehensive sequence diagrams and analysis for the USB stack implementation based on the current code structure.

## 1. USB Stack Initialization Sequence

```mermaid
sequenceDiagram
    participant System as Zephyr System
    participant Init as usb_stack_init.c
    participant Core as usb_stack_core.c
    participant PHY as qcom_phy.c
    participant DWC3 as dwc3_controller.c
    participant TypeC as typec_manager.c
    participant Serial as usb_serial.c

    Note over System: System Boot Process
    System->>System: Zephyr kernel initialization
    
    Note over System: Device Driver Initialization (DEVICE_DT_INST_DEFINE)
    System->>PHY: qcom_phy_init() [AUTO]
    PHY-->>System: PHY driver ready
    
    System->>DWC3: dwc3_controller_init() [AUTO]
    DWC3-->>System: DWC3 driver ready
    
    System->>TypeC: typec_manager_init() [AUTO]
    TypeC-->>System: Type-C driver ready
    
    Note over System: SYS_INIT Hook Execution
    System->>Init: usb_stack_system_init() [SYS_INIT]
    
    Init->>Init: LOG_INF("Starting USB stack initialization")
    Init->>Init: Initialize g_usb_stack_device structure
    Init->>Init: Set default configuration
    
    Note over Init: Hardware Component Discovery
    Init->>Init: usb_stack_phy_init()
    Init->>Init: DEVICE_DT_GET_ANY(qcom_usb_phy)
    Init-->>Init: PHY device found
    
    Init->>Init: usb_stack_hw_init()
    Init->>Init: DEVICE_DT_GET_ANY(synopsys_dwc3)
    Init-->>Init: DWC3 device found
    
    Init->>Init: usb_stack_typec_init()
    Init->>Init: DEVICE_DT_GET_ANY(typec_manager)
    Init-->>Init: Type-C device found
    
    Note over Init: USB Stack Core Initialization
    Init->>Core: usb_stack_init(&g_usb_stack_device, config)
    Core->>Core: Initialize device structure
    Core->>Core: Initialize synchronization objects
    Core->>Core: Initialize event handling
    Core->>Core: Initialize endpoints
    Core->>Core: Start work queue
    Core-->>Init: USB_STACK_SUCCESS
    
    Init->>Init: g_usb_stack_initialized = true
    Init-->>System: Initialization complete
    
    Note over System: Application Layer Initialization
    System->>Serial: usb_serial_init() [Application call]
    Serial->>Serial: Initialize USB serial device
    Serial->>Serial: Set default line coding
    Serial->>Serial: Allocate buffers
    Serial->>Serial: Initialize string descriptors
    Serial->>Core: usb_stack_init(&serial->usb_dev, &usb_config)
    Core-->>Serial: Success
    Serial-->>System: USB Serial ready
```

## 2. USB Stack Enable/Runtime Sequence

```mermaid
sequenceDiagram
    participant App as Application
    participant Serial as usb_serial.c
    participant Core as usb_stack_core.c
    participant PHY as qcom_phy.c
    participant DWC3 as dwc3_controller.c
    participant TypeC as typec_manager.c
    participant Host as USB Host

    Note over App: Application Starts USB Communication
    App->>Serial: usb_serial_enable()
    Serial->>Core: usb_stack_enable(&serial->usb_dev)
    
    Note over Core: Enable Hardware Components
    Core->>PHY: phy_api->init(phy_dev)
    PHY->>PHY: Initialize QUSB2 and QMP PHYs
    PHY->>PHY: Configure PHY registers
    PHY-->>Core: Success
    
    Core->>PHY: phy_api->power_on(phy_dev)
    PHY->>PHY: Power on PHYs
    PHY->>PHY: Enable clocks and regulators
    PHY-->>Core: Success
    
    Core->>DWC3: ctrl_api->init(controller_dev)
    DWC3->>DWC3: Core soft reset
    DWC3->>DWC3: Configure global control
    DWC3->>DWC3: Initialize device mode
    DWC3->>DWC3: Setup event buffers
    DWC3-->>Core: Success
    
    Core->>DWC3: ctrl_api->enable(controller_dev)
    DWC3->>DWC3: Start controller (RUN_STOP bit)
    DWC3-->>Core: Success
    
    Core->>TypeC: typec_api->enable(typec_dev)
    TypeC->>TypeC: Enable Type-C port
    TypeC->>TypeC: Configure CC detection
    TypeC-->>Core: Success
    
    Core->>Core: dev->state = USB_STACK_STATE_POWERED
    Core-->>Serial: USB_STACK_SUCCESS
    
    Note over Serial: Configure USB Serial
    Serial->>Serial: usb_serial_configure()
    Serial->>Core: usb_stack_configure_endpoint(OUT_EP)
    Serial->>Core: usb_stack_configure_endpoint(IN_EP)
    Serial->>Core: usb_stack_configure_endpoint(NOTIFY_EP)
    Serial->>Core: usb_stack_submit_transfer(RX_transfer)
    Serial-->>App: USB Serial enabled
    
    Note over Host: USB Host Connection
    Host->>DWC3: USB cable connected
    DWC3->>DWC3: Generate connect event
    DWC3->>Core: usb_stack_submit_event(USB_STACK_EVENT_CONNECT)
    Core->>Core: Process event in work queue
    Core->>Core: dev->state = USB_STACK_STATE_ATTACHED
    Core->>Serial: event_callback(USB_STACK_EVENT_CONNECT)
    
    Host->>DWC3: USB reset
    DWC3->>Core: usb_stack_submit_event(USB_STACK_EVENT_RESET)
    Core->>Core: dev->state = USB_STACK_STATE_DEFAULT
    Core->>Serial: event_callback(USB_STACK_EVENT_RESET)
```

## 3. USB Serial Communication Sequence

```mermaid
sequenceDiagram
    participant Host as USB Host
    participant DWC3 as dwc3_controller.c
    participant Core as usb_stack_core.c
    participant Adapt as usb_adaptation.c
    participant Serial as usb_serial.c
    participant App as Application

    Note over Host: USB Enumeration Process
    Host->>DWC3: SETUP: GET_DESCRIPTOR (Device)
    DWC3->>Core: usb_stack_process_setup()
    Core->>Core: handle_get_descriptor()
    Core->>Serial: Return device descriptor
    Core->>DWC3: Control IN transfer
    DWC3-->>Host: Device descriptor

    Host->>DWC3: SETUP: SET_ADDRESS (7)
    DWC3->>Core: usb_stack_process_setup()
    Core->>Core: handle_set_address()
    Core->>Core: dev->address = 7
    Core->>DWC3: ctrl_api->set_address(7)
    Core->>DWC3: Control status
    DWC3-->>Host: ACK

    Host->>DWC3: SETUP: GET_DESCRIPTOR (Configuration)
    DWC3->>Core: usb_stack_process_setup()
    Core->>Core: handle_get_descriptor()
    Core->>Serial: Return config descriptor
    Core->>DWC3: Control IN transfer
    DWC3-->>Host: Configuration descriptor

    Host->>DWC3: SETUP: SET_CONFIGURATION (1)
    DWC3->>Core: usb_stack_process_setup()
    Core->>Core: handle_set_configuration()
    Core->>Core: dev->state = USB_STACK_STATE_CONFIGURED
    Core->>DWC3: Configure bulk endpoints
    Core->>DWC3: Control status
    DWC3-->>Host: ACK

    Note over Host: CDC Class Requests
    Host->>DWC3: SETUP: SET_LINE_CODING
    DWC3->>Core: usb_stack_process_setup()
    Core->>Serial: usb_serial_class_handler()
    Serial->>Serial: Update line_coding
    Serial->>DWC3: Control OUT transfer
    DWC3-->>Host: ACK

    Host->>DWC3: SETUP: SET_CONTROL_LINE_STATE
    DWC3->>Core: usb_stack_process_setup()
    Core->>Serial: usb_serial_class_handler()
    Serial->>Serial: Update DTR/RTS state
    Serial->>App: line_state_callback(DTR, RTS)
    Serial->>DWC3: Control status
    DWC3-->>Host: ACK

    Note over Host: Data Communication
    Host->>DWC3: Bulk OUT data
    DWC3->>DWC3: Process endpoint event
    DWC3->>Serial: usb_serial_rx_complete()
    Serial->>App: rx_callback(data, length)
    Serial->>DWC3: Submit new RX transfer

    App->>Serial: usb_serial_write(data, length)
    Serial->>Serial: Copy to TX buffer
    Serial->>Core: usb_stack_submit_transfer(TX_transfer)
    Core->>DWC3: ctrl_api->submit_transfer()
    DWC3->>DWC3: Setup TRB and start transfer
    DWC3-->>Host: Bulk IN data
    DWC3->>Serial: usb_serial_tx_complete()
    Serial->>App: tx_complete_callback()
```

## 4. USB Stack Deinitialization Sequence

```mermaid
sequenceDiagram
    participant App as Application
    participant Serial as usb_serial.c
    participant Core as usb_stack_core.c
    participant DWC3 as dwc3_controller.c
    participant PHY as qcom_phy.c
    participant TypeC as typec_manager.c

    App->>Serial: usb_serial_disable()
    Serial->>Serial: serial->configured = false
    Serial->>Core: usb_stack_disable(&serial->usb_dev)
    
    Core->>DWC3: ctrl_api->disable(controller_dev)
    DWC3->>DWC3: Clear RUN_STOP bit
    DWC3->>DWC3: Stop controller
    DWC3-->>Core: Success
    
    Core->>PHY: phy_api->power_off(phy_dev)
    PHY->>PHY: Power off PHYs
    PHY->>PHY: Disable clocks and regulators
    PHY-->>Core: Success
    
    Core->>TypeC: typec_api->disable(typec_dev)
    TypeC->>TypeC: Disable Type-C port
    TypeC-->>Core: Success
    
    Core->>Core: dev->state = USB_STACK_STATE_DETACHED
    Core-->>Serial: Success
    Serial-->>App: USB Serial disabled
```

## How USB Stack Gets Enabled

### **1. Automatic System Initialization (SYS_INIT)**

The USB stack is automatically enabled during Zephyr system boot through the `SYS_INIT` mechanism:

```c
// In usb_stack_init.c
SYS_INIT(usb_stack_system_init, POST_KERNEL, CONFIG_USB_STACK_INIT_PRIORITY);
```

**Boot Sequence:**
1. **Zephyr Kernel Boot** → **Device Driver Init** → **SYS_INIT Hooks** → **Application Init**

2. **Device Driver Initialization (Automatic):**
   - `qcom_phy_init()` - Called by `DEVICE_DT_INST_DEFINE` macro
   - `dwc3_controller_init()` - Called by `DEVICE_DT_INST_DEFINE` macro  
   - `typec_manager_init()` - Called by `DEVICE_DT_INST_DEFINE` macro

3. **USB Stack System Init (SYS_INIT):**
   - `usb_stack_system_init()` called at priority `CONFIG_USB_STACK_INIT_PRIORITY`
   - Discovers hardware devices using `DEVICE_DT_GET_ANY()`
   - Initializes USB stack core with `usb_stack_init()`
   - Sets `g_usb_stack_initialized = true`

### **2. Application-Level Enablement**

Applications enable USB functionality by calling:

```c
// Application code
usb_serial_init();      // Initialize USB serial
usb_serial_enable();    // Enable USB serial communication
```

**Application Flow:**
1. **`usb_serial_init()`** → Initialize USB serial device structure
2. **`usb_serial_enable()`** → Call `usb_stack_enable()` → Enable hardware components
3. **Hardware Enablement** → PHY power on → Controller enable → Type-C enable
4. **USB Ready** → Device appears on USB bus for host enumeration

### **3. Configuration Control**

The USB stack enablement is controlled by:

**Kconfig Options:**
- `CONFIG_USB_STACK_QCOM_PHY` - Enable Qualcomm PHY support
- `CONFIG_USB_STACK_DWC3_CONTROLLER` - Enable DWC3 controller support  
- `CONFIG_USB_STACK_TYPEC_SUPPORT` - Enable Type-C support
- `CONFIG_USB_STACK_INIT_PRIORITY` - Set initialization priority

**Device Tree:**
- Hardware devices must be defined in device tree
- `DEVICE_DT_GET_ANY()` discovers devices from device tree
- Missing device tree entries result in initialization failure

### **4. Runtime Control**

**Enable USB Stack:**
```c
struct usb_stack_device *dev = usb_stack_get_device();
usb_stack_enable(dev);  // Powers on PHY, enables controller
```

**Disable USB Stack:**
```c
usb_stack_disable(dev); // Powers off PHY, disables controller
```

### **5. Key Initialization Points**

1. **System Boot** → `SYS_INIT(usb_stack_system_init)` → USB stack structure ready
2. **Application Start** → `usb_serial_enable()` → Hardware powered and enabled
3. **USB Connection** → Host detects device → Enumeration begins
4. **Configuration** → Host sets configuration → Endpoints active → Data transfer ready

The USB stack uses a **two-phase initialization**:
- **Phase 1 (System)**: Structure initialization and device discovery
- **Phase 2 (Application)**: Hardware enablement and USB bus connection

This design allows the USB stack to be ready for use while giving applications control over when USB functionality is actually enabled on the bus.

## USB Stack Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│              (usb_serial.c - CDC ACM)                       │
├─────────────────────────────────────────────────────────────┤
│                   Adaptation Layer                          │
│            (usb_adaptation.c - Standard Requests)           │
├─────────────────────────────────────────────────────────────┤
│                 Function Driver Layer                       │
│              (usb_function_driver.c)                        │
├─────────────────────────────────────────────────────────────┤
│                 Device Core Layer                           │
│            (usb_device_core.c - State Machine)              │
├─────────────────────────────────────────────────────────────┤
│                USB Stack Core                               │
│         (usb_stack_core.c - Event Processing)               │
├─────────────────────────────────────────────────────────────┤
│                Controller Layer                             │
│              (dwc3_controller.c)                            │
├─────────────────────────────────────────────────────────────┤
│                  Hardware Layer                             │
│    (qcom_phy.c + typec_manager.c + Hardware Registers)      │
└─────────────────────────────────────────────────────────────┘
```

## Key Files and Their Roles

| File | Role | Key Functions |
|------|------|---------------|
| `usb_stack_init.c` | System initialization and device discovery | `usb_stack_system_init()`, `SYS_INIT` |
| `usb_stack_core.c` | Core USB stack functionality and event processing | `usb_stack_init()`, `usb_stack_enable()` |
| `usb_serial.c` | USB CDC ACM serial communication | `usb_serial_init()`, `usb_serial_enable()` |
| `dwc3_controller.c` | DWC3 hardware controller driver | `dwc3_controller_init()`, interrupt handling |
| `qcom_phy.c` | Qualcomm USB PHY driver | `qcom_phy_init()`, power management |
| `typec_manager.c` | Type-C port management | `typec_manager_init()`, CC detection |
| `usb_adaptation.c` | USB standard request handling | Standard setup packet processing |

## Event Flow Summary

1. **System Boot** → Hardware drivers auto-initialize → USB stack structure ready
2. **Application Call** → `usb_serial_enable()` → Hardware powered on → USB bus active
3. **Host Connection** → Cable connected → Events generated → State transitions
4. **Enumeration** → Setup packets → Descriptors exchanged → Device configured
5. **Data Transfer** → Bulk transfers → Application callbacks → Serial communication active
6. **Disconnection** → Cable removed → Events generated → Hardware powered down

This comprehensive flow ensures proper USB device operation from system boot through active communication and clean shutdown.
