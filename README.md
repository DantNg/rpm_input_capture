# Tài liệu Thư viện myLib - STM32F1 Modbus Proximity Sensor System

## Tổng quan
Dự án này bao gồm một hệ thống đo lường proximity sensor với giao tiếp Modbus RTU/TCP cho STM32F103C8. Hệ thống hỗ trợ chế độ đo RPM với proximity sensor 1 xung/vòng, có configurable hysteresis filtering và speed unit display, với khả năng lưu cấu hình vào Flash.

## Cấu trúc Thư viện

### 1. **command_handler** - Xử lý Lệnh UART
**Mục đích**: Engine xử lý lệnh từ UART1 cho việc cấu hình và điều khiển hệ thống real-time

**Files chính**:
- `command_handler.h/c` - Core command processing engine

#### Kiến trúc Command Handler
**State Machine**:
- **IDLE**: Chờ lệnh mới
- **RECEIVING**: Đang nhận command string
- **PROCESSING**: Parse và execute command
- **RESPONDING**: Gửi response về UART

**Command Buffer Management**:
```c
typedef struct {
    char buffer[COMMAND_BUFFER_SIZE];  // Command buffer
    uint16_t index;                    // Current position
    uint8_t state;                     // State machine state
    uint8_t uart_port;                 // Active UART port
} CommandHandler_t;
```

#### Danh sách Lệnh Chi tiết

**1. Basic Commands** - Lệnh hệ thống cơ bản:
- `help` - Hiển thị danh sách tất cả commands
- `status` - Hiển thị trạng thái hệ thống chi tiết
- `reset` - Khởi động lại hệ thống
- Response: Thông tin chi tiết về cấu hình và trạng thái

**2. Speed Display Commands** - Đơn vị hiển thị tốc độ:
- `speed_unit` - Hiển thị đơn vị tốc độ hiện tại
- `speed_unit rpm` - Hiển thị tốc độ theo RPM (rotations per minute)
- `speed_unit m/min` - Hiển thị tốc độ theo m/min (meters per minute)
- Response: `"✅ Speed display unit set to [RPM/m/min]"`
- Note: Tự động chuyển đổi RPM sang m/min = RPM × π × diameter

**3. Hysteresis Configuration** - Cấu hình bảng hysteresis filtering:
- `hyst` - Hiển thị bảng hysteresis hiện tại
- `hyst show` - Hiển thị bảng hysteresis chi tiết
- `hyst set <index> <rpm> <threshold>` - Set entry tại index (0-9)
- `hyst add <rpm> <threshold>` - Thêm entry mới (tự động tìm index)
- `hyst clear` - Xóa tất cả entries
- `hyst default` - Khôi phục bảng mặc định
- `hyst save` - Lưu bảng vào Flash (TODO)
- `hyst load` - Tải bảng từ Flash (TODO)
- Example: `hyst set 2 500 15`, `hyst add 1200 35`
- Response: `"✅ Hysteresis[2] set: RPM=500, Threshold=15"`
- Note: Tối đa 10 entries, tự động sắp xếp theo RPM tăng dần

**4. UART Configuration** - Cấu hình UART:
- `uart1 baud <rate>` - Set UART1 baud rate (2400-921600)
- `uart3 baud <rate>` - Set uart3 baud rate (2400-921600)  
- `uart1 parity <n>` - Set parity cho tất cả UART (0=None,1=Odd,2=Even)
- `uart1 stop <n>` - Set UART1 stop bits (1 hoặc 2)
- `uart3 stop <n>` - Set uart3 stop bits (1 hoặc 2)
- Example: `uart1 baud 115200`, `uart3 parity 1`
- Response: `"✅ UART[1/3] [BAUD/PARITY/STOP] set to <value> and saved"`
- Auto-save: Lưu cấu hình UART vào Flash

**5. Proximity Sensor Configuration** - Cấu hình proximity sensor:
- `ppr <n>` - Set pulses per revolution (1-10000)
- `dia <f>` - Set đường kính bánh xe (0.001-10.0 meters)
- `time <ms>` - Set thời gian timeout cho no-pulse detection (10-10000ms)
- Example: `ppr 1`, `dia 0.25`, `time 5000`
- Response: `"✅ [PPR/DIA/TIMEOUT] set to <value> and saved"`
- Auto-save: Lưu tham số proximity sensor vào Flash
- Note: PPR thường = 1 cho proximity sensor 1 xung/vòng

**6. Modbus Configuration** - Cấu hình Modbus:
- `modbus` - Hiển thị trạng thái Modbus chi tiết
- `modbus id <n>` - Set Modbus SLAVE ID (1-247, hỗ trợ hex và decimal)
- `modbus enable` - Bật Modbus communication
- `modbus disable` - Tắt Modbus communication
- Example: `modbus id 5`, `modbus id 0x05`, `modbus enable`
- Response: `"✅ Modbus SLAVE ID set to 0x05 (5) and saved"`
- Auto-save: Lưu cấu hình Modbus vào Flash
- Note: Thay đổi Slave ID cần restart để có hiệu lực

#### Command Processing Flow

**Character-by-Character Processing**:
1. **Receive**: UART interrupt → `CommandHandler_Process()`
2. **Buffer**: Character append to command buffer (với backspace support)
3. **Parse**: Detect command delimiter (CR/LF)
4. **Execute**: Parse command string và execute function
5. **Respond**: Send response qua UART1 (với formatting và emoji)
6. **Cleanup**: Clear buffer, reset state, show prompt

**Command Parsing Algorithm**:
```c
// Enhanced command parsing với category-based processing
if (strcmp(cmd_buffer, "status") == 0) {
    Process_BasicCommands(handler, cmd_buffer);
} else if (strncmp(cmd_buffer, "mode ", 5) == 0) {
    Process_ModeCommands(handler, cmd_buffer);
} else if (strncmp(cmd_buffer, "speed_unit", 10) == 0) {
    Process_SpeedUnitCommands(handler, cmd_buffer);
} else if (strncmp(cmd_buffer, "modbus", 6) == 0) {
    Process_ModbusCommands(handler, cmd_buffer);
}
```

**Status Command Output Example**:
```
=== SYSTEM STATUS ===
PROXIMITY SENSOR: PPR=1 DIA=0.250 TIMEOUT=5000ms
UART: BAUD=115200 PARITY=0 STOP=1
MODBUS: SLAVE_ID=0x05 STATUS=ENABLED
SPEED DISPLAY: m/min (meters per minute)
CURRENT SPEED: 25.3 m/min (48.2 RPM)
HYSTERESIS: 5 entries configured
```

#### Error Handling và Validation

**Input Validation**:
- Command length check (max 32 characters)
- Parameter range validation (encoder: 1-10000, diameter: 0.001-10.0)
- Float/Integer format validation
- Modbus slave ID range check (1-247)
- Command syntax checking

**Error Responses**:
- `"❌ Unknown: <command> (type 'help' for commands)"` - Lệnh không được nhận diện
- `"❌ Invalid parameter"` - Tham số không hợp lệ
- `"❌ Value out of range"` - Giá trị ngoài phạm vi cho phép
- `"❌ Flash write failed"` - Lỗi ghi Flash memory
- `"❌ Invalid UART command format. Use: uart1/uart3 <command>"` - Sai format lệnh UART

**Command Examples**:
```
Cmd> status
=== SYSTEM STATUS ===
PROXIMITY SENSOR: PPR=1 DIA=0.250 TIMEOUT=5000ms
SPEED DISPLAY: RPM (rotations per minute)
CURRENT SPEED: 1250.5 RPM

Cmd> hyst show
=== CURRENT HYSTERESIS TABLE ===
Index | RPM Threshold | Hysteresis
------|---------------|----------
  0   |       0       |     5
  1   |     500       |    15
  2   |    1000       |    25

Cmd> hyst set 3 1500 40
✅ Hysteresis[3] set: RPM=1500, Threshold=40

Cmd> speed_unit m/min
✅ Speed display unit set to m/min (meters per minute)
📏 Speed will be displayed as linear speed

Cmd> modbus id 0x10
✅ Modbus SLAVE ID set to 0x10 (16) and saved
⚠️  Note: System restart required for ID change to take effect
```

**Cách sử dụng chi tiết**:
```c
// Khởi tạo command handler
CommandHandler_Config_t cmd_config = {
    .huart = &huart1,
    .reinit_uarts = Reinit_UARTs,
    .apply_parity_config = Apply_Parity_Config,
    .save_uart_params = myFlash_SaveUARTParams,
    .save_modbus_config = myFlash_SaveModbusConfig,
    // ... other config parameters
};
CommandHandler_Init(&cmdh, &cmd_config);

// Load Modbus config from Flash
CommandHandler_InitModbusFromFlash();

// Main loop processing
while(1) {
    CommandHandler_Process(&cmdh);
    // Other system tasks
    HAL_Delay(10);
}
```

### 2. **modbus** - Giao tiếp Modbus RTU
**Mục đích**: Implementation đầy đủ Modbus RTU Master/Slave với RS485 support và advanced error handling

**Files chính**:
- `modbus.h/c` - Core Modbus protocol functions
- `modbus_master/` - Modbus Master implementation
- `modbus_slave/` - Modbus Slave implementation  
- `crc16/` - CRC16 calculation module

#### Kiến trúc Modbus Protocol

**Protocol Stack**:
```
┌─────────────────────────┐
│   Application Layer     │ ← User functions
├─────────────────────────┤
│   Modbus RTU Layer      │ ← Frame processing
├─────────────────────────┤ 
│   RS485 Physical Layer  │ ← uart3 + DE control
└─────────────────────────┘
```

**Frame Structure**:
```
┌───────┬────────────┬──────────┬─────────┬─────────┐
│ Addr  │ Function   │   Data   │ CRC Low │ CRC High│
│ (1B)  │   Code     │  (0-252B)│  (1B)   │  (1B)   │
│       │   (1B)     │          │         │         │
└───────┴────────────┴──────────┴─────────┴─────────┘
```

#### Modbus Functions Support

**Implemented Function Codes**:
- **0x03** - Read Holding Registers
- **0x04** - Read Input Registers  
- **0x06** - Write Single Register
- **0x10** - Write Multiple Registers
- **0x16** - Mask Write Register

**Exception Codes**:
- **0x01** - Illegal Function
- **0x02** - Illegal Data Address  
- **0x03** - Illegal Data Value
- **0x04** - Slave Device Failure
- **0x06** - Slave Device Busy

#### Register Map Chi tiết

**Holding Registers (Read/Write)**:
```
Address │ Description           │ Type    │ Access │ Units
────────┼─────────────────────  ┼─────────┼────────┼──────
0x0000  │ RPM Value Low Word    │ UINT16  │ R/W    │ rpm
0x0001  │ RPM Value High Word   │ UINT16  │ R/W    │ rpm  
0x0002  │ Length Low Word       │ UINT16  │ R/W    │ mm
0x0003  │ Length High Word      │ UINT16  │ R/W    │ mm
0x0004  │ Measurement Mode      │ UINT16  │ R/W    │ enum
0x0005  │ Sensor Type           │ UINT16  │ R/W    │ enum
0x0006  │ System Status         │ UINT16  │ R      │ flags
0x0007  │ Error Counter         │ UINT16  │ R      │ count
0x0008  │ Encoder Counter Low   │ UINT16  │ R      │ pulses
0x0009  │ Encoder Counter High  │ UINT16  │ R      │ pulses
0x000A  │ Configuration Flags   │ UINT16  │ R/W    │ bits
0x000B  │ Firmware Version      │ UINT16  │ R      │ BCD
```

**Input Registers (Read Only)**:
```
Address │ Description           │ Type    │ Units  │ Update Rate
────────┼─────────────────────  ┼─────────┼────────┼────────────
0x0000  │ Current RPM Raw       │ UINT16  │ rpm    │ 100ms
0x0001  │ Current Length Raw    │ UINT16  │ mm     │ 100ms
0x0002  │ Temperature           │ INT16   │ 0.1°C  │ 1s
0x0003  │ Supply Voltage        │ UINT16  │ mV     │ 1s
0x0004  │ Runtime Hours Low     │ UINT16  │ hours  │ 1h
0x0005  │ Runtime Hours High    │ UINT16  │ hours  │ 1h
```

#### RS485 Communication

**DE Pin Control Macros**:
```c
// Automatic transmit enable
#define MODBUS_SET_DE_TX() HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET)

// Automatic receive enable  
#define MODBUS_SET_DE_RX() HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET)
```

**Communication Parameters**:
- **Baud Rate**: 9600, 19200, 38400, 115200 bps
- **Data Bits**: 8
- **Parity**: None, Even, Odd
- **Stop Bits**: 1 hoặc 2
- **Flow Control**: RTS/DE pin control

**Timing Requirements**:
- **T1.5**: 1.5 character time (silence between characters)
- **T3.5**: 3.5 character time (frame delimiter)
- **Response Timeout**: 1000ms (configurable)
- **DE Pin Timing**: 10μs setup/hold time

#### Master Mode Implementation

**Master State Machine**:
```
┌─────────┐ send request ┌──────────┐ receive    ┌──────────┐
│  IDLE   │ ────────────→ │ WAITING  │ ─────────→ │ RESPONSE │
└─────────┘               └──────────┘            └──────────┘
     ↑                         │                       │
     │ timeout/complete         │ timeout               │
     └─────────────────────────┴───────────────────────┘
```

**Master Functions**:
```c
// Read multiple holding registers
ModbusStatus_t ModbusMaster_ReadHoldingRegisters(
    uint8_t slave_id, 
    uint16_t start_addr, 
    uint16_t num_regs,
    uint16_t* data_buffer
);

// Write single register
ModbusStatus_t ModbusMaster_WriteSingleRegister(
    uint8_t slave_id,
    uint16_t reg_addr, 
    uint16_t value
);

// Write multiple registers
ModbusStatus_t ModbusMaster_WriteMultipleRegisters(
    uint8_t slave_id,
    uint16_t start_addr,
    uint16_t num_regs,
    uint16_t* data_buffer  
);
```

#### Slave Mode Implementation

**Slave State Machine**:
```
┌─────────┐ receive frame ┌──────────┐ process    ┌──────────┐
│ LISTEN  │ ─────────────→ │ VALIDATE │ ─────────→ │ RESPOND  │
└─────────┘                └──────────┘            └──────────┘
     ↑                         │                       │
     │ response sent            │ invalid frame         │
     └─────────────────────────┴───────────────────────┘
```

**Slave Configuration**:
```c
typedef struct {
    uint8_t slave_id;                    // Device address (1-247)
    uint16_t* holding_registers;         // Holding registers array
    uint16_t* input_registers;           // Input registers array  
    uint16_t num_holding_regs;           // Number of holding registers
    uint16_t num_input_regs;             // Number of input registers
    ModbusResponseCallback on_response;  // Response callback function
} ModbusSlave_Config_t;
```

#### Error Handling và Diagnostics

**Error Detection**:
- **CRC Check**: Hardware CRC16 calculation
- **Frame Validation**: Length, timing, format checks
- **Address Validation**: Slave ID verification
- **Function Code Validation**: Supported function check
- **Data Range Validation**: Register address bounds

**Diagnostic Counters**:
```c
typedef struct {
    uint32_t total_messages;      // Total messages received
    uint32_t valid_messages;      // Valid messages processed
    uint32_t crc_errors;          // CRC error count
    uint32_t frame_errors;        // Frame format errors
    uint32_t timeout_errors;      // Communication timeouts
    uint32_t exception_responses; // Exception responses sent
} ModbusDiagnostics_t;
```

**Cách sử dụng chi tiết**:

**Master Mode Setup**:
```c
// Initialize Modbus Master
ModbusMaster_Init(&huart3);  // Use uart3 for communication

// Configure RS485 DE pin
Modbus_ConfigureDEPin(DE_GPIO_Port, DE_Pin);

// Read encoder values from slave device
uint16_t encoder_data[4];  // Buffer for 2 float values
if (ModbusMaster_ReadHoldingRegisters(1, 0x0000, 4, encoder_data) == MODBUS_OK) {
    float rpm = *((float*)&encoder_data[0]);
    float length = *((float*)&encoder_data[2]);
    printf("RPM: %.2f, Length: %.2f\n", rpm, length);
}
```

**Slave Mode Setup**:
```c
// Initialize register arrays
uint16_t holding_regs[32];
uint16_t input_regs[16];

// Configure slave
ModbusSlave_Config_t slave_config = {
    .slave_id = 1,
    .holding_registers = holding_regs,
    .input_registers = input_regs,
    .num_holding_regs = 32,
    .num_input_regs = 16,
    .on_response = my_response_callback
};

// Initialize slave
ModbusSlave_Init(&slave_config, &huart3);

// In main loop
while(1) {
    ModbusSlave_Process();  // Process incoming requests
    Update_RegisterValues(); // Update measurement values
    HAL_Delay(10);
}
```

### 3. **proximity_counter** - Thư viện Đo lường Proximity Sensor
**Mục đích**: Thư viện đo RPM sử dụng proximity sensor 1 xung/vòng với configurable hysteresis filtering

**Files chính**:
- `proximity_counter.h/c` - Core proximity sensor functions
- `measurement_mode.h` - Định nghĩa chế độ đo (legacy, giờ chỉ support RPM)

#### Kiến trúc Proximity Counter

**Input Capture với TIM2**:
- Sử dụng Timer 2 Input Capture Channel 1 để bắt cạnh lên của xung
- Timer chạy với tần số 1MHz (72MHz/72 prescaler)
- Tính RPM từ thời gian giữa hai xung liên tiếp

**Configurable Hysteresis Table**:
```c
typedef struct {
    uint16_t rpm_threshold;  // RPM threshold for this entry
    uint16_t hysteresis;     // Hysteresis value for this threshold
} ProximityHysteresisEntry_t;

// Maximum 10 entries, auto-sorted by RPM threshold
ProximityHysteresisEntry_t hysteresis_table[10];
```

**Speed Unit Support**:
```c
typedef enum {
    PROXIMITY_SPEED_UNIT_RPM = 0,   // Rotations per minute
    PROXIMITY_SPEED_UNIT_M_MIN = 1  // Meters per minute
} ProximitySpeedUnit_t;

// Conversion: m/min = RPM × π × diameter
float ProximityCounter_GetSpeed(const ProximityCounter_t *prox_counter, ProximitySpeedUnit_t unit);
```

#### Core Functions

**Initialization**:
```c
// Initialize proximity counter với configuration
ProximityCounterConfig_t config = {
    .ppr = 1,              // 1 pulse per revolution
    .diameter = 0.25f,     // 0.25 meters diameter
    .timeout_ms = 10000,   // 10 second timeout
    .averaging_samples = 3  // Average over 3 samples
};
ProximityCounter_Init(&proximity_counter, &config, &htim2);
ProximityCounter_Start(&proximity_counter);
```

**Measurement Processing**:
```c
// Main loop processing
ProximityCounter_ProcessCapture(&proximity_counter);  // Process new captures
ProximityCounter_CheckTimeout(&proximity_counter);    // Handle timeout

// Get measurements
float rpm = ProximityCounter_GetRPM(&proximity_counter);
float speed_m_min = ProximityCounter_GetSpeed(&proximity_counter, PROXIMITY_SPEED_UNIT_M_MIN);
```

**Hysteresis Management**:
```c
// Set individual hysteresis entry
ProximityCounter_SetHysteresisEntry(&proximity_counter, 0, 0, 5);      // 0-99 RPM: threshold=5
ProximityCounter_SetHysteresisEntry(&proximity_counter, 1, 500, 20);   // 500+ RPM: threshold=20
ProximityCounter_SetHysteresisEntry(&proximity_counter, 2, 1200, 40);  // 1200+ RPM: threshold=40

// Initialize default table
ProximityCounter_InitDefaultHysteresis(&proximity_counter);
```

**Interrupt Handlers** (called from main.c):
```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    ProximityCounter_HandleCapture(&proximity_counter, htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    ProximityCounter_HandleOverflow(&proximity_counter, htim);
}
```

#### Advanced Features

**Adaptive Period Averaging**:
- Lần đầu: sử dụng single period
- Các lần sau: average 3 periods để tăng độ chính xác
- Tự động reset khi có timeout

**Configurable Hysteresis Filtering**:
- Bảng tối đa 10 entries
- Mỗi entry có RPM threshold và hysteresis value
- Tự động select threshold dựa trên RPM hiện tại
- Stability counter để smooth updates

**Timeout Detection**:
- Configurable no-pulse timeout (default 10s)
- Tự động reset RPM = 0 khi không có xung
- Reset tất cả averaging và hysteresis state

**Overflow Handling**:
- 16-bit timer với overflow counting
- Support đo period rất dài (low RPM)
- 64-bit effective range cho timing

**Tính năng**:
- ✅ RPM measurement với 1 pulse/revolution proximity sensor
- ✅ Configurable hysteresis table (max 10 entries)
- ✅ Speed display units: RPM hoặc m/min conversion
- ✅ Adaptive period averaging cho accuracy
- ✅ Timeout detection với auto-reset
- ✅ Input capture với overflow handling
- ✅ Real-time parameter updates qua commands
- ✅ Integration với Modbus registers

### 4. **myFlash** - Flash Memory Persistence
**Mục đích**: Lưu trữ cấu hình vào Flash memory để persistence qua power cycles

**Files chính**:
- `myFlash.h/c` - Flash read/write operations

**Tính năng**:
- ✅ Save/Load UART configuration (baud, parity, stop bits)
- ✅ Save/Load Proximity Sensor parameters (PPR, diameter, timeout)
- ✅ Save/Load Hysteresis table (10 entries max)
- ✅ Save/Load Speed unit preference (RPM/m/min)
- ✅ Save/Load Modbus configuration (slave ID, enable/disable)
- ✅ Flash page management với wear leveling
- ✅ Data integrity verification với default fallback

**Flash Layout**:
```
STM32F103C8 Flash Memory Layout:
0x0801EC00: UART Config (4 words: baud, parity, stop, timeout)
0x0801F800: Proximity Params (3 words: diameter_mm, PPR, timeout_ms)
0x0801FC00: Speed Unit (1 word: unit enum)
0x0801F400: Hysteresis Table (20 words: 10 entries × 2 words each)
0x0801F000: Modbus Config (1 word: packed slave_id, enabled, reserved)
```

**Modbus Configuration Structure**:
```c
typedef struct {
    uint8_t slave_id;         // Modbus slave ID (1-247)
    uint8_t enabled;          // 1=enabled, 0=disabled
    uint16_t reserved;        // padding for 4-byte alignment
} myModbusConfig;

// Save/Load functions
HAL_StatusTypeDef myFlash_SaveModbusConfig(const myModbusConfig *config);
void myFlash_LoadModbusConfig(myModbusConfig *out);
```

**Cách sử dụng**:
```c
// Save configuration
Flash_Save_UART_Config(uart_port);
Flash_Save_Encoder_Value(encoder_value);
Flash_Save_Length_Value(length_value);
Flash_Save_Measurement_Mode(mode);

// Load configuration
uint32_t uart_config = Flash_Load_UART_Config();
float encoder_val = Flash_Load_Encoder_Value();
MeasurementMode_t mode = Flash_Load_Measurement_Mode();
```

### 5. **queue** - Buffer Management
**Mục đích**: Circular buffer implementation cho UART data handling

**Files chính**:
- `queue.h/c` - Circular queue implementation

**Tính năng**:
- ✅ Thread-safe circular buffer
- ✅ Dynamic size allocation
- ✅ Overflow protection
- ✅ UART receive buffer management

**Cách sử dụng**:
```c
Queue_t uart_queue;
Queue_Init(&uart_queue, buffer, BUFFER_SIZE);

// Enqueue data
Queue_Enqueue(&uart_queue, data);

// Dequeue data
uint8_t data = Queue_Dequeue(&uart_queue);
```

### 6. **storage** - Non-Volatile Storage
**Mục đích**: High-level storage abstraction layer

**Files chính**:
- `nonVolatileStorage.h/c` - Storage management

**Tính năng**:
- ✅ Abstraction layer trên myFlash
- ✅ Configuration management
- ✅ Data versioning
- ✅ Backup and restore

## Tích hợp Hệ thống

### Luồng hoạt động chính:
1. **Khởi tạo**: Load tất cả cấu hình từ Flash (UART, Proximity, Modbus, Hysteresis)
2. **Command Processing**: Xử lý lệnh từ UART1 với real-time response
3. **RPM Measurement**: Đo proximity sensor với configurable hysteresis filtering
4. **Speed Display**: Dynamic unit conversion (RPM ↔ m/min) dựa trên user preference
5. **Modbus Communication**: Giao tiếp qua uart3 với enable/disable control
6. **Configuration Save**: Auto-save mọi thay đổi vào Flash
7. **Hysteresis Management**: Real-time cập nhật filtering parameters

### Runtime Configuration Management:
- **Hysteresis Table Control**: Cấu hình tối đa 10 threshold entries qua UART
- **Dynamic Speed Units**: Chuyển đổi RPM ↔ m/min real-time
- **Modbus Control**: Bật/tắt Modbus mà không cần restart
- **Proximity Parameters**: Thay đổi PPR, diameter, timeout với immediate effect
- **Flash Persistence**: Tất cả cấu hình tự động lưu và restore

### Command Integration với Main System:
```c
// Main loop integration
while(1) {
    // Process user commands
    CommandHandler_Process(&cmdh);
    
    // Update RPM measurements
    ProximityCounter_ProcessCapture(&proximity_counter);
    ProximityCounter_CheckTimeout(&proximity_counter);
    
    // Display speed according to user preference
    ProximitySpeedUnit_t unit = ProximityCounter_GetSpeedUnit(&proximity_counter);
    float speed = ProximityCounter_GetSpeed(&proximity_counter, unit);
    if (unit == PROXIMITY_SPEED_UNIT_RPM) {
        printf("RPM: %.2f\r\n", speed);
    } else {
        printf("Speed: %.2f m/min\r\n", speed);
    }
    
    // Process Modbus only if enabled
    if (CommandHandler_IsModbusEnabled()) {
        if (queue_pop(&frame)) {
            modbus_handle_frame(frame.data, frame.len);
        }
    } else {
        // Discard Modbus frames when disabled
        queue_frame_t dummy;
        while(queue_pop(&dummy)) { /* discard */ }
    }
    
    HAL_Delay(10);
}
```

## Cấu hình Hardware

### Pinout:
- **UART1** (Debug): PA9 (TX), PA10 (RX)
- **uart3** (Modbus): PB10 (TX), PB11 (RX)
- **TIM2** (Proximity): PA0 (CH1 Input Capture)
- **DE Control**: PB12 (Modbus DE pin)
- **Power Status**: PA4 (Power loss detection)

### Peripherals:
- **TIM2**: Input Capture mode cho proximity sensor
- **IWDG**: Watchdog timer
- **Flash**: Configuration storage
- **UART1/3**: Communication interfaces

## Build và Deployment

### Compile:
```bash
cd f1_flash_length
make clean && make all
```

### Flash Programming:
- Sử dụng STM32CubeIDE hoặc ST-Link
- Binary location: `Debug/f1_flash_length.bin`

### Debug:
- SWD debugging qua ST-Link
- UART1 debug messages
- Modbus monitoring qua uart3

## Troubleshooting

### Common Issues:
1. **RPM Instability**: Check proximity sensor alignment, cấu hình hysteresis table phù hợp
2. **Modbus Communication**: Verify DE pin timing, baudrate settings
3. **Flash Corruption**: Check power supply stability, implement backup/restore
4. **Hysteresis Not Working**: Verify table entries, check RPM threshold ranges
5. **Speed Unit Display**: Ensure diameter được set chính xác cho m/min conversion

### Debug Commands:
```
# System Status và Information
status             - Hiển thị trạng thái hệ thống chi tiết
help              - Danh sách tất cả commands
reset             - System restart

# Speed Display Units
speed_unit        - Hiển thị đơn vị tốc độ hiện tại  
speed_unit rpm    - Hiển thị tốc độ theo RPM
speed_unit m/min  - Hiển thị tốc độ theo m/min

# Hysteresis Configuration (NEW)
hyst              - Hiển thị bảng hysteresis hiện tại
hyst show         - Hiển thị bảng chi tiết với index
hyst set 0 0 5    - Set entry 0: RPM>=0, threshold=5
hyst add 1500 35  - Thêm entry: RPM>=1500, threshold=35
hyst clear        - Xóa tất cả entries
hyst default      - Khôi phục bảng mặc định
hyst save         - Lưu bảng vào Flash
hyst load         - Tải bảng từ Flash

# Modbus Configuration
modbus            - Trạng thái Modbus chi tiết
modbus id 5       - Set slave ID = 5 (decimal)
modbus id 0x05    - Set slave ID = 5 (hex)  
modbus enable     - Bật Modbus communication
modbus disable    - Tắt Modbus communication

# UART Configuration
uart1 baud 115200 - Set UART1 baud rate
uart3 baud 9600   - Set uart3 baud rate
uart1 parity 1    - Set parity = Odd (global)
uart1 stop 2      - Set UART1 stop bits = 2

# Proximity Sensor Parameters (UPDATED)
ppr 1             - Set pulses per revolution (usually 1)
dia 0.25          - Set diameter = 0.25 meters
time 5000         - Set no-pulse timeout = 5000ms
```

---
*Tài liệu được tạo ngày: 21/12/2025*  
*Phiên bản: 1.0*  
*Tác giả: dat.nguyen*