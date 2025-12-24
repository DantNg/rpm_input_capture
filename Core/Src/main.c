/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "myEncoder/proximity_counter.h"
#include "queue/queue.h"
#include "modbus/modbus.h"
#include <stdlib.h>
#include <string.h>
#include "myFlash/myFlash.h"
#include "command_handler/command_handler.h"

// Declare external callback variable from modbus_master
extern ModbusResponseCallback modbus_user_on_response;

// Runtime measurement mode variable
MeasurementMode_t current_measurement_mode = MEASUREMENT_MODE_LENGTH;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CommandHandler_t cmdh;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POWER_STATUS_PIN GPIO_PIN_4
#define POWER_STATUS_GPIO_PORT GPIOA
// Chọn chế độ hoạt động
// #define MODBUS_MASTER
#define MODBUS_SLAVE
uint32_t PPR = 1;
float DIA = 0.25f;
uint32_t TIME = 100;
int64_t pulse_t = 0;
float current_speed=0;
////////////////////// Dùng cái này nếu stm32 là MODBUS SLAVE /////////////
#define SLAVE_ID 0x01
uint16_t holding_regs[10];
volatile uint32_t encoder_pulses = 0;
volatile uint32_t distance_mm = 0;
int32_t len_val;
float dia_val;
uint8_t current_modbus_slave_id = 0x01;  // Will be loaded from Flash
bool modbus_communication_enabled = true; // Will be loaded from Flash
// MODBUS MASTER timeout management
typedef enum {
	MASTER_IDLE, MASTER_WAITING_RESPONSE
} master_state_t;

// Runtime measurement values - now handled by encoder lib
uint8_t test = 0;
uint8_t uart_byte;
uint32_t parity = 0;
uint32_t c_uart = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
// ------------------- Printf --------------------------
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

GETCHAR_PROTOTYPE {
	uint8_t ch = 0;

	/* Clear the Overrun flag just before receiving the first character */
	__HAL_UART_CLEAR_OREFLAG(&huart1);

	/* Wait for reception of a character on the USART RX line - NO ECHO */
	HAL_UART_Receive(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	// Removed echo to prevent double character display
	return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

// Proximity counter instance
ProximityCounter_t proximity_counter;

// Command interface functions for proximity counter
void SetProximitySpeedUnit(int unit) {
    if (unit == 0) {
        ProximityCounter_SetSpeedUnit(&proximity_counter, PROXIMITY_SPEED_UNIT_RPM);
        printf("✅ Speed unit set to RPM\r\n");
    } else if (unit == 1) {
        ProximityCounter_SetSpeedUnit(&proximity_counter, PROXIMITY_SPEED_UNIT_M_MIN);
        printf("✅ Speed unit set to m/min\r\n");
    }
}

void SetProximityHysteresis(int index, int rpm_threshold, int hysteresis) {
    if (index >= 0 && index < 10) {
        ProximityCounter_SetHysteresisEntry(&proximity_counter, index, rpm_threshold, hysteresis);
        printf("✅ Hysteresis[%d] set: RPM=%d, Threshold=%d\r\n", index, rpm_threshold, hysteresis);
    }
}

void ShowProximityHysteresis(void) {
    printf("=== CURRENT HYSTERESIS TABLE ===\r\n");
    printf("Index | RPM Threshold | Hysteresis\r\n");
    printf("------|---------------|----------\r\n");
    for (int i = 0; i < 10; i++) {
        ProximityHysteresisEntry_t entry;
        if (ProximityCounter_GetHysteresisEntry(&proximity_counter, i, &entry)) {
            printf("  %d   |     %5d     |   %3d\r\n", i, entry.rpm_threshold, entry.hysteresis);
        } else {
            printf("  %d   |       ---     |   ---\r\n", i);
        }
    }
    printf("💡 Use: hyst set <index> <rpm> <hysteresis>\r\n");
}

void ClearProximityHysteresis(void) {
    // Clear by resetting the table size and reinitializing
    proximity_counter.hysteresis_table_size = 0;
    printf("✅ Hysteresis table cleared\r\n");
}

void SaveProximityHysteresis(void) {
    printf("💾 Saving hysteresis table to Flash...\r\n");
    printf("🔍 Current table has %d entries:\r\n", proximity_counter.hysteresis_table_size);
    
    myHysteresisTable table = {0};
    table.entry_count = proximity_counter.hysteresis_table_size;
    
    // Copy entries from proximity counter to Flash structure
    for (int i = 0; i < table.entry_count && i < 10; i++) {
        table.entries[i].rpm_threshold = proximity_counter.hysteresis_table[i].rpm_threshold;
        table.entries[i].hysteresis = proximity_counter.hysteresis_table[i].hysteresis;
        printf("  Entry %d: RPM=%d, Hyst=%d\r\n", i, 
               table.entries[i].rpm_threshold, table.entries[i].hysteresis);
    }
    
    if (myFlash_SaveHysteresisTable(&table) == HAL_OK) {
        printf("✅ Hysteresis table saved successfully (%d entries)\r\n", table.entry_count);
        
        // Verify save by reading back
        myHysteresisTable verify_table = {0};
        myFlash_LoadHysteresisTable(&verify_table);
        printf("🔍 Verification: entry_count=%d\r\n", verify_table.entry_count);
    } else {
        printf("❌ Failed to save hysteresis table to Flash\r\n");
    }
}

void LoadProximityHysteresis(void) {
    printf("📖 Loading hysteresis table from Flash...\r\n");
    
    myHysteresisTable table = {0};
    myFlash_LoadHysteresisTable(&table);
    
    // Debug: Show what was loaded from Flash
    printf("🔍 Flash data: entry_count=%d (0x%02X)\r\n", table.entry_count, table.entry_count);
    
    // Check if data is valid (not uninitialized Flash)
    // Uninitialized flash will have entry_count = 0xFF (255)
    if (table.entry_count != 0xFF && table.entry_count <= 10 && table.entry_count > 0) {
        // Clear current table completely
        proximity_counter.hysteresis_table_size = 0;
        memset(proximity_counter.hysteresis_table, 0, sizeof(proximity_counter.hysteresis_table));
        
        // Load ALL entries from Flash to proximity counter at correct positions
        for (int i = 0; i < table.entry_count && i < 10; i++) {
            // Only skip entries with uninitialized Flash data (0xFFFF)
            if (table.entries[i].rpm_threshold != 0xFFFF && table.entries[i].hysteresis != 0xFFFF &&
                table.entries[i].hysteresis > 0) {  // Allow RPM=0, only check hysteresis > 0
                proximity_counter.hysteresis_table[i].rpm_threshold = table.entries[i].rpm_threshold;
                proximity_counter.hysteresis_table[i].hysteresis = table.entries[i].hysteresis;
                // Update table size to include this entry
                if (i >= proximity_counter.hysteresis_table_size) {
                    proximity_counter.hysteresis_table_size = i + 1;
                }
                printf("  Entry %d: RPM=%d, Hyst=%d\r\n", i, 
                       table.entries[i].rpm_threshold, table.entries[i].hysteresis);
            }
        }
        printf("✅ Loaded %d valid hysteresis entries from Flash\r\n", proximity_counter.hysteresis_table_size);
    } else {
        printf("⚠️  Invalid Flash data (entry_count=%d) - using default hysteresis table\r\n", table.entry_count);
        ProximityCounter_InitDefaultHysteresis(&proximity_counter);
    }

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Hysteresis filter function now in proximity_counter library

static void Restart_UART3_DMA(void) {
	HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

static void Reinit_UARTs(void) {
	HAL_UART_DeInit(&huart1);
	HAL_UART_Init(&huart1);
	HAL_UART_DeInit(&huart3);
	HAL_UART_Init(&huart3);
	Restart_UART3_DMA();
	// UART1 now uses scanf - no interrupt setup needed
}

static void Apply_Parity_Config(uint32_t parity_mode) {
	switch (parity_mode) {
	case 0: // NONE
		huart1.Init.Parity = UART_PARITY_NONE;
		huart1.Init.WordLength = UART_WORDLENGTH_8B;
		huart3.Init.Parity = UART_PARITY_NONE;
		huart3.Init.WordLength = UART_WORDLENGTH_8B;
		break;
	case 1: // ODD
		huart1.Init.Parity = UART_PARITY_ODD;
		huart1.Init.WordLength = UART_WORDLENGTH_9B;
		huart3.Init.Parity = UART_PARITY_ODD;
		huart3.Init.WordLength = UART_WORDLENGTH_9B;
		break;
	case 2: // EVEN
		huart1.Init.Parity = UART_PARITY_EVEN;
		huart1.Init.WordLength = UART_WORDLENGTH_9B;
		huart3.Init.Parity = UART_PARITY_EVEN;
		huart3.Init.WordLength = UART_WORDLENGTH_9B;
		break;
	default:
		break;
	}
}

static void Apply_UART_Params(const myUARTParams *p) {
	if (!p)
		return;
	huart1.Init.BaudRate = p->baudRate;
	huart3.Init.BaudRate = p->baudRate;

	Apply_Parity_Config(p->parity);

	if (p->stopBits == 2U) {
		huart1.Init.StopBits = UART_STOPBITS_2;
		huart3.Init.StopBits = UART_STOPBITS_2;
	} else {
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart3.Init.StopBits = UART_STOPBITS_1;
	}

	TIME = p->frameTimeoutMs;
	Reinit_UARTs();
}

static void HoldingRegs_Refresh(void) {
	holding_regs[0] = PPR;					  // pulses per revolution
	holding_regs[1] = (uint16_t) (DIA * 1000); // diameter in mm
	holding_regs[2] = TIME;					  // sample time in ms
	holding_regs[3] = (uint16_t) ProximityCounter_GetRPM(&proximity_counter); // current RPM
}


static void Handle_Buttons(void) {
	static bool emergency_save_done = false;

	if (HAL_GPIO_ReadPin(POWER_STATUS_GPIO_PORT, POWER_STATUS_PIN)
			== GPIO_PIN_SET) {
		// Power loss detected - emergency save all critical parameters
		if (!emergency_save_done) {
			// 1. Save current length (highest priority - measurement data)
			uint32_t current_length_mm = 0;
			myFlash_SaveLength(current_length_mm);

			// 2. Save encoder params
			myEncoderParams enc_params = { .diameter = (uint32_t) (DIA * 1000), // Convert to mm
					.pulsesPerRev = PPR };
			myFlash_SaveEncoderParams(&enc_params);

			// 3. Save UART params
			myUARTParams p;
			p.baudRate = huart3.Init.BaudRate;
			p.parity = parity;
			p.stopBits = (huart3.Init.StopBits == UART_STOPBITS_2) ? 2U : 1U;
			p.frameTimeoutMs = TIME;
			myFlash_SaveUARTParams(&p);

			emergency_save_done = true;
		}

		// Minimal delay to debounce, then wait for power restoration or complete loss
		uint32_t start_time = HAL_GetTick();
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET
				&& (HAL_GetTick() - start_time) < 100) {
			// Keep watchdog alive during power loss event
			HAL_IWDG_Refresh(&hiwdg);
		}

		// Reset flag when power is restored
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
			emergency_save_done = false;
		}
	}
}

void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		uint16_t len = UART_RX_BUFFER_SIZE
				- __HAL_DMA_GET_COUNTER(huart->hdmarx);
		// Only accept plausible Modbus RTU frames (min 4 bytes incl. CRC)
		if (len >= 4 && len <= MODBUS_FRAME_MAX_LEN) {
			queue_frame_t frame;
			memcpy(frame.data, uart_rx_buffer, len);
			frame.len = len;
			if (!queue_push(&frame)) {
				printf("⚠️ Queue full- Remove frame!\r\n");
			}
		}

		// Reset DMA
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_DMAStop(huart);
		HAL_UART_Receive_DMA(huart, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	}
}

void on_write_single_register(uint16_t addr, uint16_t value) {
	switch (addr) {
	case 0:
		holding_regs[0] = value;
		PPR = value;
		ProximityCounter_UpdateConfig(&proximity_counter, PPR, DIA);
		break; // số xung
	case 1:
		holding_regs[1] = value;
		DIA = (float) value / 1000.0;
		ProximityCounter_UpdateConfig(&proximity_counter, PPR, DIA);
		break; // đường kính (mm)
	case 2:
		holding_regs[2] = value;
		TIME = value;
		ProximityCounter_SetTimeout(&proximity_counter, value * 10); // Convert to reasonable timeout
		break; // thời gian lấy mẫu (ms)
	default:
		break;
	}
}
void on_write_multiple_registers(uint16_t addr, const uint16_t *values,
		uint16_t quantity) {
	printf("Master write multi registers!\n");
}
void modbus_slave_setup() {
	ModbusSlaveConfig slave_cfg = { .id = SLAVE_ID, .coils = NULL, .coil_count =
			0, .discrete_inputs = NULL, .discrete_input_count = 0,
			.holding_registers = holding_regs, .holding_register_count = 10,
			.input_registers = NULL, .input_register_count = 0, .on_read_coils =
					NULL, .on_read_discrete_inputs = NULL,
			.on_read_holding_registers = NULL, .on_read_input_registers = NULL,
			.on_write_single_coil = NULL, .on_write_single_register =
					on_write_single_register, .on_write_multiple_coils = NULL,
			.on_write_multiple_registers = on_write_multiple_registers };

	modbus_init_slave(&huart3, &slave_cfg, MODBUS_MODE_RTU);
	holding_regs[0] = PPR;		  // số xung
	holding_regs[1] = DIA * 1000; // đường kính (mm)
	holding_regs[2] = TIME;		  // thời gian lấy mẫu(ms)
	MODBUS_SET_DE_RX();			  // DE = LOW (RX mode)
}

// Callback function để xử lý response từ Modbus slave
void on_modbus_master_response(uint8_t *data, uint16_t len) {
	printf("📩 Modbus Master received response: ");
	for (uint16_t i = 0; i < len; i++) {
		printf("0x%02X ", data[i]);
	}
	printf("\r\n");

	// Parse response để lấy thông tin chi tiết
	ModbusParsedResponse_t parsed;
	if (modbus_parse_response(data, len, &parsed)) {
		if (parsed.is_exception) {
			printf(
					"❌ Modbus Exception: Function 0x%02X, Exception Code 0x%02X\r\n",
					parsed.func_code, parsed.exception_code);
		} else {
			printf("✅ Modbus Response: SlaveID=0x%02X, Function=0x%02X",
					parsed.slave_id, parsed.func_code);

			switch (parsed.func_code) {
			case MODBUS_FC_READ_HOLDING_REGISTERS:
			case MODBUS_FC_READ_INPUT_REGISTERS:
				printf(", Registers Read: ");
				for (uint16_t i = 0; i < parsed.read.quantity; i++) {
					printf("%d ", parsed.read.registers[i]);
				}
				break;
			case MODBUS_FC_WRITE_SINGLE_REGISTER:
			case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
				printf(", Write ACK: Addr=0x%04X, Value=%d",
						parsed.write_ack.addr, parsed.write_ack.value);
				break;
			default:
				printf(", Unknown function code");
				break;
			}
			printf("\r\n");
		}
	} else {
		printf("⚠️ Failed to parse Modbus response\r\n");
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		// Ngay khi gửi xong, chuyển về chế độ nhận (DE=LOW) bằng macro
		MODBUS_SET_DE_RX();
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// Delegate to proximity counter library
	ProximityCounter_HandleCapture(&proximity_counter, htim);
}

// Timer overflow callback - counts overflows between captures
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Delegate to proximity counter library
	ProximityCounter_HandleOverflow(&proximity_counter, htim);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_TIM4_Init();
	MX_USART3_UART_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	// Initialize proximity counter
	ProximityCounterConfig_t prox_config = {
		.ppr = PPR,
		.diameter = DIA,
		.timeout_ms = 10000,  // 10 seconds
		.averaging_samples = 3
	};
	ProximityCounter_Init(&proximity_counter, &prox_config, &htim2);
	ProximityCounter_Start(&proximity_counter);
	
	setvbuf(stdin, NULL, _IONBF, 0);
	
	// Load Modbus configuration from Flash FIRST
	CommandHandler_InitModbusFromFlash();
	current_modbus_slave_id = CommandHandler_GetModbusSlaveId();
	modbus_communication_enabled = CommandHandler_IsModbusEnabled();
	printf("⬇️ Loaded Modbus config from Flash: ID=0x%02X Status=%s\r\n", 
		   current_modbus_slave_id, modbus_communication_enabled ? "ENABLED" : "DISABLED");
	
	// Load Speed Unit configuration from Flash
	CommandHandler_InitSpeedUnitFromFlash();
	SpeedDisplayUnit_t loaded_speed_unit = CommandHandler_GetSpeedDisplayUnit();
	SetProximitySpeedUnit((loaded_speed_unit == SPEED_UNIT_RPM) ? 0 : 1);
	printf("⬇️ Loaded speed display unit: %s\r\n", 
		   (loaded_speed_unit == SPEED_UNIT_RPM) ? "RPM" : "m/min");
	
	// Load Hysteresis table from Flash
	LoadProximityHysteresis();
	
	// Initialize Command Handler
	CommandHandler_Config_t cmd_config = {
		.huart = &huart1,
		.reinit_uarts = Reinit_UARTs,
		.apply_parity_config = Apply_Parity_Config,
		.encoder_init = NULL,  // No encoder init needed - using proximity counter
		.save_uart_params = (HAL_StatusTypeDef (*)(const void *))myFlash_SaveUARTParams,
		.save_encoder_params = (HAL_StatusTypeDef (*)(const void *))myFlash_SaveEncoderParams,
		.save_length = myFlash_SaveLength,
		.save_measurement_mode = myFlash_SaveMeasurementMode,
		.load_measurement_mode = myFlash_LoadMeasurementMode,
		.ppr = &PPR,
		.dia = &DIA,
		.time = &TIME,
		.parity = &parity,
		.measurement_mode = &current_measurement_mode,
		.huart1 = &huart1,
		.huart3 = &huart3,
		.encoder = NULL,
		.htim = &htim2,
		.slave_id = current_modbus_slave_id};
	// Load UART params from Flash and apply (simple validity check)
	myUARTParams saved_uart;
	myFlash_LoadUARTParams(&saved_uart);
	if (saved_uart.baudRate != 0xFFFFFFFFU && saved_uart.baudRate >= 2400U && saved_uart.baudRate <= 921600U)
	{
		Apply_UART_Params(&saved_uart);
		// Re-initialize CommandHandler after UART params change
		CommandHandler_Init(&cmdh, &cmd_config);
		printf(
			"⬇️ Loaded UART params from Flash: baud=%lu parity=%lu stop=%lu timeout=%lu ms\r\n",
			(unsigned long)saved_uart.baudRate, (unsigned long)saved_uart.parity,
			(unsigned long)saved_uart.stopBits,
			(unsigned long)saved_uart.frameTimeoutMs);
	}
	else
	{
		myUARTParams def = {115200U, 0U, 1U, TIME};
		if (myFlash_SaveUARTParams(&def) == HAL_OK)
		{
			printf("⚙️ Initialized default UART params and saved to Flash\r\n");
		}
	}

	// Load encoder params from Flash and apply
	myEncoderParams saved_encoder;
	myFlash_LoadEncoderParams(&saved_encoder);
	if (saved_encoder.diameter != 0xFFFFFFFFU && saved_encoder.pulsesPerRev != 0xFFFFFFFFU && saved_encoder.diameter > 0 && saved_encoder.pulsesPerRev > 0)
	{
		DIA = (float)saved_encoder.diameter / 1000.0f; // Convert from mm to meters
		PPR = saved_encoder.pulsesPerRev;
		printf("⬇️ Loaded encoder params from Flash: PPR=%lu DIA=%.3f\r\n",
			   PPR, (double)DIA);
	}
	else
	{
		// Initialize with current values if not in flash
		myEncoderParams def_enc = {
			.diameter = (uint32_t)(DIA * 1000),
			.pulsesPerRev = PPR};
		if (myFlash_SaveEncoderParams(&def_enc) == HAL_OK)
		{
			printf("⚙️ Initialized default encoder params and saved to Flash\r\n");
		}
	}

	// Initialize length value if not in flash
	uint32_t saved_length = myFlash_LoadLength();
	if (saved_length == 0xFFFFFFFFU)
	{
		// Initialize length to 0 if not set
		if (myFlash_SaveLength(0) == HAL_OK)
		{
			printf("⚙️ Initialized encoder length to 0 and saved to Flash\r\n");
		}
	}
	else
	{
		printf("⬇️ Loaded encoder length from Flash: %lu\r\n",
			   (unsigned long)saved_length);
	}

	// Load measurement mode from Flash
	uint32_t saved_mode = myFlash_LoadMeasurementMode();
	if (saved_mode == 0xFFFFFFFFU || saved_mode > MEASUREMENT_MODE_LENGTH)
	{
		// Initialize to LENGTH mode if not set or invalid
		current_measurement_mode = MEASUREMENT_MODE_LENGTH;
		if (myFlash_SaveMeasurementMode(MEASUREMENT_MODE_LENGTH) == HAL_OK)
		{
			printf("⚙️ Initialized measurement mode to LENGTH and saved to Flash\r\n");
		}
	}
	else
	{
		current_measurement_mode = (MeasurementMode_t)saved_mode;
		printf("⬇️ Loaded measurement mode from Flash: %s\r\n",
			   (current_measurement_mode == MEASUREMENT_MODE_LENGTH) ? "LENGTH" : "RPM");
	}

	// ----------------- UART3 -----------------------------
	HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	modbus_slave_setup();
	printf("🔌 Modbus SLAVE mode initialized\r\n");
	// ----------------- IWDG -------------------------------
	// Already initialized in MX_IWDG_Init(); keep refreshing in loop

	CommandHandler_Init(&cmdh, &cmd_config);
	printf("System initialized.\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		HAL_IWDG_Refresh(&hiwdg);
		CommandHandler_Process(&cmdh);
		// Process proximity counter
		ProximityCounter_ProcessCapture(&proximity_counter);
		ProximityCounter_CheckTimeout(&proximity_counter);

		// Display speed according to current unit setting
		ProximitySpeedUnit_t current_unit = ProximityCounter_GetSpeedUnit(&proximity_counter);
		current_speed = ProximityCounter_GetSpeed(&proximity_counter, current_unit);
		
    static uint32_t last_print_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_print_tick >= 1000) {
      last_print_tick = now;
      if (current_unit == PROXIMITY_SPEED_UNIT_RPM) {
        printf("RPM: %.2f\r\n", current_speed);
      } else {
        printf("Speed: %.2f m/min\r\n", current_speed);
      }
    }
		HoldingRegs_Refresh();
		Handle_Buttons();

		queue_frame_t frame;
		if (queue_pop(&frame))
		{
			// Only process Modbus if communication is enabled
			if (CommandHandler_IsModbusEnabled()) {
				modbus_handle_frame(frame.data, frame.len);
			}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Reload = 2499;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
