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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "usbd_cdc_if.h" // Untuk CDC_Transmit_FS
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_OUTPUT_PCFS 1
#define NUM_INPUT_PCFS  1
#define PINS_PER_PCF    8

// Alamat I2C dasar untuk PCF8574 (non-A variant)
// Alamat 7-bit adalah 0x20. Untuk HAL, alamat ini di-shift left 1 kali.
// (0x20 << 1) = 0x40
#define PCF8574_BASE_ADDRESS 0x20

// I2C Handles (diasumsikan hi2c1 dan hi2c2 didefinisikan oleh CubeMX)
#define I2C_OUTPUT_HANDLE hi2c1
#define I2C_INPUT_HANDLE  hi2c2

#define I2C_TIMEOUT_MS 100 // Timeout untuk operasi I2C
#define TEST_OUTPUT_DELAY_MS 10 // Delay setelah mengaktifkan satu output sebelum membaca input
#define INTER_PIN_TEST_DELAY_MS 50 // Delay antar pengujian pin output berikutnya

// Buffer untuk string output serial
#define SERIAL_OUT_BUFFER_SIZE 512
static char serial_out_buffer[SERIAL_OUT_BUFFER_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* Definitions for CableTester */
osThreadId_t CableTesterHandle;
const osThreadAttr_t CableTester_attributes = { .name = "CableTester",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for CableTesterSema */
osSemaphoreId_t CableTesterSemaHandle;
const osSemaphoreAttr_t CableTesterSema_attributes =
		{ .name = "CableTesterSema" };
/* USER CODE BEGIN PV */
// Alamat untuk PCF8574 output di I2C1
uint8_t output_pcf_addresses[NUM_OUTPUT_PCFS];
// Alamat untuk PCF8574 input di I2C2
uint8_t input_pcf_addresses[NUM_INPUT_PCFS];

volatile bool g_test_running = false; // Flag untuk status tes
char usb_rx_buffer[16]; // Buffer untuk perintah serial
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void StartCableTester(void *argument);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef PCF8574_WriteByte(I2C_HandleTypeDef *hi2c,
		uint8_t device_address, uint8_t data);
static HAL_StatusTypeDef PCF8574_ReadByte(I2C_HandleTypeDef *hi2c,
		uint8_t device_address, uint8_t *read_data);
static void Initialize_PCF_Addresses(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	// Implementasi sederhana, bisa ditambahkan timeout atau mekanisme non-blocking
	uint32_t timeout_start = HAL_GetTick();
	while (CDC_Transmit_FS((uint8_t*) ptr, len) == USBD_BUSY) {
		if (HAL_GetTick() - timeout_start > 100) { // Timeout 100ms
			return -1; // Gagal
		}
		osDelay(1); // Beri kesempatan RTOS
	}
	return len;
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
	MX_I2C1_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	MX_USB_DEVICE_Init();
	Initialize_PCF_Addresses();
//	printf("Cable Tester STM32F103C8T6 - PCF8574\r\n");
//	printf("Inisialisasi selesai. Memulai RTOS...\r\n");
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of CableTesterSema */
	CableTesterSemaHandle = osSemaphoreNew(1, 1, &CableTesterSema_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of CableTester */
	CableTesterHandle = osThreadNew(StartCableTester, NULL,
			&CableTester_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	if (CableTesterHandle == NULL) {
		printf("FATAL Error: Gagal membuat CableTesterTask!\r\n");
		Error_Handler();
	}

	if (CableTesterSemaHandle == NULL) {
		printf("FATAL Error: Gagal membuat CableTesterSemaHandle!\r\n");
		Error_Handler();
	}
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_BTN_Pin */
	GPIO_InitStruct.Pin = USER_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_LED_Pin */
	GPIO_InitStruct.Pin = USER_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Menginisialisasi array alamat PCF8574.
 */
static void Initialize_PCF_Addresses(void) {
	for (int i = 0; i < NUM_OUTPUT_PCFS; i++) {
		// Alamat 7-bit PCF8574 adalah 0x20 + i (jika A0-A2 di-set 0 hingga 7)
		output_pcf_addresses[i] = (PCF8574_BASE_ADDRESS + i) << 1; // Shift left untuk HAL API
	}
	for (int i = 0; i < NUM_INPUT_PCFS; i++) {
		input_pcf_addresses[i] = (PCF8574_BASE_ADDRESS + i) << 1; // Shift left untuk HAL API
	}
}

/**
 * @brief Menulis satu byte ke PCF8574.
 * @param hi2c Pointer ke handle I2C.
 * @param device_address Alamat 8-bit PCF8574 (sudah di-shift).
 * @param data Byte yang akan ditulis.
 * @retval HAL_StatusTypeDef Status operasi.
 */
static HAL_StatusTypeDef PCF8574_WriteByte(I2C_HandleTypeDef *hi2c,
		uint8_t device_address, uint8_t data) {
	uint8_t buffer[1];
	buffer[0] = data;
	return HAL_I2C_Master_Transmit(hi2c, device_address, buffer, 1,
	I2C_TIMEOUT_MS);
}

/**
 * @brief Membaca satu byte dari PCF8574.
 * @param hi2c Pointer ke handle I2C.
 * @param device_address Alamat 8-bit PCF8574 (sudah di-shift).
 * @param read_data Pointer untuk menyimpan byte yang dibaca.
 * @retval HAL_StatusTypeDef Status operasi.
 */
static HAL_StatusTypeDef PCF8574_ReadByte(I2C_HandleTypeDef *hi2c,
		uint8_t device_address, uint8_t *read_data) {
	return HAL_I2C_Master_Receive(hi2c, device_address, read_data, 1,
	I2C_TIMEOUT_MS);
}

// Handler untuk data yang diterima dari USB CDC
void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
	if (Len > 0 && Len < sizeof(usb_rx_buffer)) {
		memcpy(usb_rx_buffer, Buf, Len);
		usb_rx_buffer[Len] = '\0'; // Pastikan null-terminated

		if (Len == 1) {
			if (usb_rx_buffer[0] == '?') { // Perintah START
				if (!g_test_running) {
					g_test_running = true;
					osSemaphoreRelease(CableTesterSemaHandle); // Beri sinyal ke task untuk mulai
					printf("OK: Perintah START diterima. Memulai tes...\r\n");
				} else {
					printf("WARN: Tes sudah berjalan.\r\n");
				}
			} else if (usb_rx_buffer[0] == '!') { // Perintah STOP
				if (g_test_running) {
					g_test_running = false;
					printf(
							"OK: Perintah STOP diterima. Tes akan berhenti setelah pin saat ini.\r\n");
				} else {
					printf("INFO: Tes tidak sedang berjalan.\r\n");
				}
			} else {
				printf(
						"ERROR: Perintah tidak dikenal '%c'. Gunakan '?' untuk start, '!' untuk stop.\r\n",
						usb_rx_buffer[0]);
			}
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCableTester */
/**
 * @brief  Function implementing the CableTester thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCableTester */
void StartCableTester(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	uint8_t input_pcf_data[NUM_INPUT_PCFS];

	printf("Cable Tester Siap. Kirim '?' untuk memulai tes.\r\n");

	// Inisialisasi semua PCF8574 input ke mode input (tulis 0xFF)
	for (int i = 0; i < NUM_INPUT_PCFS; i++) {
		PCF8574_WriteByte(&I2C_INPUT_HANDLE, input_pcf_addresses[i], 0xFF);
	}
	osDelay(10);
	// Matikan semua output PCF8574 awal (semua high agar output mati)
	for (int i = 0; i < NUM_OUTPUT_PCFS; i++) {
		PCF8574_WriteByte(&I2C_OUTPUT_HANDLE, output_pcf_addresses[i], 0xFF);
	}
	osDelay(100);
	HAL_StatusTypeDef status;
	/* Infinite loop */
	for (;;) {
		// 1. Tunggu perintah START dari semaphore
		osSemaphoreAcquire(CableTesterSemaHandle, osWaitForever);
		if (!g_test_running) { // Pengecekan jika ada wakeup palsu
			continue;
		}

		printf("--- Memulai Siklus Tes Kabel ---\r\n");

		for (int out_pcf_idx = 0; out_pcf_idx < NUM_OUTPUT_PCFS;
				out_pcf_idx++) {
			for (int out_pin_idx = 0; out_pin_idx < PINS_PER_PCF;
					out_pin_idx++) {
				// *** PERIKSA PERINTAH BERHENTI ***
				if (!g_test_running) {
					goto test_loop_exit;
					// Gunakan goto untuk keluar dari loop bersarang
				}

				int absolute_out_pin = (out_pcf_idx * PINS_PER_PCF)
						+ out_pin_idx + 1;
				uint8_t output_byte = ~(1 << out_pin_idx);

				status = PCF8574_WriteByte(&I2C_OUTPUT_HANDLE,
						output_pcf_addresses[out_pcf_idx], output_byte);
				if (status != HAL_OK) {
					printf("%d;ERROR_WRITE\r\n", absolute_out_pin);
					osDelay(INTER_PIN_TEST_DELAY_MS);
					continue;
				}

				osDelay(TEST_OUTPUT_DELAY_MS);

				int detected_count = 0;
				int buffer_offset = 0;
				buffer_offset += snprintf(serial_out_buffer + buffer_offset,
				SERIAL_OUT_BUFFER_SIZE - buffer_offset, "%d;",
						absolute_out_pin);

				for (int in_pcf_idx = 0; in_pcf_idx < NUM_INPUT_PCFS;
						in_pcf_idx++) {
					status = PCF8574_ReadByte(&I2C_INPUT_HANDLE,
							input_pcf_addresses[in_pcf_idx],
							&input_pcf_data[in_pcf_idx]);
					if (status != HAL_OK)
						continue;

					for (int in_pin_idx = 0; in_pin_idx < PINS_PER_PCF;
							in_pin_idx++) {
						if (((input_pcf_data[in_pcf_idx] >> in_pin_idx) & 0x01)
								== 0) {
							int absolute_in_pin = (in_pcf_idx * PINS_PER_PCF)
									+ in_pin_idx + 1;
							if (detected_count > 0) {
								if (SERIAL_OUT_BUFFER_SIZE - buffer_offset > 1)
									buffer_offset += snprintf(
											serial_out_buffer + buffer_offset,
											SERIAL_OUT_BUFFER_SIZE
													- buffer_offset, ",");
							}
							if (SERIAL_OUT_BUFFER_SIZE - buffer_offset > 5) {
								buffer_offset += snprintf(
										serial_out_buffer + buffer_offset,
										SERIAL_OUT_BUFFER_SIZE - buffer_offset,
										"%d", absolute_in_pin);
							}
							detected_count++;
						}
					}
				}

				if (SERIAL_OUT_BUFFER_SIZE - buffer_offset > 3) {
					buffer_offset += snprintf(serial_out_buffer + buffer_offset,
					SERIAL_OUT_BUFFER_SIZE - buffer_offset, "\r\n");
				}
				printf("%s", serial_out_buffer);

				status = PCF8574_WriteByte(&I2C_OUTPUT_HANDLE,
						output_pcf_addresses[out_pcf_idx], 0xFF);
				osDelay(INTER_PIN_TEST_DELAY_MS);
			}
		}

		test_loop_exit:
		// Matikan semua output setelah siklus selesai atau dihentikan
		for (int i = 0; i < NUM_OUTPUT_PCFS; i++) {
			PCF8574_WriteByte(&I2C_OUTPUT_HANDLE, output_pcf_addresses[i],
					0xFF);
		}

		if (g_test_running) { // Jika selesai secara normal
			printf("--- Siklus Tes Selesai. Kirim '?' untuk tes lagi. ---\r\n");
		} else { // Jika dihentikan oleh perintah '!'
			printf(
					"--- Tes Dihentikan. Kirim '?' untuk memulai tes baru. ---\r\n");
		}
		g_test_running = false; // Reset flag setelah selesai atau dihentikan
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
