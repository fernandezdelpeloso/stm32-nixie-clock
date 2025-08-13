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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "math.h"
#include <stdio.h>
#include "lookup_table.h"
#include "ds3231_for_stm32_hal.h"
#include "ssd1306_fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Setup for OLED */
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_16x26
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02
#define N_ROW 1
#define N_PIXEL 1
#define N_READ ((N_PIXEL + 1) * 2 + 1)

/* Setup for IR presence detector
#define D6T_I2C_ADDR     (0x0A << 1)   // HAL expects 8-bit address
#define D6T_READ_LEN     5
#define D6T_CMD          0x4C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* Buffer to read */
char rx_data[2048];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const size_t lookup_table_length = sizeof(lookup_table) / sizeof(lookup_table[0]);

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define EARTH_RADIUS_KM 6371.0

float haversine(float lat1, float lon1, float lat2, float lon2) {
    float dlat = DEG_TO_RAD(lat2 - lat1);
    float dlon = DEG_TO_RAD(lon2 - lon1);
    float a = sinf(dlat / 2) * sinf(dlat / 2) +
              cosf(DEG_TO_RAD(lat1)) * cosf(DEG_TO_RAD(lat2)) *
              sinf(dlon / 2) * sinf(dlon / 2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
    return EARTH_RADIUS_KM * c;
}

const CsvRow* find_closest_entry(float my_lat, float my_lon) {
    const CsvRow* best = NULL;
    float min_distance = 1e9f;  // big initial value

    for (size_t i = 0; i < lookup_table_length; i++) {
        float d = haversine(my_lat, my_lon, lookup_table[i].Latitude, lookup_table[i].Longitude);
        if (d < min_distance) {
            min_distance = d;
            best = &lookup_table[i];
        }
    }

    return best;
}


int16_t conv8us_s16_le(uint8_t* buf, int n) {
    uint16_t ret;
    ret = (uint16_t)buf[n];
    ret += ((uint16_t)buf[n + 1]) << 8;
    return (int16_t)ret;   // and convert negative.
}

int day_of_year(int year, int month, int day) {
    static const int days_before_month[] = {
        0, 31, 59, 90, 120, 151, 181,
        212, 243, 273, 304, 334
    };
    int doy = days_before_month[month - 1] + day;

    // Leap year correction
    if (month > 2 &&
        ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) {
        doy += 1;
    }

    return doy;
}

static bool already_read_gps = false;


uint8_t pin_values[4];
uint8_t pin_numbers[4] = {0, 1, 4, 5};
 int time_h_m[4];
 int count_numbers, count_bits;
 int delay_time;
//#define GPIO_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7)

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    uint8_t d6t_addr = 0x0A << 1;  // 7-bit address shifted for STM32 HAL
    uint8_t cmd = 0x4C;
    uint8_t rx_buf[5];  // 1 status + 2 bytes for 1 temperature + 2 CRC
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

  /* Initialize all configured peripherals
  	 usart2 	  -
  	 usart3 	  -
  	 I2C1 (hi2c1) -
  	 I2C3 (hi2c3) -
  	 I2C4 (hi2c4) -
  */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  int time_h_m[4];
  ssd1306_Init();
  DS3231_Init(&hi2c3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		void control_pins(uint8_t pin_values[4], uint8_t pin_numbers[4])
		{
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_RESET);


		    // Iterate through the pin values vector
		    for (int i = 0; i < 4; i++) {
		        if (pin_values[i] > 0) {
		            // Set pin high (raise)
		            HAL_GPIO_WritePin(GPIOA, (int)(pow(2, pin_numbers[i])), GPIO_PIN_SET);  // Map index to GPIO pin
		        } else {
		            // Set pin low (lower)
		            HAL_GPIO_WritePin(GPIOA, (int)(pow(2, pin_numbers[i])), GPIO_PIN_RESET);  // Map index to GPIO pin
		        }
		    }
		}

	  HAL_UART_Receive_IT(&huart3, rx_data, sizeof(rx_data));


	  if (HAL_I2C_Master_Transmit(&hi2c1, d6t_addr, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
	      {
	          printf("Transmit failed!\r\n");
	          HAL_Delay(1000);
	          continue;
	      } else {
	    	  printf("Transmit OK!!!!\r\n");
	      }

	    uint8_t rawData[5] = {0};
	    uint8_t command = D6T_CMD;

	    HAL_Delay(500);

	    if (HAL_I2C_Master_Receive(&hi2c1, d6t_addr, rawData, 5, HAL_MAX_DELAY) != HAL_OK) {
	        printf("I2C READ FAILED\r\n");
	    }

	    int16_t temp_pixel = (int16_t)(rawData[3] << 8 | rawData[2]);
	    float temperature = temp_pixel / 10.0f;

	    printf("Raw Pixel Temp: %d (%d C)\r\n", (int)temp_pixel, (int)temperature);

	    HAL_Delay(500);

	  uint8_t hour = DS3231_GetHour();
	  uint8_t minute = DS3231_GetMinute();


	  time_h_m[1] = hour % 10;
	  time_h_m[0] = (int)(hour / 10);

	  time_h_m[3] = minute % 10;
	  time_h_m[2] = (int)(minute / 10);

	  for (count_numbers = 0; count_numbers < 4; count_numbers++) {

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);

		  switch (count_numbers)  {
					case 0:
						delay_time = 1500;
						break;
					case 1:
					case 3:
						delay_time = 125;
						break;
					case 2:
						delay_time = 500;
		  }

		  HAL_Delay(delay_time);


		  for (count_bits = 0; count_bits < 4; count_bits++) {
			  pin_values[count_bits] = time_h_m[count_numbers] & (int)pow(2, count_bits);
		  }

		  control_pins(pin_values, pin_numbers);

		  HAL_Delay(500);

	  }

	  char time_str[24];
	  snprintf(time_str, sizeof(time_str), "%02d:%02d UTC - %d oC", hour, minute, (int)temperature);
	  printf("\r\n\n");
	  printf(time_str);
	  printf("\r\n\n");


	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString(time_str, Font_7x10, White);
	  // ssd1306_Fill(1);
	  ssd1306_UpdateScreen();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000003;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00100D14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00100D14;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

bool validate_nmea_checksum(const char* sentence) {
    if (sentence[0] != '$') return false;

    const char* checksum_str = strstr(sentence, "*");
    int sizestr = strlen(checksum_str);
    if (!checksum_str || strlen(checksum_str) < 3) return false;

    uint8_t checksum = 0;
    for (const char* p = sentence; *p != '*' && *p != '\0'; p++) {
        checksum ^= *p;
    }

    uint8_t received;
    sscanf(checksum_str + 1, "%2hhX", &received);
    return checksum == received;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	if (!already_read_gps)
	{


		  // String to be analysed starts with $
		  char* substr = strstr(rx_data, "$GNRMC");
		  if (substr == NULL) return;

		  // and ends after the checksum
		  char* substr2 = strstr(substr, "\*");
		  if (substr2 == NULL || substr2[1] == '\0' || substr2[2] == '\0') return;
		  substr2[3] = '\0';

		  if (!validate_nmea_checksum(substr)) return;


		  // Parse GPS message
		  char** parsed_gps = parse_csv(substr);

		  // Returns if GPS data in invalid
		  if (parsed_gps[2][0] == 'V') return;

		  ssd1306_SetCursor(0,18);
		  ssd1306_WriteString("Valid GPS message!", Font_7x10, White);
		  ssd1306_UpdateScreen();

		  // Longitude extraction
		  double lon= atof(parsed_gps[5]);
		  int londeg = (int)(lon / 100);
		  double lonmin = lon - londeg * 100;
		  lon = londeg + lonmin / 60.0;
		  if (parsed_gps[6][0] == 'W') lon = -lon;
		  else if (parsed_gps[6][0] != 'E') return;

		  // Latitude extraction
		  double lat = atof(parsed_gps[3]);
		  int latdeg =  (int)(lat / 100);
		  double latmin = lat - latdeg * 100;
		  lat = latdeg + latmin / 60.0;
		  if (parsed_gps[4][0] == 'S') lat = -lat;
		  else if (parsed_gps[4][0] != 'N') return;

		  const CsvRow* nearest = find_closest_entry(lat, lon);

		  // Get UTC time from GPS (parsed_gps[1] format is "hhmmss.sss")
		  char* utc_str = parsed_gps[1];
		  int gps_hour = (utc_str[0] - '0') * 10 + (utc_str[1] - '0');
		  int gps_min = (utc_str[2] - '0') * 10 + (utc_str[3] - '0');
		  int gps_sec = (utc_str[4] - '0') * 10 + (utc_str[5] - '0');

		  // Get date from GPS (parsed_gps[9] format is "ddmmyy")
		  char* date_str = parsed_gps[9];
		  int gps_day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
		  int gps_month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
		  int gps_year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');

		  // Build date string for DST comparison
		  char today[11];
		  snprintf(today, sizeof(today), "%04d-%02d-%02d", gps_year, gps_month, gps_day);

		  bool DST = false;

		  // Determine if DST applies
		  int doy = day_of_year(gps_year, gps_month, gps_day);
		  float local_offset = nearest->UTC_Offset;
		  if (nearest->DST_Start_DOY > 0 &&
			  doy >= nearest->DST_Start_DOY &&
			  doy <= nearest->DST_End_DOY) {
			  local_offset += 1.0f;
			  DST = true;
		  }

		  // Apply local offset
		  int corrected_hour = gps_hour + (int)local_offset;
		  if (corrected_hour >= 24) corrected_hour -= 24;
		  if (corrected_hour < 0) corrected_hour += 24;

		  char time_str[24];
		  char offset[4];
		  if (fabs(local_offset - (int)local_offset) > 0.00000001) snprintf(offset, sizeof(offset), "%.1f", local_offset);
		  else snprintf(offset, sizeof(offset), "%d", (int)local_offset);
		  snprintf(time_str, sizeof(time_str), "%02d:%02d UTC%s%s", corrected_hour, gps_min, (lon >= 0.0) ? "+" : "-", offset);
		  if (DST) snprintf(time_str, sizeof(time_str), "%s \(DST\)", time_str);
		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString("Data from the GPS:", Font_7x10, White);
		  ssd1306_SetCursor(0, 11);
		  ssd1306_WriteString(time_str, Font_7x10, White);
		  ssd1306_SetCursor(0, 22);
		  ssd1306_WriteString("Saving to the RTC.", Font_7x10, White);
		  ssd1306_UpdateScreen();

		  //HAL_Delay(1);

		  // Update DS3231 with corrected time
		  DS3231_SetFullTime(corrected_hour, gps_min, gps_sec);
		  DS3231_SetFullDate(gps_day, gps_month, 1, gps_year); // 1 = Monday placeholder

		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(25,4);
		  ssd1306_WriteString("Time saved.", Font_7x10, White);
		  ssd1306_SetCursor(25,18);
		  ssd1306_WriteString("RTC in use!", Font_7x10, White);
		  ssd1306_UpdateScreen();

		  //HAL_Delay(1000);
		  //int b = 1;
		  //for (int a = 0; a < 50000000; a++) b++;

		  //delay_us(10);

		  //ssd1306_Fill(Black);
		  //ssd1306_UpdateScreen();

		  already_read_gps = true;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
