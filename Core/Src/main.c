/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C_ADDRESS 0x4E//Adres układu slave

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void LCD_send(uint8_t rs, uint8_t data, uint8_t is8BitsMode);
void LCD_printf(char *data);
void LCD_begin(int8_t line, int8_t column);
void LCD_printf_number(int16_t number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  /* USER CODE BEGIN 2 */

  //Inicjalizacja
  HAL_Delay(16);//Opóźnienie 16ms
  LCD_send(0, 0x30, 0);
  HAL_Delay(5);//Opóźnienie 5ms
  LCD_send(0, 0x30, 0);
  HAL_Delay(1);//Opóźnienie 1ms
  LCD_send(0, 0x30, 0);
  HAL_Delay(1);//Opóźnienie 1ms
  //Konfiguracja
  LCD_send(0, 0x20, 0);
  LCD_send(0, 0x28, 1);//Tryb pracy (function set)
  LCD_send(0, 0x0F, 1);//Włącz wyświetlacz (display on)
  LCD_send(0, 0x01, 1);//Wyczyszczenie wyświetlacza
  LCD_send(0, 0x06, 1);//Tryb wprowadzania (entry mode set)

  LCD_printf("Test 1");//Wyświetlenie napisu "Test 1"
  LCD_begin(2,4);//Przejście do 4 kolumny w 2 linii wyświetlacza
  LCD_printf("Test 2");//Wyświetlenie napisu "Test 2"

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00C0EAFF;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//Funkcja umożliwiająca określenie wiersza i kolumny,
//od których rozpocznie się wyświetlanie.
//line - wartość 1 oznacza pierwszy wiersz wyświetlacza, a 2 drugi
//column - kolumna wyświetlacza (numeracja kolumn rozpoczyna się od 1)
void LCD_begin(int8_t line, int8_t column){
	int8_t a;//Adres pozycji w wierszu
	if(line==1){//Pierwszy wiersz wyświetlacza
		a = column-1;//Przypisanie wartości adresu pozycji w wierszu
	}
	else if(line==2){//Drugi wiersz wyświetlacza
		a = column+0x3f;//Przypisanie wartości adresu pozycji w wierszu
	}
	a = a|0x80;//Przypisanie wartości 1 dla najstarszego bitu
	LCD_send(0, a, 1);//Ustawienie adresu DD-RAM
}

//Funkcja umożliwiająca wyświetlanie dowolnego ciągu znaków na wyświetlaczu
void LCD_printf(char *data){//data - ciąg znaków do wyświetlenia
	uint8_t i=0;//zmienna przechowująca numer aktualnie przetwarzanego znaku z ciągu
	while(data[i]!=0){//dopóki ciąg znaków się nie skończy
		LCD_send(1,data[i],1);//przesłanie pojedyńczego znaku z ciągu do wyświetlacza
		i++;//przejście do kolejnego znaku z ciągu
	}
}

//Funkcja zapisująca 1 bajt danych z wykorzystaniem interfejsu I2C
//rs - ustawienie linii RS,
//data - bajt do przesłania,
//is8BitsMode - dla 0 tryb pracy 4-bitowy, dla 1 8-bitowy
void LCD_send(uint8_t rs, uint8_t data, uint8_t is8BitsMode){
	uint8_t bdata[7];//Tablica z danymi do przesłania
	uint8_t send_data;//Docelowa dana do przesłania
	uint8_t data_count;//Liczba bitów do przesłania
	if(rs==1){
		bdata[0] = 0x1;//RS=1
		bdata[1] = 0x5;//E=1,RS=1
		send_data = (data&0xf0);//przygotowanie danych do dalszej edycji
		bdata[2] = (send_data|0x5);//E=1, RS=1
		bdata[3] = (send_data|0x1);//E=0, RS=1
		data_count = 4;//Ustawienie liczby bitów do przesłania
	}
	else if(rs==0){
		bdata[0] = 0x0;//RS=0
		bdata[1] = 0x4;//E=1, RS=0
		send_data = (data&0xf0);//przygotowanie danych do dalszej edycji
		bdata[2] = (send_data|0x4);//E=1, RS=0
		bdata[3] = (send_data|0x0);//E=0, RS=0
		data_count = 4;//Ustawienie liczby bitów do przesłania
	}
	if(is8BitsMode==1){//sprawdzenie czy podana zmienna jest 8-bitowa
		send_data = 0x0;
		if(rs==1){
			bdata[4] = 0x5;//E=1, RS=1
			send_data = (data&0x0f)<<4;//przygotowanie danych do dalszej edycji
			bdata[5] = (send_data|0x05);//E=1, RS=1
			bdata[6] = (send_data|0x01);//E=0, RS=1
			data_count = 7;//Ustawienie liczby bitów do przesłania
		}
		else if(rs==0){
			bdata[4] = 0x4;//E=1, RS=0
			send_data = (data&0x0f)<<4;//przygotowanie danych do dalszej edycji
			bdata[5] = (send_data|0x04);//E=1, RS=0
			bdata[6] = (send_data|0x00);//E=0, RS=0
			data_count = 7;//Ustawienie liczby bitów do przesłania
		}
	}//Transmisja danych zapisanych w tablicy bdata przy pomocy I2C
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)I2C_ADDRESS, bdata, data_count, 20);
	HAL_Delay(4);//Opóźnienie 4ms
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
