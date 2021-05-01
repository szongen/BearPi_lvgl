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
#include "dma.h"
#include "fatfs.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_spi2_drv.h"
#include "bsp_w25q64.h"
#include "mbedtls/aes.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_demo_widgets.h"
#include "lv_demo_benchmark.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t dat[11] = "mculover666";
uint8_t read_buf[11] = {0};

////密钥
//unsigned char key[16] = "520199112301234";
////明文 
//unsigned char plain[16] = "DaoBanMoJieYJW";
////密文
//unsigned char cipher[16] = {0};
////解密后的明文
//unsigned char plain_decrypt[16] = {0};

//mbedtls_aes_context aes;



uint8_t work[_MAX_SS] ;
void Mount_Fatfs(void)
{
    retUSER = f_mount(&USERFatFS, USERPath, 1);

    if(retUSER != FR_OK)
    {
        if(retUSER == FR_NO_FILESYSTEM)
        {
            printf("f_mount 没有文件系统,开始格式化spi-flash\r\n");
            retUSER = f_mkfs(USERPath, FM_ANY, 0, work, sizeof(work));

            if(retUSER != FR_OK)
            {
                printf("f_mkfs 格式化失败，err = %d\r\n", retUSER);
            }
            else
            {
                printf("格式化成功，开始重新挂载spi-flash\r\n");
                retUSER = f_mount(&USERFatFS, USERPath, 1);

                if(retUSER != FR_OK)
                {
                    printf("f_mount 发生错误，err = %d\r\n", retUSER);
                }
                else
                {
                    printf("spi-flash文件系统挂载成功\r\n");
                }
            }
        }
        else
        {
            printf("f_mount 发生其他错误，err = %d\r\n", retUSER);
        }
    }
    else
        printf("spi-flash文件系统挂载成功\r\n");
}

/*获取盘内存*/
uint8_t  f_GetTotal_Free(uint8_t *drv, uint32_t *total, uint32_t *free)
{
    FATFS *fs1;
    uint8_t res;
    DWORD fre_clust = 0, fre_sect = 0, tot_sect = 0;
    res = f_getfree((const TCHAR*)drv, &fre_clust, &fs1);//得到磁盘信息及空闲簇数量

    if(res == 0)
    {
        tot_sect = (fs1->n_fatent - 2) * fs1->csize; //得到总扇区数
        fre_sect = fre_clust * fs1->csize;        //得到空闲扇区数
        #if _MAX_SS!=512                                  //扇区大小不是512字节,则转换为512字节
        tot_sect *= fs1->ssize / 512;
        fre_sect *= fs1->ssize / 512;
        #endif
        *total = tot_sect >> 1; //单位为KB
        *free = fre_sect >> 1; //单位为KB
    }
    return res;
}

static void btn_event_cb(lv_obj_t * btn, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, NULL);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}

void lvgl_first_demo_start(void)
{
    lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_set_event_cb(btn, btn_event_cb);                 /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn, NULL);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/


	lv_obj_t * label1 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(label1, "Hello world!"); 
	lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_obj_align(btn, label1, LV_ALIGN_OUT_TOP_MID, 0, -10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint16_t device_id;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    
    uint8_t res;
    uint32_t Total = 0; //读取FLASH总容量
    uint32_t Free = 0; //读取FLASH剩余容量
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_QUADSPI_Init();
  MX_FATFS_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  Mount_Fatfs();
  f_GetTotal_Free((uint8_t*)"0:",&Total,&Free);//获取SD卡总容量和剩余容量
  printf("当前Fatfs总容量:%dKB==>%dMB?剩余容量:%dKB==>%dMB\n",Total,Total/1024,Free,Free/1024);

    LCD_Init();
	lv_init();
	lv_port_disp_init();
	lv_demo_widgets();
//    mbedtls_aes_init(&aes);	//初始化
    printf("Test W25QXX...\r\n");
    device_id = W25QXX_ReadID();
    printf("device_id = 0x%X\r\n\r\n", device_id);

//    /* 为了验证，首先读取要写入地址处的数据 */
//    printf("-------- read data before write -----------\r\n");
//    W25QXX_Read(read_buf, 5, 11);
//    printf("read date is %s\r\n", (char*)read_buf);

//    /* 擦除该扇区 */
//    printf("-------- erase sector 0 -----------\r\n");
//    W25QXX_Erase_Sector(0);

//    /* 写数据 */
//    printf("-------- write data -----------\r\n");
//    W25QXX_Page_Program(dat, 5, 11);

//    /* 再次读数据 */
//    printf("-------- read data after write -----------\r\n");
//    W25QXX_Read(read_buf, 5, 11);
//    printf("read date is %s\r\n", (char*)read_buf);

//  lv_obj_t * label;
//  lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
//  lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, 0);
//  label = lv_label_create(btn1, NULL);
//  lv_label_set_text(label, "Button");

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		lv_task_handler();
//        mbedtls_aes_setkey_enc(&aes, key, 128);	//设置加密密钥
//		mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plain, cipher);//ECB加密
//		printf("%s\r\n",plain);
//		mbedtls_aes_setkey_dec(&aes, key, 128);//设置解密密钥
//		mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, cipher, plain_decrypt);//ECB解密
//        printf("%s\r\n",plain_decrypt);
//		HAL_Delay(500);        
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if(htim == &htim7)
	 {
	    lv_tick_inc(1);//lvgl 的 1ms 心跳
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
