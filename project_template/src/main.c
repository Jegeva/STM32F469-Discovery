/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
//#include "stm32f469_discovery.h"
#include "stm32f4xx_hal_conf.h" // again, added because ST didn't put it here ?
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32469i_discovery.h"
#include "stm32469i_discovery_lcd.h"
#include "stm32469i_discovery_sdram.h"
#include "stm32469i_discovery_qspi.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

/* Private define ------------------------------------------------------------*/
#define VSYNC           1  
#define VBP             1 
#define VFP             1
#define VACT            480
#define HSYNC           1
#define HBP             1
#define HFP             1
#define HACT            800

#define LAYER0_ADDRESS  (LCD_FB_START_ADDRESS)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimingDelay;

extern DSI_HandleTypeDef hdsi_eval;
DSI_PLLInitTypeDef dsiPllInit;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN       = 360;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2; /* to have ck_pll = 360 MHz / 2 = 180 MHz */
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  RCC_OscInitStruct.PLL.PLLR       = 6; /* to have ck_plllcd = 60 MHz : replace DPHY PLL clock when this PLL is Off in ULPM mode */
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
 

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
  

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType =  (RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_HCLK   |
                                 RCC_CLOCKTYPE_PCLK1  |
                                 RCC_CLOCKTYPE_PCLK2);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

  /* Set nominal timing configurations for STM32F469xx */
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1; /* f(hclk) = 180 MHz / 1 = 180 MHz        */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   /* f(pclk1) = f(hclk)/4  = 180/4 = 45 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   /* f(pclk2) = f(hclk)/2  = 90 MHz         */

  /* Set Flash latency parameters (wait states) depending on CPU clock = ck_sys = 180 MHz */
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}


void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

DSI_HandleTypeDef DSI_Handle;


void f_error()
{
    HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_6);
    while(1);
    
}

struct 
{
    uint8_t border ;
    
} GUI_Conf;

    

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
    SystemClock_Config();
    uint8_t  lcd_status = LCD_OK;
    HAL_MspInit();
    HAL_Init();
    BSP_SDRAM_Init();
    //  uint32_t i;
    int i;
    
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;
   
  
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed =  GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL; //GPIO_PULLDOWN;//GPIO_PULLUP;// 

    GUI_Conf.border=8;
    

    GPIO_InitStructure.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOG,&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOD,&GPIO_InitStructure);
    GPIO_InitStructure.Pin = 	GPIO_PIN_4;
    HAL_GPIO_Init(GPIOD,&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOK,&GPIO_InitStructure);


    BSP_LCD_Reset(); BSP_LCD_MspInit();
    //lcd_status = BSP_LCD_InitEx(LCD_ORIENTATION_PORTRAIT);
    //lcd_status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE);
    lcd_status = BSP_LCD_Init();

   
    
    
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);   
    BSP_LCD_SelectLayer(0);
    BSP_LCD_DisplayOn();
    BSP_LCD_SetTransparency(0,0xff);
    if(lcd_status!=LCD_OK)
	f_error();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
 
   
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(0, 0,BSP_LCD_GetXSize(),BSP_LCD_GetYSize());
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(GUI_Conf.border, GUI_Conf.border,BSP_LCD_GetXSize()-2*GUI_Conf.border,BSP_LCD_GetYSize()-2*GUI_Conf.border);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(LCD_COLOR_TRANSPARENT);
    BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"  Project template");

  
    
    BSP_LCD_SetTextColor(0xffff0000);
    BSP_LCD_FillRect(340,200 ,40,40);
    BSP_LCD_SetTextColor(0xff00ff00);
    BSP_LCD_FillRect(380,200 ,40,40);
    BSP_LCD_SetTextColor(0xff0000ff);
    BSP_LCD_FillRect(420,200 ,40,40);
    
  
    i=0;
    
 
    

    
    while(1){

	Delay(1000);
	switch(i%4){
	case 0:
	    HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_6);
	    break;
	    
	case 1: 
	    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_4);
	    break;
	case 2:
	    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_5);
	    break;
	case 3:
	    HAL_GPIO_TogglePin(GPIOK,GPIO_PIN_3);
	    break;
	}
	i++;
	if(! i%4)
	    i=0;
	
	
	
    }
    
    
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    { 
	TimingDelay--;
    }
    
}

void Delay(__IO uint32_t nTime)
{
   TimingDelay = nTime;
   while(TimingDelay != 0);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
