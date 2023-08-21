/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> 
#include "usart.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Vref 2400

#define Ke 1.4
//#define KpPLL 7
//#define KiPLL 1 
float KpPLL=7;
float KiPLL=1;

#define  fs 50000

#define  PI2 (float)6.28318531
#define  PI20 (float)62.831853
#define	 PI100 (float)314.1592654

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
typedef struct
{
	float in;
	float v0_s;
	float v1_s;
	float vAlpha;
	float vBeta;
	float vQtemp;
	float vQ;
	float vD;
	float phaseAngle;
} SOGIPLL;

static SOGIPLL Vout;
static SOGIPLL IL;

uint8_t tempData[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0x00, 0x00, 0x80, 0x7f};
	
int sinetable[1000]={75,226,377,528,678,829,980,1131,1281,1432,1582,1733,1883,2033,2184,2334,2484,
2634,2783,2933,3083,3232,3382,3531,3680,3829,3978,4126,4275,4423,4571,4719,4867,
5014,5162,5309,5456,5603,5749,5895,6042,6187,6333,6478,6623,6768,6913,7057,7201,
7345,7488,7631,7774,7917,8059,8201,8342,8483,8624,8765,8905,9045,9184,9324,9462,
9601,9739,9876,10014,10150,10287,10423,10559,10694,10829,10963,11097,11230,11363,11496,11628,
11760,11891,12022,12152,12282,12411,12540,12668,12796,12923,13050,13177,13302,13428,13552,13676,
13800,13923,14046,14168,14289,14410,14530,14650,14769,14888,15006,15123,15240,15356,15472,15587,
15701,15815,15928,16040,16152,16264,16374,16484,16593,16702,16810,16917,17024,17130,17235,17340,
17444,17547,17649,17751,17852,17953,18052,18151,18250,18347,18444,18540,18636,18730,18824,18917,
19010,19102,19192,19283,19372,19461,19548,19636,19722,19807,19892,19976,20059,20142,20223,20304,
20384,20463,20542,20619,20696,20772,20847,20921,20995,21068,21139,21210,21281,21350,21418,21486,
21553,21619,21684,21748,21811,21874,21935,21996,22056,22115,22173,22230,22287,22342,22397,22451,
22504,22555,22607,22657,22706,22754,22802,22849,22894,22939,22983,23026,23068,23109,23149,23189,
23227,23265,23301,23337,23372,23405,23438,23470,23501,23531,23561,23589,23616,23643,23668,23693,
23716,23739,23761,23781,23801,23820,23838,23855,23871,23886,23900,23914,23926,23937,23948,23957,
23966,23973,23980,23986,23990,23994,23997,23999,24000,24000,23999,23997,23994,23990,23986,23980,
23973,23966,23957,23948,23937,23926,23914,23900,23886,23871,23855,23838,23820,23801,23781,23761,
23739,23716,23693,23668,23643,23616,23589,23561,23531,23501,23470,23438,23405,23372,23337,23301,
23265,23227,23189,23149,23109,23068,23026,22983,22939,22894,22849,22802,22754,22706,22657,22607,
22555,22504,22451,22397,22342,22287,22230,22173,22115,22056,21996,21935,21874,21811,21748,21684,
21619,21553,21486,21418,21350,21281,21210,21139,21068,20995,20921,20847,20772,20696,20619,20542,
20463,20384,20304,20223,20142,20059,19976,19892,19807,19722,19636,19548,19461,19372,19283,19192,
19102,19010,18917,18824,18730,18636,18540,18444,18347,18250,18151,18052,17953,17852,17751,17649,
17547,17444,17340,17235,17130,17024,16917,16810,16702,16593,16484,16374,16264,16152,16040,15928,
15815,15701,15587,15472,15356,15240,15123,15006,14888,14769,14650,14530,14410,14289,14168,14046,
13923,13800,13676,13552,13428,13302,13177,13050,12923,12796,12668,12540,12411,12282,12152,12022,
11891,11760,11628,11496,11363,11230,11097,10963,10829,10694,10559,10423,10287,10150,10014,9876,
9739,9601,9462,9324,9184,9045,8905,8765,8624,8483,8342,8201,8059,7917,7774,7631,
7488,7345,7201,7057,6913,6768,6623,6478,6333,6187,6042,5895,5749,5603,5456,5309,
5162,5014,4867,4719,4571,4423,4275,4126,3978,3829,3680,3531,3382,3232,3083,2933,
2783,2634,2484,2334,2184,2033,1883,1733,1582,1432,1281,1131,980,829,678,528,
377,226,75,-75,-226,-377,-528,-678,-829,-980,-1131,-1281,-1432,-1582,-1733,-1883,
-2033,-2184,-2334,-2484,-2634,-2783,-2933,-3083,-3232,-3382,-3531,-3680,-3829,-3978,-4126,-4275,
-4423,-4571,-4719,-4867,-5014,-5162,-5309,-5456,-5603,-5749,-5895,-6042,-6187,-6333,-6478,-6623,
-6768,-6913,-7057,-7201,-7345,-7488,-7631,-7774,-7917,-8059,-8201,-8342,-8483,-8624,-8765,-8905,
-9045,-9184,-9324,-9462,-9601,-9739,-9876,-10014,-10150,-10287,-10423,-10559,-10694,-10829,-10963,-11097,
-11230,-11363,-11496,-11628,-11760,-11891,-12022,-12152,-12282,-12411,-12540,-12668,-12796,-12923,-13050,-13177,
-13302,-13428,-13552,-13676,-13800,-13923,-14046,-14168,-14289,-14410,-14530,-14650,-14769,-14888,-15006,-15123,
-15240,-15356,-15472,-15587,-15701,-15815,-15928,-16040,-16152,-16264,-16374,-16484,-16593,-16702,-16810,-16917,
-17024,-17130,-17235,-17340,-17444,-17547,-17649,-17751,-17852,-17953,-18052,-18151,-18250,-18347,-18444,-18540,
-18636,-18730,-18824,-18917,-19010,-19102,-19192,-19283,-19372,-19461,-19548,-19636,-19722,-19807,-19892,-19976,
-20059,-20142,-20223,-20304,-20384,-20463,-20542,-20619,-20696,-20772,-20847,-20921,-20995,-21068,-21139,-21210,
-21281,-21350,-21418,-21486,-21553,-21619,-21684,-21748,-21811,-21874,-21935,-21996,-22056,-22115,-22173,-22230,
-22287,-22342,-22397,-22451,-22504,-22555,-22607,-22657,-22706,-22754,-22802,-22849,-22894,-22939,-22983,-23026,
-23068,-23109,-23149,-23189,-23227,-23265,-23301,-23337,-23372,-23405,-23438,-23470,-23501,-23531,-23561,-23589,
-23616,-23643,-23668,-23693,-23716,-23739,-23761,-23781,-23801,-23820,-23838,-23855,-23871,-23886,-23900,-23914,
-23926,-23937,-23948,-23957,-23966,-23973,-23980,-23986,-23990,-23994,-23997,-23999,-24000,-24000,-23999,-23997,
-23994,-23990,-23986,-23980,-23973,-23966,-23957,-23948,-23937,-23926,-23914,-23900,-23886,-23871,-23855,-23838,
-23820,-23801,-23781,-23761,-23739,-23716,-23693,-23668,-23643,-23616,-23589,-23561,-23531,-23501,-23470,-23438,
-23405,-23372,-23337,-23301,-23265,-23227,-23189,-23149,-23109,-23068,-23026,-22983,-22939,-22894,-22849,-22802,
-22754,-22706,-22657,-22607,-22555,-22504,-22451,-22397,-22342,-22287,-22230,-22173,-22115,-22056,-21996,-21935,
-21874,-21811,-21748,-21684,-21619,-21553,-21486,-21418,-21350,-21281,-21210,-21139,-21068,-20995,-20921,-20847,
-20772,-20696,-20619,-20542,-20463,-20384,-20304,-20223,-20142,-20059,-19976,-19892,-19807,-19722,-19636,-19548,
-19461,-19372,-19283,-19192,-19102,-19010,-18917,-18824,-18730,-18636,-18540,-18444,-18347,-18250,-18151,-18052,
-17953,-17852,-17751,-17649,-17547,-17444,-17340,-17235,-17130,-17024,-16917,-16810,-16702,-16593,-16484,-16374,
-16264,-16152,-16040,-15928,-15815,-15701,-15587,-15472,-15356,-15240,-15123,-15006,-14888,-14769,-14650,-14530,
-14410,-14289,-14168,-14046,-13923,-13800,-13676,-13552,-13428,-13302,-13177,-13050,-12923,-12796,-12668,-12540,
-12411,-12282,-12152,-12022,-11891,-11760,-11628,-11496,-11363,-11230,-11097,-10963,-10829,-10694,-10559,-10423,
-10287,-10150,-10014,-9876,-9739,-9601,-9462,-9324,-9184,-9045,-8905,-8765,-8624,-8483,-8342,-8201,
-8059,-7917,-7774,-7631,-7488,-7345,-7201,-7057,-6913,-6768,-6623,-6478,-6333,-6187,-6042,-5895,
-5749,-5603,-5456,-5309,-5162,-5014,-4867,-4719,-4571,-4423,-4275,-4126,-3978,-3829,-3680,-3531,
-3382,-3232,-3083,-2933,-2783,-2634,-2484,-2334,-2184,-2033,-1883,-1733,-1582,-1432,-1281,-1131,
-980,-829,-678,-528,-377,-226,-75,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SOGI(SOGIPLL *Vo)
{
//	v0_s = ((Va-vAlpha)*Ke-vBeta)*OmegaN ;
	Vo->v0_s = (Vo->in- Vo->vAlpha - Vo->vBeta) ;	//Ke=1
	Vo->vAlpha += Vo->v0_s/fs*PI100;
	Vo->v1_s += Vo->vAlpha/fs;
	Vo->vBeta = Vo->v1_s*PI100;
}

void PLL(SOGIPLL *Vo)
{
	float dvQ=0,dvQ1=0;
	float tempCOS,tempSIN;

	tempCOS = cosf(Vo->phaseAngle);
	tempSIN = sinf(Vo->phaseAngle);
	
	Vo->vQ = tempCOS*Vo->vBeta - tempSIN*Vo->vAlpha;
	Vo->vD = tempCOS*Vo->vAlpha + tempSIN*Vo->vBeta;

//	dvQ = vQ-0;//error
	dvQ = Vo->vQ;//error
	Vo->vQtemp += (dvQ-dvQ1) * KpPLL + dvQ * KiPLL ;
  dvQ1 = dvQ;
    
	if(Vo->vQtemp > PI20)	Vo->vQtemp = PI20;
	else if(Vo->vQtemp < -PI20)	Vo->vQtemp = -PI20;  

	Vo->phaseAngle +=  (PI100 + Vo->vQtemp)/fs;
		
	if(Vo->phaseAngle >= PI2) Vo->phaseAngle -= PI2;
	else if(Vo->phaseAngle <=0) Vo->phaseAngle -= PI2;  
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern HRTIM_HandleTypeDef hhrtim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
 static int c=0;
	c++;
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	static int i;
	static uint16_t Vduty=0,Vtrig=0;	
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
	Vout.in = ADC_ConvertedValue[4]-ADC_ConvertedValue[0];  //VO+ - VO-
//	Vout.in = ADC_ConvertedValue[0];  //VO+ - VO-
//	Vout.in -= 2048;	
	SOGI(&Vout);
	PLL(&Vout);	
	
	IL.in = ADC_ConvertedValue[3];  //VO+ - VO-
	IL.in -= 2048;	
	SOGI(&IL);
	PLL(&IL);
//	
	if(i++>(1000-1))i=0;
//	Vout.in=sinetable[i];
	Vduty=(uint16_t)(sinetable[i]+27200);
	Vtrig=(uint16_t)((sinetable[i]>>1)+13600);
	
	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = Vduty;
	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP1xR = Vduty;
	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = Vtrig;
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	static float temp[4];
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	temp[0]=IL.phaseAngle;
	temp[1]=Vout.phaseAngle;
	temp[2]=IL.in;
	temp[3]=Vout.in;
	memcpy(tempData, (uint8_t*)&temp, sizeof(temp));
	HAL_UART_Transmit_DMA(&huart3, tempData, 20);
	
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles HRTIM master timer global interrupt.
  */
void HRTIM1_Master_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_Master_IRQn 0 */

  /* USER CODE END HRTIM1_Master_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_MASTER);
  /* USER CODE BEGIN HRTIM1_Master_IRQn 1 */

  /* USER CODE END HRTIM1_Master_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
