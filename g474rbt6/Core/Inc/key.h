#ifndef __KEY_H
#define __KEY_H	 

#include "main.h"
#include "hrtim.h"

#define KEY_1_Pin GPIO_PIN_9
#define KEY_1_GPIO_Port GPIOB
#define KEY_2_Pin GPIO_PIN_13
#define KEY_2_GPIO_Port GPIOC
#define KEY_3_Pin GPIO_PIN_14
#define KEY_3_GPIO_Port GPIOC
#define KEY_4_Pin GPIO_PIN_15
#define KEY_4_GPIO_Port GPIOC

#define varRate 1

typedef enum
{
  KEY_UP   = 0,
  KEY_DOWN = 1,
}KEYState_TypeDef;

#define KEY_PUSH               0
#define KEY_PUSH_0020MS        20-1     //定义按键判断时间   20X1ms=20MS
#define KEY_PUSH_0400MS        300-1    //定义按键判断时间   400X1ms=400MS
#define KEY_PUSH_1000MS        1000-1   //定义按键判断时间   200X5ms=1000MS


typedef int KEY_State;
typedef int KEY_Condition;

//状态
#define KEY_STATES            2
#define KEY_STATES_RELEASE    0
#define KEY_STATES_PUSH       1



typedef void  (*KeyDeal)( void);

typedef struct
{
  KEY_State         State_last;       //记录当前的状态
	uint32_t          usTime;        //记录按下的时间
  uint32_t          usTS;          //记录释放的时间
  uint32_t          usKeyDtcNum;	 //记录脉冲数
	KeyDeal           NO_PRO_Deal;
	KeyDeal           SHOURT_Deal;
	KeyDeal           LONG_Deal;
	KeyDeal           DOUBLE_Deal;
	
}KEY_StateMachine,*KEY_pStateMachine;
 
typedef void  (*ActionFunType)( KEY_pStateMachine  pStateMachine);
typedef struct
{
	GPIO_TypeDef*   GPIOx;
	uint16_t        Pin;
}MYKEY_SET_TYPE;
typedef struct
{
	MYKEY_SET_TYPE    keyset;
	KEY_StateMachine  Machine;
}MYKEY_TYPE;

extern unsigned int angUpdateFlag;
	
void KEY_Init(void);
void Botton_Process(void);

#endif
