#include "key.h"

MYKEY_TYPE  MYKEY1 = {
	{KEY_1_GPIO_Port,KEY_1_Pin},
	{0,0,0,0,0,0,0,0, },
};
 
MYKEY_TYPE  MYKEY2 = {
	{KEY_2_GPIO_Port,KEY_2_Pin},
	{0,0,0,0,0,0,0,0, },
};
	
MYKEY_TYPE  MYKEY3 = {
 	{KEY_3_GPIO_Port,KEY_3_Pin},
	{0,0,0,0,0,0,0,0, },
};
 
MYKEY_TYPE  MYKEY4 = {
 	{KEY_4_GPIO_Port,KEY_4_Pin},
	{0,0,0,0,0,0,0,0, },
};

void  act0 ( KEY_StateMachine *   pStateMachine);
void  act1 ( KEY_StateMachine *   pStateMachine);
void  act2 ( KEY_StateMachine *   pStateMachine);
void  act3 ( KEY_StateMachine *   pStateMachine);

ActionFunType transition_table[KEY_STATES][KEY_STATES]={
{&act3,&act0,},
{&act2,&act1,},
};

void KEY_GPIO_Init( MYKEY_TYPE* mykey )
{
  mykey->Machine.State_last=0;
  mykey->Machine.usKeyDtcNum=0;
  mykey->Machine.usTime=0;
  mykey->Machine.usTS=0;
}
int addCount=0;

//释放状态被按下
void  act0 ( KEY_StateMachine *  pStateMachine)
{
   pStateMachine->usTime=0;
}
//按下状态
void  act1 ( KEY_StateMachine *   pStateMachine)
{
   pStateMachine->usTime++;
	 if( pStateMachine->usTime >=  KEY_PUSH_1000MS  )
	 {
		pStateMachine->usKeyDtcNum=0;
		 addCount++;
	  (*pStateMachine->LONG_Deal)(); //长按键处理
	 }		 
}
//按下状态被释放
void  act2 ( KEY_StateMachine *   pStateMachine)
{
   pStateMachine->usTS=0;
	 if ( ( pStateMachine->usTime >KEY_PUSH_0020MS) && ( pStateMachine->usTime <KEY_PUSH_0400MS) )
	 {
	   pStateMachine->usKeyDtcNum++;
	 }
}
//释放状态
void  act3 ( KEY_StateMachine *   pStateMachine)
{
	 if( pStateMachine->usTS < KEY_PUSH_0400MS )
	 {
	     pStateMachine->usTS++;
	 }else
	 {
	   if(pStateMachine->usKeyDtcNum == 1)
		 {
		   (*pStateMachine->SHOURT_Deal)(); //短按键处理
		 } 
		 if(pStateMachine->usKeyDtcNum >= 2)
		 {
		    (*pStateMachine->DOUBLE_Deal)(); //双击按键处理
		 }
     pStateMachine->usKeyDtcNum=0;
		 pStateMachine->usTS=0;		 		 
	 }
}



void KEY_GPIO_DEAL(MYKEY_TYPE* mykey)
{
	 KEY_State  State_now= KEY_STATES_RELEASE ;
	 if(HAL_GPIO_ReadPin(mykey->keyset.GPIOx,mykey->keyset.Pin)==KEY_PUSH)
		 State_now= KEY_STATES_PUSH;     //读取当前按键状态
	
   (*transition_table[mykey->Machine.State_last][State_now] )( (KEY_StateMachine *) (&(mykey->Machine))  );  //根据状态 *FUNC【last】【now】 执行对应的处理函数
	
	  mykey->Machine.State_last=State_now;  //保存当前的按键状态
}


//按键1回调函数
void callback10()
{
	;
}
void callback11()
{
	;
}
void callback12()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
}
void callback13()
{
	;
}


//按键2回调函数
void callback20()
{
	;
}
void callback21()
{
	;
}
void callback22()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
}
void callback23()
{
	;
}



//按键3回调函数
void callback30()
{
	;
}
void callback31()
{
	;
}
void callback32()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
}
void callback33()
{
	;
}


//按键4回调函数
void callback40()
{
	;
}
void callback41()
{
	;
}
void callback42()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
}
void callback43()
{
	;
}

void KEY_Init()
{
	MYKEY1.Machine.NO_PRO_Deal=callback10;
	MYKEY1.Machine.SHOURT_Deal=callback11;
	MYKEY1.Machine.DOUBLE_Deal=callback12;
	MYKEY1.Machine.LONG_Deal  =callback13;
	KEY_GPIO_Init(&MYKEY1);	
	MYKEY2.Machine.NO_PRO_Deal=callback20;
	MYKEY2.Machine.SHOURT_Deal=callback21;
	MYKEY2.Machine.DOUBLE_Deal=callback22;
	MYKEY2.Machine.LONG_Deal  =callback23;
	KEY_GPIO_Init(&MYKEY2);	
	MYKEY3.Machine.NO_PRO_Deal=callback30;
	MYKEY3.Machine.SHOURT_Deal=callback31;
	MYKEY3.Machine.DOUBLE_Deal=callback32;
	MYKEY3.Machine.LONG_Deal  =callback33;
	KEY_GPIO_Init(&MYKEY3);	
	MYKEY4.Machine.NO_PRO_Deal=callback40;
	MYKEY4.Machine.SHOURT_Deal=callback41;
	MYKEY4.Machine.DOUBLE_Deal=callback42;
	MYKEY4.Machine.LONG_Deal  =callback43;
	KEY_GPIO_Init(&MYKEY4);	
}

void Botton_Process()
{	
	KEY_GPIO_DEAL(&MYKEY1);
	KEY_GPIO_DEAL(&MYKEY2);
	KEY_GPIO_DEAL(&MYKEY3);
	KEY_GPIO_DEAL(&MYKEY4);
}

