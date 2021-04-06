#ifndef _TRANSMIT_H
#define _TRANSMIT_H

#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal.h"
#define sense 5
#define RX_BUFF_LENGH 30////usart接受数据储存BUFFER长度
extern uint8_t dbus_meta_data0[RX_BUFF_LENGH]__attribute__((at(0x24006000)));
typedef struct
{
float target_speed[4];
	float pid_output_speed[4];
	float final_output_speed[4];
signed short int feedback_speed[4];
//	float finalt_speed[4];//5
	float final_sense;
	int Delta_speed;//?
	float Delta_speed_sense;//0.01，不要大于0.05
	float vx_sense;
	float vy_sense;
	float omega_sense;
}Wheel;
extern Wheel wheel;

struct KK
{
float a;
};
extern struct KK kk;

typedef struct { 
   struct     {       
   int16_t sw;
	 int16_t ch0;       
   int16_t ch1;       
   int16_t ch2;      
   int16_t ch3;      
   int8_t  s1; 
	 int8_t  s1_last; 
   int8_t  s2;
	 int8_t  s2_last; 
	 int16_t left_VR; //拨杆变换值
	 int16_t left_HR;
	 int16_t right_VR;
	 int16_t right_HR;
	 int16_t left_RR;//拨轮变化值
    }rc; //比官方代码扩展了几个，why???似乎是后来操作要用，为了整齐美观就写到一起了
 
   struct     {      
   int16_t x;       
   int16_t y;       
   int16_t z;         
   uint8_t press_l;     
   uint8_t press_r; 
	 uint8_t press_l_flag;     
   uint8_t press_r_flag; 
	 }mouse; 
 
    struct  {     
    uint16_t v;  
    }keyboard;
	uint8_t  key[18];
	uint8_t  keyflag[18];	
	uint32_t key_filter_cnt[18];
 }RC_Ctl_t; 
extern  RC_Ctl_t RC_Ctl;


#define  max_output 10000
#define min_output -10000
#define key_W_flag        keyflag[0]
#define key_A_flag        keyflag[1]
#define key_S_flag        keyflag[2]
#define key_D_flag        keyflag[3]
#define key_shift_flag    keyflag[4]
#define key_ctrl_flag     keyflag[5]
#define key_Q_flag        keyflag[6]
#define key_E_flag        keyflag[7]
#define key_V_flag        keyflag[8]
#define key_F_flag        keyflag[9]
#define key_G_flag        keyflag[10]
#define key_C_flag        keyflag[11]
#define key_R_flag        keyflag[12]
#define key_B_flag        keyflag[13]
#define key_Z_flag        keyflag[14]
#define key_X_flag        keyflag[15]


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void USART1_receive_deal(uint8_t *dbus_meta_data);
void Keyboard_control_logic(void);
 void PC_keybroad_filter(void);
 void PID_wheels(void);
void USART1_IDLE_CALLBACK();
void Wheel_tergat_original(void);
float Wheel_Pid_Control(float Feedback,int16_t target,uint8_t who);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
uint8_t FDCAN2_send_msg_chassis(float * number);
void Wheel_Control();
uint8_t Remote_Online_Check();




#endif
