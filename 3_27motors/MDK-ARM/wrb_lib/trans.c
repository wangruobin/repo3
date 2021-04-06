#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal.h"
#include "transmit.h"

typedef struct Motors_wheels
{
double tergat_speed[4];
double fabs_max_speed;
float dead_grap;
double feedback_speed[4];
double final_output_speed[4];
float pid_p_param;
float pid_i_param;
float pid_d_param;


};


void USART1_receive_deal()
{
uint8_t correct_num=0;//_用来计数的变量	，初始化为0，把18个字节分成6份，每比较一份数据就++
       		correct_num=0;
				if(/*如果通道一的值小于1684且大于364*/( (dbus_meta_data[0] | (dbus_meta_data[1] << 8)) & 0x07ff) <=1684   &&/*并且*/  ((dbus_meta_data[0]        | (dbus_meta_data[1] << 8)/*为真*/)                            & 0x07ff) >=364)
					correct_num++;//1
				if((((dbus_meta_data[1] >> 3) | (dbus_meta_data[2] << 5)) & 0x07ff)<=1684                            	&&/*并且*/  (((dbus_meta_data[1] >> 3) | (dbus_meta_data[2] << 5))                            & 0x07ff) >=364)
					correct_num++;
				if((((dbus_meta_data[2] >> 6) | (dbus_meta_data[3] << 2) |(dbus_meta_data[4] << 10)) & 0x07ff)<=1684 	&&/*并且*/  (((dbus_meta_data[2] >> 6) | (dbus_meta_data[3] << 2) |(dbus_meta_data[4] << 10)) & 0x07ff) >=364)
					correct_num++;//3
				if((RC_Ctl.rc.ch3 = ((dbus_meta_data[4] >> 1) | (dbus_meta_data[5] << 7)) & 0x07ff)<=1684 &&/*并且*/  (RC_Ctl.rc.ch3 = ((dbus_meta_data[4] >> 1) | (dbus_meta_data[5] << 7)) & 0x07ff)>=364)
					correct_num++;
				if((((dbus_meta_data[5] >> 4)& 0x000C) >> 2)==1 || (((dbus_meta_data[5] >> 4)& 0x000C) >> 2)==2 || (((dbus_meta_data[5] >> 4)& 0x000C) >> 2)==3)
					correct_num++;//5
				if(((dbus_meta_data[5] >> 4)& 0x0003)==1 || ((dbus_meta_data[5] >> 4)& 0x0003)==2 || ((dbus_meta_data[5] >> 4)& 0x0003)==3)
					correct_num++;
				
				if(correct_num==6)	//数据完成性验证 ，开始拆分数据给RC_Ctl结构体，RC_Ctl结构体就是按照ch0~3,s1,s2,mouse x,y,z,鼠标left,right,key的结构编的， 很容易对照着手册看。
				{//这部分看我的画图笔记
				RC_Ctl.rc.ch0 = (dbus_meta_data[0]| (dbus_meta_data[1] << 8)) & 0x07ff; //!< Channel 0   高8位与低3位
				RC_Ctl.rc.ch1 = ((dbus_meta_data[1] >> 3) | (dbus_meta_data[2] << 5)) & 0x07ff; //!< Channel 1     高5位与低6位
				RC_Ctl.rc.ch2 = ((dbus_meta_data[2] >> 6) | (dbus_meta_data[3] << 2) |(dbus_meta_data[4] << 10)) & 0x07ff; //!< Channel 2
				RC_Ctl.rc.ch3 = ((dbus_meta_data[4] >> 1) | (dbus_meta_data[5] << 7)) & 0x07ff; //!< Channel 3
				RC_Ctl.rc.s1 = ((dbus_meta_data[5] >> 4)& 0x000C) >> 2; //!< Switch left
				RC_Ctl.rc.s2 = ((dbus_meta_data[5] >> 4)& 0x0003); //!< Switch right
        RC_Ctl.rc.sw=(uint16_t)(dbus_meta_data[16]|(dbus_meta_data[17]<<8))&0x7ff; 
					//ch0为右摇杆左右horizontal，ch1为右摇杆前后vertical
				RC_Ctl.rc.right_HR=(RC_Ctl.rc.ch0-1024);//1024为1684和364的中间零点值？？
				RC_Ctl.rc.right_VR=(RC_Ctl.rc.ch1-1024);//为什么遥控器4各通道的值正好对应电机的电流值范围？
				RC_Ctl.rc.left_HR=(RC_Ctl.rc.ch2-1024);//是专门为了饭方便操控而配套设计的吗？
			  RC_Ctl.rc.left_VR=(RC_Ctl.rc.ch3-1024);
					
					/***********按键映射*************/
	      RC_Ctl.mouse.x = dbus_meta_data[6]  | (dbus_meta_data[7] << 8);                    //!< Mouse X axis横坐标   
        RC_Ctl.mouse.y = dbus_meta_data[8]  | (dbus_meta_data[9] << 8);                    //!< Mouse Y axis     
        RC_Ctl.mouse.z = dbus_meta_data[10] | (dbus_meta_data[11] << 8);                  //!< Mouse Z axis   
        RC_Ctl.mouse.press_l = dbus_meta_data[12];                                        //!< Mouse Left Is Press ?   
        RC_Ctl.mouse.press_r = dbus_meta_data[13];                                        //!< Mouse Right Is Press ? 
				
			 if(RC_Ctl.mouse.x>25000)   RC_Ctl.mouse.x=25000;//其实范围是+-32767，这里是考虑电机实际功率上限，做一个限幅。
			 if(RC_Ctl.mouse.x<-25000)  RC_Ctl.mouse.x=-25000;	
			 if(RC_Ctl.mouse.y>25000)   RC_Ctl.mouse.y=25000;
			 if(RC_Ctl.mouse.y<-25000)  RC_Ctl.mouse.y=-25000;
		//不知道键盘值有啥用？？？	
		//不是遥控吗？咋还有mouse,keyboard?----答：因为电脑用usb线和遥控器连在一起时就可以用电脑控制了。没连时当然只能用那4各通道和s1s2拨杆
			  RC_Ctl.keyboard.v = dbus_meta_data[14]| (dbus_meta_data[15] << 8);  	//!< KeyBoard value   

			
				RC_Ctl.key_W=dbus_meta_data[14]&0x01;	//uint8_t key[18]对应18个按键
				RC_Ctl.key_S=(dbus_meta_data[14]>>1)&0x01;					
				RC_Ctl.key_A=(dbus_meta_data[14]>>2)&0x01;
				RC_Ctl.key_D=(dbus_meta_data[14]>>3)&0x01;					
				RC_Ctl.key_B=(dbus_meta_data[15]>>7)&0x01;
				RC_Ctl.key_V=(dbus_meta_data[15]>>6)&0x01;				
				RC_Ctl.key_C=(dbus_meta_data[15]>>5)&0x01;
				RC_Ctl.key_X=(dbus_meta_data[15]>>4)&0x01;					
				RC_Ctl.key_Z=(dbus_meta_data[15]>>3)&0x01;					
				RC_Ctl.key_G=(dbus_meta_data[15]>>2)&0x01;			
				RC_Ctl.key_F=(dbus_meta_data[15]>>1)&0x01;
				RC_Ctl.key_R=(dbus_meta_data[15])&0x01;													
				RC_Ctl.key_E=(dbus_meta_data[14]>>7)&0x01;
				RC_Ctl.key_Q=(dbus_meta_data[14]>>6)&0x01;
				RC_Ctl.key_ctrl=(dbus_meta_data[14]>>5)&0x01;
				RC_Ctl.key_shift=(dbus_meta_data[14]>>4)&0x01;
					
				//防抖
				PC_keybroad_filter();

}


//***********************
void PID_wheels()
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//1,将usart1返回数据重新结算

//2,pid
//3,限幅
//4,send
	

}

