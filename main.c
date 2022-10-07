/*!
    \file    main.c
    \brief   USART printf demo

    \version 2014-12-26, V1.0.0, demo for GD32F10x
    \version 2017-06-30, V2.0.0, demo for GD32F10x
    \version 2021-04-30, V2.1.0, demo for GD32F10x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f10x.h"
#include "gd32f10x_it.h"
#include "gd32f105r_start.h"
#include "systick.h"
#include <stdio.h>

#define JianSuQu 200  //1° 高低加速度
#define FWJianSuQu 250  //1° 方位加速度
#define ShanshaoJianSuQu 300  //1° 扇扫加速度
#define SiQu     20   //0.1° 死区范围
#define SHANSHAOSiQu     50   //0.1° 扇扫速度
#define ZhuliuZuiXiaoSuDu 100 //驻留最小速度
#define ShanshaoZuiXiaoSuDu 1800 //扇扫最小速度
#define JiaSuDu_CanShu  100 //a=30°/s2 加速度常数
//static int32_t  ttt,FWttt = 0;//启动加速变量
double fwdata = 0, gddata = 0;//方位数据，高低数据
uint32_t time;//区分第一次上电
uint32_t encoder_shujuFW,encoder_shujuGD;//编码器数据（方位/高低）
uint8_t test_data[6] = {0x11,0x22,0X33,0x06,00,0x0F};	//测试数据	
extern uint16_t dataflag;


typedef int32_t position_t;
typedef int16_t speed_t;

typedef enum
{
	      TingZhi = 0,
	      ZhuLiu,
	      fwshansao,
	      gdshansao,
        Guiling,
	      SheZhiLingWei,
	      ZhengZhuan,
	      FanZhuan = 7,
        SWJ_MingLing_BUTT=0
} SWJMingLingtypedef;
typedef enum
{
        INSIDE,
        OUT_OF_START,
        OUT_OF_STOP,
        WEIZHI_STATUS_BUTT,
	      FWINSIDE,
        FWOUT_OF_START,
        FWOUT_OF_STOP,
       FWWEIZHI_STATUS_BUTT
} WeiZhiStatusTypeDef;
//typedef enum
//{
//    StandFrame,
//    ExtendedFrame
//} Frametype;

extern SWJMingLingtypedef tSWJMingLing;//程序命令
WeiZhiStatusTypeDef weizhistatus = WEIZHI_STATUS_BUTT;//高低位置状态
WeiZhiStatusTypeDef fwweizhistatus = FWWEIZHI_STATUS_BUTT;//方位位置状态

extern uint16_t SWJcount;
extern uint8_t SWJdata1[10],R_data_CAN0[8],R_data_CAN1[8],can1_flag,can2_flag,zhujizijian_flag,fenjizijian_flag,normal_flag,special_flag,test_flag;//设置变量
extern uint8_t HY_flag,UTCtime_flag,Datatime_flag,long_flag,ceju_flag,data_fail,work_flag,MY_flag,dingxiang_flag,reback_flag,weizhi_flag_CAN0,weizhi_flag_CAN1,weizhi_flag,Controller_transfer_Data_flag;//设置变量
extern uint8_t SWJdata[70],ZIJIAN[8],Daowei_flag,FW_motor,GD_motor,communication_error,zhuji_return,daowei_flag,daowei_stop,lingwei_flag,tansuo_flag,guiling_flag;//设置变量
extern uint8_t  MY_return,HY_return,Time_return,boshu_flag,zijian_return,work_return,reback_flag,data_ok,data_recive_ok;
extern uint8_t time_flag_1,zhujizijian_flag_1,work_flag_1,dingxiang_flag_1,MY_flag_1,HY_flag_1,UTCtime_flag_1,Datatime_flag_1,dataflag_1,weizhi_flag_1,daowei_flag_1,long_flag_1,ceju_flag_1,fenjizijian_flag_1;


position_t zhuliuFW,zhuliuGD,mapandangqian;//驻留方位，驻留高低，码盘当前值
position_t fwshansao_start, fwshansao_stop,gdshansao_start, gdshansao_stop;//方位扇扫启用，方位扇扫停止，高低扇扫启用，高低扇扫停止
volatile position_t FWmapan0,FWmapanxiuzheng,FWmapanyuanshi,GDmapan0,GDmapanxiuzheng,GDmapanyuanshi,FW_last,GD_last;//设置变量
speed_t shezhifwsudu,shezhigdsudu,aaaa;//设置方位速度，设置高低速度，备用
speed_t fwshansaosudu, gdshansaosudu, fwsousuosudu, fwzhuliusudu, gdzhuliusudu,zidongduizhunsudu;//方位扇扫速度?，高低扇扫速度?，方位搜索速度，方位驻留速度，高低驻留速度，自动对准速度
uint16_t fangwei_cha=0x0000,fuyang_cha=0x2AAA;	//方位0和俯仰值60 原始零位与设置零位差值

void usart_send_char(uint32_t usart_periph,uint8_t data);
void UART_Write(uint8_t *pData, uint32_t dataLen);
void sendSWJdata(uint8_t len,uint8_t type);		//CAN0
void sendSWJdata_1(uint8_t len,uint8_t type);//发送至主机数据CAN1
void sendQDQdata(uint8_t len,uint8_t type);

void sendagain(uint8_t flag);
speed_t GDZhuLiuJianSu ( speed_t sudu, position_t mubiao, position_t dangqian );//高低驻留减速
speed_t FWZhuLiuJianSu ( speed_t FWsudu, position_t FWmubiao, position_t FWdangqian );//方位驻留减速
speed_t GDZhuLiuFuc ( position_t zhuliuweizhi, position_t dangqianweizhi, speed_t sd );
speed_t FWZhuLiuFuc ( position_t FWzhuliuweizhi, position_t FWdangqianweizhi, speed_t FWsd );

void sendzhujidata(void);//向驱动器发送数据
void send_to_can(void);
void COMxSendData( uint32_t USARTX, uint8_t* p, uint8_t num );

//void Controller_transfer_Data(void); //????????
//void CANsenddata(void);
//uint8_t Can_Send_Msg(uint32_t can_periph, Frametype datatype, uint32_t Id, uint8_t* msg);

position_t xianwei(position_t weizhi);//限位

/* select can */
#define CAN0_USED
//#define CAN1_USED

#ifdef  CAN0_USED
    #define CANX CAN0
#else 
    #define CANX CAN1
#endif

/* select CAN baudrate */
/* 1MBps */
//#define CAN_BAUDRATE  1000
/* 500kBps */
 #define CAN_BAUDRATE  500 
/* 250kBps */
// #define CAN_BAUDRATE  250 
/* 125kBps */
/* #define CAN_BAUDRATE  125 */
/* 100kBps */ 
/* #define CAN_BAUDRATE  100 */
/* 50kBps */ 
/* #define CAN_BAUDRATE  50 */
/* 20kBps */ 
//#define CAN_BAUDRATE  20 

can_parameter_struct can_init_parameter;
can_filter_parameter_struct can_filter_parameter;


extern FlagStatus receive_flag,usart_rx_to_can_flag;
extern can_receive_message_struct receive_message;
can_trasnmit_message_struct transmit_message;
extern uint8_t data[9];
extern uint8_t data_last;
uint8_t data_1[9]={0};
uint8_t tx_data[9],usart1_receive[9];
int j=0;
uint32_t sys_clk_freq,ahb_clk_freq,apb1_clk_freq,apb2_clk_freq;
can_parameter_struct can_parameter;
can_filter_parameter_struct can_filter;

static void usart1_config(void);
static void uart3_config(void);
//void can_config(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter);
void can0_networking_init(void);
void can1_networking_init(void);
void nvic_config(void);
void can_gpio_config(void);


int fputc(int ch, FILE *f);

void usart_sendByte(uint32_t USARTX, uint8_t ch);
void usart_sendString(uint32_t USARTX, char *str);

uint16_t a,b,c,weizhi_fangwei_temp,weizhi_fuyang_temp;


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* initialize the LEDs */
   // test_status_led_init();
    
    /* configure systick */
    systick_config();
    
    /* flash the LEDs for 1 time */
		//  led_flash(1);
    
    
		/* configure GPIO */
    can_gpio_config();

		/* configure NVIC */
    nvic_config();
		/* configure USART */
    usart1_config();
		uart3_config();
		/* configure CAN */
		can0_networking_init();
		can1_networking_init();
//		can_config(can_init_parameter, can_filter_parameter);
		/*CAN interrupt enavle*/
		can_interrupt_enable(CAN0, CAN_INT_RFNE0);
		can_interrupt_enable(CAN1, CAN_INT_RFNE1);

//    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//		gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,  GPIO_PIN_1);


//		gpio_bit_reset(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
//		gpio_bit_set(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

    /* configure TAMPER key */
 //   gd_eval_key_init(KEY_TAMPER, KEY_MODE_GPIO);
    
    /* output a message on hyperterminal using printf function */
//    printf("\r\n USART printf example: please press the Tamper key \r\n");
    
    /* wait for completion of USART transmission */
//    while(RESET == usart_flag_get(USART1 ,USART_FLAG_TC)){
//    }

//		can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &can_parameter);
/* initialize transmit message */
//    transmit_message.tx_sfid = 0x321;
//    transmit_message.tx_efid = 0x321;
//    transmit_message.tx_ft = CAN_FT_DATA;
//    transmit_message.tx_ff = CAN_FF_STANDARD;
//    transmit_message.tx_dlen = 2;
//		can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &can_filter);
    //printf("\r\nplease press the Tamper key to transmit data!\r\n");
		/*时钟频率验证*/
			sys_clk_freq=rcu_clock_freq_get(CK_SYS);
			ahb_clk_freq=rcu_clock_freq_get(CK_AHB);
			apb1_clk_freq=rcu_clock_freq_get(CK_APB1);
			apb2_clk_freq=rcu_clock_freq_get(CK_APB2);

    while(1){

				sendzhujidata();//向主机发送数据
//			if((SWJdata[0] == 0x1e) && (SWJdata[1] == 0x1e)&& (SWJdata[2] != 0x00))
//			{
//				Controller_transfer_Data();
//			}
				
//				if(time == 0)//第一次上电执行
//			{
//				time = 1;//使变量为1，下个循环不在进入
//				COMxSendData(USART1,test_data,1);
//				delay_1ms(100);  //等待100ms
////				zijian_return = 1;//自检状态
////				sendSWJdata(10,0x13); //自检
////				dataflag = 1;//接收指令
////				tSWJMingLing=Guiling;
//			}
//					usart_data_transmit(UART3,0xab);
//				delay_1ms(1000);


		//				send_to_can();	//USART1接收到的数据发送给CAN
}
}

/*********************************向天线发送数据********************************************/

void sendzhujidata(void)//
{
	if(long_flag == 1)//经纬度
	{
		sendSWJdata(11,0x03);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(11,0x03);//重新发送
		}
		else long_flag = 0;//重新赋值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(11,0x03);//重新发送
		}
    long_flag = 0;//重新赋值
	}
	else if(ceju_flag == 1)//测距
	{
		sendSWJdata(10,0x04);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(10,0x04);//重新发送
		}
		else ceju_flag = 0;//重新赋值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(10,0x04);//重新发送
		}
    ceju_flag = 0;//重新赋值 
	}
	else if(UTCtime_flag == 1)//UTC时间转发
	{
		sendSWJdata(10,0x01);//转发数据
		UTCtime_flag = 0;//重新赋值
	}
	else if(Datatime_flag == 1)//Data时间转发
	{
		sendSWJdata(10,0x02);//转发数据
		Datatime_flag = 0;//重新赋值
	}
	else if(zhujizijian_flag == 1)//主机自检
	{
		sendSWJdata(6,0x05);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(6,0x05);//重新发送
		}
		else work_flag = 0;//重新赋初值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(6,0x05);//重新发送
		}
		zhujizijian_flag = 0;//重新赋初值
		delay_1ms(100);//等待100ms
		fenjizijian_flag = 1;//触发分机自检
		sendagain(zijian_return);//数据确认
	}
	else if(data_recive_ok == 1)//数据有无效判断
	{
		sendSWJdata(7,0);//转发数据
		delay_1ms(100);//等待100ms
	  if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(7,0);//重新发送
		}
		else data_recive_ok = 0;//重新赋初值
		delay_1ms(100);;//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(7,0);//重新发送
		}
		data_recive_ok = 0;//重新赋初值
	}
	else if(work_flag == 1)//工作模式
	{
		sendSWJdata(10,0x06);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(10,0x06);//重新发送
		}
		else work_flag = 0;//重新赋初值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata(10,0x06);//重新发送
		}
		work_flag = 0;//重新赋初值
	}
    	else if(dingxiang_flag == 1)//定向询问
	{
			sendSWJdata(10,0x07);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata(10,0x07);//重新发送
			}
			else dingxiang_flag = 0;//重新赋初值
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata(10,0x07);//重新发送
			}
			dingxiang_flag = 0;//重新赋初值
			sendagain(boshu_flag);//数据确认
	}
	else if(MY_flag == 1)//MY切换
	{
			sendSWJdata(10,0x08);//转发数据
		  delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata(10,0x08);//重新发送
			}
			else work_flag = 0;//重新赋初值
			
		  delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata(10,0x08);//重新发送
			}
      MY_flag = 0;//赋初值
		  sendagain(MY_return);//数据确认
	}
	else if(HY_flag == 1)//HY切换
	{
			sendSWJdata(6,0x09);//转发数据
		  delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			 sendSWJdata(6,0x09);//转发数据
			}
			else work_flag = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			 sendSWJdata(6,0x09);//转发数据
			}
      HY_flag = 0;//赋初值
		  sendagain(HY_return);
	}
	else if(fenjizijian_flag == 1)//自检状态上报
	{
		  sendSWJdata(10,0x13);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		    sendSWJdata(10,0x13);//转发数据
			}
			else fenjizijian_flag = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		    sendSWJdata(10,0x13);//转发数据
			}
		  fenjizijian_flag = 0;//赋初值
	}
	else if(weizhi_flag == 1)//方位俯仰角度信息
	{
			sendSWJdata(10,0x11);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendSWJdata(10,0x11);//转发数据
			}
			else weizhi_flag = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata(10,0x11);//转发数据
			}
		  weizhi_flag = 0;//赋初值
	}	
	else if(daowei_flag == 1)//到位信息
	{
		if(daowei_stop == 1)//到位后发送
		{
			sendSWJdata(10,0x12);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendSWJdata(10,0x12);//转发数据
			}
			else daowei_stop = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendSWJdata(10,0x12);//转发数据
			}
			daowei_stop = 0;//赋初值
		}
	}
	else if(data_recive_ok == 1)//数据有无效判断
	{			
		sendSWJdata(7,0x00);//转发数据
	}
	else if(Controller_transfer_Data_flag==1)
	{
		Controller_transfer_Data();
	}
	/********CAN1*******/
	if(long_flag_1 == 1)//经纬度
	{
		sendSWJdata_1(11,0x03);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(11,0x03);//重新发送
		}
		else long_flag_1 = 0;//重新赋值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(11,0x03);//重新发送
		}
    long_flag_1 = 0;//重新赋值
	}
	else if(ceju_flag_1 == 1)//测距
	{
		sendSWJdata_1(10,0x04);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(10,0x04);//重新发送
		}
		else ceju_flag_1 = 0;//重新赋值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(10,0x04);//重新发送
		}
    ceju_flag_1 = 0;//重新赋值 
	}
	
	else if(UTCtime_flag_1 == 1)//UTC时间转发
	{
		sendSWJdata_1(10,0x01);//转发数据
		UTCtime_flag_1 = 0;//重新赋值
	}
	else if(Datatime_flag_1 == 1)//Data时间转发
	{
		sendSWJdata_1(10,0x02);//转发数据
		Datatime_flag_1 = 0;//重新赋值
	}
	else if(zhujizijian_flag_1 == 1)//主机自检
	{
		sendSWJdata_1(6,0x05);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(6,0x05);//重新发送
		}
		else work_flag_1 = 0;//重新赋初值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(6,0x05);//重新发送
		}
		zhujizijian_flag_1 = 0;//重新赋初值
		delay_1ms(100);//等待100ms
		fenjizijian_flag_1 = 1;//触发分机自检
		sendagain(zijian_return);//数据确认
	}
	else if(data_recive_ok == 1)//数据有无效判断
	{
		sendSWJdata_1(7,0);//转发数据
		delay_1ms(100);//等待100ms
	  if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(7,0);//重新发送
		}
		else data_recive_ok = 0;//重新赋初值
		delay_1ms(100);;//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(7,0);//重新发送
		}
		data_recive_ok = 0;//重新赋初值
	}
	else if(work_flag_1 == 1)//工作模式
	{
		sendSWJdata_1(10,0x06);//转发数据
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(10,0x06);//重新发送
		}
		else work_flag_1 = 0;//重新赋初值
		delay_1ms(100);//等待100ms
		if(data_ok == 0)//判断是否传输成功
		{
		  sendSWJdata_1(10,0x06);//重新发送
		}
		work_flag_1 = 0;//重新赋初值
	}
    	else if(dingxiang_flag_1 == 1)//定向询问
	{
			sendSWJdata_1(10,0x07);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata_1(10,0x07);//重新发送
			}
			else dingxiang_flag_1 = 0;//重新赋初值
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata_1(10,0x07);//重新发送
			}
			dingxiang_flag_1 = 0;//重新赋初值
			sendagain(boshu_flag);//数据确认
	}
	else if(MY_flag_1 == 1)//MY切换
	{
			sendSWJdata_1(10,0x08);//转发数据
		  delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata_1(10,0x08);//重新发送
			}
			else work_flag_1 = 0;//重新赋初值
			
		  delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata_1(10,0x08);//重新发送
			}
      MY_flag_1 = 0;//赋初值
		  sendagain(MY_return);//数据确认
	}
	else if(HY_flag_1 == 1)//HY切换
	{
			sendSWJdata_1(6,0x09);//转发数据
		  delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			 sendSWJdata_1(6,0x09);//转发数据
			}
			else work_flag_1 = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			 sendSWJdata_1(6,0x09);//转发数据
			}
      HY_flag_1 = 0;//赋初值
		  sendagain(HY_return);
	}
	else if(fenjizijian_flag_1 == 1)//自检状态上报
	{
		  sendSWJdata_1(10,0x13);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		    sendSWJdata_1(10,0x13);//转发数据
			}
			else fenjizijian_flag_1 = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		    sendSWJdata_1(10,0x13);//转发数据
			}
		  fenjizijian_flag_1 = 0;//赋初值
	}
	
	else if(weizhi_flag_1 == 1)//方位俯仰角度信息
	{
			sendSWJdata_1(10,0x11);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendSWJdata_1(10,0x11);//转发数据
			}
			else weizhi_flag_1 = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
			  sendSWJdata_1(10,0x11);//转发数据
			}
		  weizhi_flag_1 = 0;//赋初值
	}	
	else if(daowei_flag_1 == 1)//到位信息
	{
		if(daowei_stop == 1)//到位后发送
		{
			sendSWJdata_1(10,0x12);//转发数据
			delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendSWJdata_1(10,0x12);//转发数据
			}
			else daowei_stop = 0;//赋初值
			
	  	delay_1ms(100);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendSWJdata_1(10,0x12);//转发数据
			}
			daowei_stop = 0;//赋初值
		}
	}
	else if(data_recive_ok == 1)//数据有无效判断
	{			
		sendSWJdata_1(7,0x00);//转发数据
	}
//	else if(Controller_transfer_Data_flag==1)
//	{
//		Controller_transfer_Data();
//	}
	
	
	/******************/
	else if(weizhi_flag_CAN0 == 1)	//方位俯仰角度信息.判断来自CAN0的数据位标志
		{
			sendQDQdata(9,0x05);//转发数据
			delay_1ms(10);//等待100ms
			if(data_ok == 0)//判断是否传输成功,成功置1
			{
		  	sendQDQdata(9,0x05);//转发数据
			}
			
				weizhi_flag_CAN0=0;
		}
	else if(weizhi_flag_CAN1 == 1)	//方位俯仰角度信息.判断来自CAN1的数据位标志
		{
			sendQDQdata(9,0x06);//转发数据
			delay_1ms(10);//等待100ms
			if(data_ok == 0)//判断是否传输成功
			{
		  	sendQDQdata(9,0x06);//转发数据
			}
			
				weizhi_flag_CAN1=0;
			
//	  	delay_1ms(100);//等待100ms
			
		}
		else if(lingwei_flag==1)				//设置零位
		{
			fangwei_cha=((uint16_t )data[2]<<8)|data[3];
			fuyang_cha =((uint16_t )data[5]<<8)|data[6];
			lingwei_flag=0;
		}	
		else if(tansuo_flag==1)					//探索模式
		{
			sendQDQdata(9,0x12);//转发数据
			delay_1ms(10);//等待100ms
			if(data_ok == 0)//判断是否传输成功,成功置1
			{
		  	sendQDQdata(9,0x12);//转发数据
			}
			
				tansuo_flag=0;
		}	
		else if(guiling_flag==1)					//角度归零
		{
			fangwei_cha=0X0000;
			fuyang_cha=0x2AAA;
			guiling_flag=0;
		}
		
}



/****************************发送至天线主机*数据*CAN0*RS422**UART3**************************/
void sendSWJdata(uint8_t len,uint8_t type)//发送至主机数据
{
	int i;
  uint8_t senddata[11];//声明
	senddata[0] = 0x1e;//数据第一位赋值
	senddata[1] = 0x1e;//数据第二位赋值
	senddata[2] = type;//数据第三位赋值
	senddata[3] = len&0x00ff;//数据第四位赋值
	senddata[4] = (len&0xff00)>>8;//数据第五位赋值
  senddata[len-1] = 0;//当前长度类型数据最后一位赋值
  if(len == 7)//数据确认帧
	{ 
    if((data_ok == 1)&&(communication_error == 0))//||(boshu_flag == 1))	//数据有无效判断
		{			
		   senddata[5] = 0;	//数据第6位赋值，有效
		}
    else senddata[5] = 0x80;	//数据第6位赋值，无效
	}
	else if(len == 10)//数据确认帧
	{
		if((type == 0x01)&&(UTCtime_flag == 1))//UTC时间信息
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if((type == 0x02)&&(Datatime_flag == 1))//data时间信息
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if((type == 0x04)&&(ceju_flag == 1))//目标距离
		{
			//数据赋值转发*
		    senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x06)//工作状态设置
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
	  else if(type == 0x07)//定向询问
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x08)//MY切换
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x11)//方位俯仰角度信息
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x12)//转动状态上报
		{
			if((zhuliuFW == ((uint16_t)(data[2]<<8)|data[3]))&&(zhuliuGD ==((uint16_t)(data[2]<<8)|data[3])))//转动到位
			{
				senddata[5] = 0x55;//数据赋值转发
			  daowei_flag = 0;//到位标志复位
				daowei_stop = 1;//停止
			}
			else                                            //未到位
			{
				senddata[5] = 0xAA;//数据赋值转发
				daowei_stop = 0;//继续
			}
			//数据赋值转发
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
		}
//		else if(type == 0x13)//自检状态上报   上电自检 命令自检
//		{
//			if(fwzhuliusudu != 0)//判断方位速度是否不为0
//			{
//				if(FW_motor == 1)//判断电机是否运行
//				{
//		  	  ZIJIAN[4] = 0x00;//方位电机正常
//				}
//				else ZIJIAN[4] = 0x10;//方位电机异常
//			}
//			if(gdzhuliusudu != 0)//判断高低速度是否不为0
//			{
//				if(GD_motor == 1)//判断电机是否运行
//				{
//		  	   ZIJIAN[3] = 0x00;//高低电机正常
//				}
//				else ZIJIAN[3] = 0x08;//高低电机异常
//			}
//      senddata[6] = ZIJIAN[0]+ZIJIAN[1]+ZIJIAN[2]+ZIJIAN[3]+ZIJIAN[4];//整合检测数据
//			if(senddata[6] == 0x00)//伺服分机正常
//			{
//				senddata[5] = 0x55;//数据赋值转发
//			}
//      else if(senddata[6]!= 0x00)//伺服分机异常
//      {
//			  senddata[5] = 0xAA;//数据赋值转发
//			}
//			//数据赋值转发*
//			senddata[7] = R_data_CAN0[6];
//			senddata[8] = R_data_CAN0[7];
//            //* 
//		}
		
	}
	else if(len == 11)//目标经纬度
	{
		if(long_flag == 1)//确认数据
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN0[3];
			senddata[6] = R_data_CAN0[4];
			senddata[7] = R_data_CAN0[5];
			senddata[8] = R_data_CAN0[6];
			senddata[9] = R_data_CAN0[7];
			//*
		}			
	}	
	for(i=0;i<len-1;i++)//异或运算
	{
		senddata[len-1] ^= senddata[i];//异或运算
	}
	
  COMxSendData(UART3,senddata,len);//422传输数据
}

/****************************发送至天线主机*数据CAN1**RS422**UART3**************************/
void sendSWJdata_1(uint8_t len,uint8_t type)//发送至主机数据
{
	int i;
  uint8_t senddata[11];//声明
	senddata[0] = 0x1e;//数据第一位赋值
	senddata[1] = 0x1e;//数据第二位赋值
	senddata[2] = type;//数据第三位赋值
	senddata[3] = len&0x00ff;//数据第四位赋值
	senddata[4] = (len&0xff00)>>8;//数据第五位赋值
  senddata[len-1] = 0;//当前长度类型数据最后一位赋值
  if(len == 7)//数据确认帧
	{ 
    if((data_ok == 1)&&(communication_error == 0))//||(boshu_flag == 1))	//数据有无效判断
		{			
		   senddata[5] = 0;	//数据第6位赋值，有效
		}
    else senddata[5] = 0x80;	//数据第6位赋值，无效
	}
	else if(len == 10)//数据确认帧
	{
		if((type == 0x01)&&(UTCtime_flag_1 == 1))//UTC时间信息
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if((type == 0x02)&&(Datatime_flag_1 == 1))//data时间信息
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if((type == 0x04)&&(ceju_flag_1 == 1))//目标距离
		{
			//数据赋值转发*
		    senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x06)//工作状态设置
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
	  else if(type == 0x07)//定向询问
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x08)//MY切换
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x11)//方位俯仰角度信息
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x12)//转动状态上报
		{
			if((zhuliuFW == ((uint16_t)(data[2]<<8)|data[3]))&&(zhuliuGD ==((uint16_t)(data[2]<<8)|data[3])))//转动到位
			{
				senddata[5] = 0x55;//数据赋值转发
			  daowei_flag_1 = 0;//到位标志复位
				daowei_stop = 1;//停止
			}
			else                                            //未到位
			{
				senddata[5] = 0xAA;//数据赋值转发
				daowei_stop = 0;//继续
			}
			//数据赋值转发
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
		}
//		else if(type == 0x13)//自检状态上报   上电自检 命令自检
//		{
//			if(fwzhuliusudu != 0)//判断方位速度是否不为0
//			{
//				if(FW_motor == 1)//判断电机是否运行
//				{
//		  	  ZIJIAN[4] = 0x00;//方位电机正常
//				}
//				else ZIJIAN[4] = 0x10;//方位电机异常
//			}
//			if(gdzhuliusudu != 0)//判断高低速度是否不为0
//			{
//				if(GD_motor == 1)//判断电机是否运行
//				{
//		  	   ZIJIAN[3] = 0x00;//高低电机正常
//				}
//				else ZIJIAN[3] = 0x08;//高低电机异常
//			}
//      senddata[6] = ZIJIAN[0]+ZIJIAN[1]+ZIJIAN[2]+ZIJIAN[3]+ZIJIAN[4];//整合检测数据
//			if(senddata[6] == 0x00)//伺服分机正常
//			{
//				senddata[5] = 0x55;//数据赋值转发
//			}
//      else if(senddata[6]!= 0x00)//伺服分机异常
//      {
//			  senddata[5] = 0xAA;//数据赋值转发
//			}
//			//数据赋值转发*
//			senddata[7] = R_data_CAN1[6];
//			senddata[8] = R_data_CAN1[7];
//            //* 
//		}
		
	}
	else if(len == 11)//目标经纬度
	{
		if(long_flag_1 == 1)//确认数据
		{
			//数据赋值转发*
			senddata[5] = R_data_CAN1[3];
			senddata[6] = R_data_CAN1[4];
			senddata[7] = R_data_CAN1[5];
			senddata[8] = R_data_CAN1[6];
			senddata[9] = R_data_CAN1[7];
			//*
		}			
	}	
	for(i=0;i<len-1;i++)//异或运算
	{
		senddata[len-1] ^= senddata[i];//异或运算
	}
	
  COMxSendData(UART3,senddata,len);//422传输数据
}



/********************************发送至平台计算机数据*CAN*************************************/
//void CANsenddata(void)
//{
//	uint8_t cansendata[8];//定义
//	
////		if((zhuji_return == 1)||(boshu_flag == 1))//主机返回信息
////		{
//			cansendata[0] = 0x01;//数据第1位赋值
//			cansendata[3] = 0;//数据第4位赋值
//			cansendata[4] = SWJdata1[5];//数据第5位赋值
//			cansendata[5] = SWJdata1[6];//数据第6位赋值
//			cansendata[6] = SWJdata1[7];//数据第7位赋值
//			cansendata[7] = SWJdata1[8];//数据第8位赋值
//	
//	
////  Can_Send_Msg ( CAN_1, StandFrame, 1891, cansendata );
//	
//		  if(reback_flag == 1)//控制反馈
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0xf0;//数据第3位赋值

//				reback_flag = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		  else if(zijian_return == 1)//设备状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x40;//数据第3位赋值
//				zijian_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(work_return == 1)//工作状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x41;//数据第3位赋值
//				work_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}	
//		 else if(MY_return == 1)//MY状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x42;//数据第3位赋值
//				MY_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(HY_return == 1)//HY状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x43;//数据第3位赋值
//				HY_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(Time_return == 1)//校时状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x44;//数据第3位赋值
//				Time_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(boshu_flag == 1)//波束范围
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x45;//数据第3位赋值
//				boshu_flag = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}			
//}

void sendagain(uint8_t flag)//数据确认
{
	uint8_t return_flag;//定义
	return_flag = flag;//赋值
    if(data_ok == 1)//判断是否传输成功
	 {
			 if(return_flag == 0)//确认成功单位收到相应的回复信息
			 {
				communication_error = 1;
			  zhuji_return = 0;
			 }
			 else //确认成功且收到相应的回复信息
			 {
				 zhuji_return = 1;
				 communication_error = 0;
		   }
	 }
}


///***********************************电机控制****************************************/	
//speed_t GDZhuLiuFuc ( position_t zhuliuweizhi, position_t dangqianweizhi, speed_t sd )
//{
//	speed_t p_sd = sd;//定义

//	if ( ( (zhuliuweizhi - dangqianweizhi)   >  18000 ) \
//	     || ( (zhuliuweizhi < dangqianweizhi) && (zhuliuweizhi - dangqianweizhi) > -18000 ) ) //最优路径判断
//	{
//		p_sd = -sd;//换向
//	}

//	return GDZhuLiuJianSu ( p_sd, zhuliuweizhi, dangqianweizhi);//判断减速区与执行
//}

//speed_t FWZhuLiuFuc ( position_t FWzhuliuweizhi, position_t FWdangqianweizhi, speed_t FWsd )
//{
//	speed_t FW_sd = FWsd;

//	if ( ( (FWzhuliuweizhi - FWdangqianweizhi)   >  18000 ) \
//	     || ( (FWzhuliuweizhi < FWdangqianweizhi) && (FWzhuliuweizhi - FWdangqianweizhi) > -18000 ) ) //最优路径判断
//	{
//		FW_sd = -FWsd;//换向
//	}

//	return FWZhuLiuJianSu ( FW_sd, FWzhuliuweizhi, FWdangqianweizhi);//判断减速区与执行
//}

//speed_t GDZhuLiuJianSu ( speed_t sudu, position_t mubiao, position_t dangqian )//高低驻留减速
//{

//	speed_t s = 0;//定义
//	speed_t biansu=0,ZuiXiaoSuDu;//定义
//		
//	 ZuiXiaoSuDu = ZhuliuZuiXiaoSuDu;//赋初值

//	if ( sudu == 0 )//判断高低速度是否为0
//	{
//		return 0;//跳出
//	}

//	if ( (abs ( mubiao - dangqian ) <= JianSuQu )&& (abs ( mubiao - dangqian ) >= SiQu )) //判断是否处于减速区内，且不在死区
//	{

//		s = mubiao - dangqian;//计算差值

//		if ( abs ( s ) < ZuiXiaoSuDu )//判断差值是否小于最小速度
//		{
//			s = ( abs ( s ) / s ) * ZuiXiaoSuDu;//使其等于最小速度，且计算速度方向
//		}
//		else if ( abs ( s ) > abs ( sudu ) )//判断是否大于设定速度
//		{
//			s = sudu;//使其等于设定速度
//		}

//	}
//	else if ( abs ( mubiao - dangqian ) < SiQu )//判断是否处于死区
//	{
//		s = 0;//设置速度为0
//		ttt = 0;//重置加速常数
//        //扇扫*
//		if (tSWJMingLing == gdshansao)//判断命令是否为扇扫
//		{
//			if (mubiao == gdshansao_start)//判断扇扫是否启用
//			{
//				weizhistatus = OUT_OF_START;//启用
//			}
//			else if (mubiao == gdshansao_stop)//判断扇扫是否停用
//			{
//				weizhistatus = OUT_OF_STOP;//停用
//			}
//		}
//		//*
//	}
//	else 
//	{
//		//启动加速
//		biansu = abs ( sudu ) / sudu * JiaSuDu_CanShu;//获取加速度值
//		s = ( ttt++ ) * biansu;//加速度比例常数自加
//		if ( abs ( s ) >= abs ( sudu ) )//判断是否大于设定速度
//		{
//			s = sudu;//使其等于设定速度
//		}
//		//*
//	}

//	return s;//返回速度值

//}

//speed_t FWZhuLiuJianSu ( speed_t FWsudu, position_t FWmubiao, position_t FWdangqian )//方位驻留减速
//{

//	speed_t FWs = 0;//声明方位速度变量
//	speed_t FWbiansu=0,FWZuiXiaoSuDu;//声明变量
//	int jiansuqu;//定义
//	int tingzhi = 0;//定义
//		tingzhi = SiQu;//赋值
//		FWZuiXiaoSuDu = ZhuliuZuiXiaoSuDu;//赋值
//		jiansuqu = FWJianSuQu;//赋值

//	if ( FWsudu == 0 )//判断方位速度是否为0
//	{
//		return 0;//跳出
//	}

//	if (( abs ( FWmubiao - FWdangqian ) <= jiansuqu) && abs (( FWmubiao - FWdangqian ) >= tingzhi) ) //判断是否处于减速区内，且不在死区
//	{

//		FWs = FWmubiao - FWdangqian;//计算差值

//		if ( abs ( FWs ) < FWZuiXiaoSuDu )//判断差值是否小于最小速度
//		{
//			FWs = ( abs ( FWs ) / FWs ) * FWZuiXiaoSuDu;//使其等于最小速度，且计算速度方向
//		}
//		else if ( abs ( FWs ) > abs ( FWsudu ) )//判断是否大于设定速度
//		{
//			FWs = FWsudu;//使其等于设定速度
//		}

//	}
//	else if ( abs ( FWmubiao - FWdangqian ) < tingzhi )//判断是否处于死区
//	{
//		FWs = 0;//设置速度为0
//		FWttt = 0;//重置加速常数
//		//扇扫*
//		if (tSWJMingLing == fwshansao)//判断命令是否为扇扫
//		{
//			if ((FWmubiao == fwshansao_start)||((FWmubiao == gdshansao_start))) //判断扇扫是否启用
//			{
//				fwweizhistatus = FWOUT_OF_START;//启用
//			}
//			else if ((FWmubiao == fwshansao_stop)||((FWmubiao == gdshansao_stop)))//判断扇扫是否停用
//			{
//				fwweizhistatus = FWOUT_OF_STOP;//停用
//			}
//		}
//		//*
//	}
//	else 
//	{
//		//启动加速
//			FWbiansu = abs ( FWsudu ) / FWsudu * JiaSuDu_CanShu;//获取加速度值
//		
//			FWs = ( FWttt++ ) * FWbiansu;//加速度比例常数自加

//			if ( abs ( FWs ) >= abs ( FWsudu ) )//判断是否大于设定速度
//			{
//				FWs = FWsudu;//使其等于设定速度
//			}
//			//*
//	}

//	return FWs;

//}



position_t xianwei(position_t weizhi)//限位
{
	  position_t geidingweizhi;//定义变量
	  geidingweizhi = weizhi;//赋值
		if((geidingweizhi >= 8000) && (geidingweizhi <18000))//判断高低限位角度
		{
			geidingweizhi = 8000;//高低限位赋值
		}
		else if((geidingweizhi >= 18000) && (geidingweizhi <=36000))//判断高低限位角度
		{
			geidingweizhi = 0;//高低限位赋值
		}
		
		return geidingweizhi;
}

/****************************发送至驱动器数据**RS422**USART1****************************/
void sendQDQdata(uint8_t len,uint8_t type)//发送至驱动器数据
{
	int i;
	uint8_t senddata_CAN0[9]={0};
//  uint8_t WeiZhiHuan[9]={0};//声明
//	int8_t SuDuHuan[9]={0};
	uint8_t senddata_CAN1[9]={0};
//	uint8_t sum;
	senddata_CAN0[0] = senddata_CAN1[0] = 0x55;//数据第一帧赋值
	senddata_CAN0[1] = senddata_CAN1[1] = 0xAA;//数据第二帧赋值
//	senddata_CAN0[2] = type;//数据第三位赋值
//	senddata_CAN0[3] = len&0x00ff;//数据第四位赋值
//	senddata_CAN0[4] = (len&0xff00)>>8;//数据第五位赋值
//	for(i=0;i<len-1;i++)
//  {
//		senddata_CAN0[8] += senddata_CAN0[i];//当前长度类型数据最后一位赋值
//	}

		/*else*/ if(type == 0x05)//方位俯仰角度信息，type==0x05,数据来自CAN0		
		{
			
			//数据赋值转发*
			/*CAN0*/	
			
			//转矩环
			if(0x00==R_data_CAN0[1])																				//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN0[4]	=	0x03;																				//fangwei_kongzhizi	第5个字节  1100 0000
				senddata_CAN0[7]	=	0x03;																				//fuyang_kongzhizi	第8个字节
				if(0x00==R_data_CAN0[2])	//方位正转
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x4E20)				//输入 方位速度>20000/0x4E20
					{
						senddata_CAN0[2]=(weizhi_fangwei_temp-0x4E20)>>8;
						senddata_CAN0[3]=(weizhi_fangwei_temp-0x4E20);
					}
				}
				else if(0x00==R_data_CAN0[3])	//俯仰正转
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x4E20)				//输入俯仰速度>20000/0x0x4E20
					{
						senddata_CAN0[5]=(weizhi_fuyang_temp-0x4E20)>>8;
						senddata_CAN0[6]=(weizhi_fuyang_temp-0x4E20);
					}
				}
				else if(0x01==R_data_CAN0[2])	//方位反转
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x4E20)				//输入 方位速度>-20000/<0xB1E0	65535-20000+1(uint16_t) == -20000(int16_t) 0xB1E0
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp-0x4E20)+0x0001)>>8);		//转换成有符号整形 就是 负数
						senddata_CAN0[3]=0xFFFF-((weizhi_fangwei_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN0[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN0[3])	//俯仰反转
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x4E20)				//输入俯仰速度>-20000/<0xB1E0		65535-20000+1(uint16_t) == -20000(int16_t)	0xB1E0
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp-0x4E20)+0x0001)>>8);	//转换成有符号整形 就是 负数
						senddata_CAN0[6]=0xFFFF-((weizhi_fuyang_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN0[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
//				else if(0x11==R_data_CAN0[2])	//方位不使能
//				{
//					senddata_CAN0[4] = 0x02;																				//fangwei_kongzhizi	第5个字节		0000 0011使能 0000 0010不使能
////					senddata_CAN0[2] = 0x00;
////					senddata_CAN0[3] = 0x00;
//				} 
//				else if(0x11==R_data_CAN0[2])	//俯仰不使能
//				{
//					senddata_CAN0[7] = 0x02;																				//fuyang_kongzhizi	第8个字节
////					senddata_CAN0[5] = 0x00;
////					senddata_CAN0[6] = 0x00;
//				}
			}
			//速度环
			else if(0x01==R_data_CAN0[1])																			//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN0[4]	=	0x13;																				//fangwei_kongzhizi	第5个字节		0001 0011使能 0001 0010不使能
				senddata_CAN0[7]	=	0x13;																				//fuyang_kongzhizi	第8个字节
				if(0x00==R_data_CAN0[2])	//fangwei正转
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x7530)				//输入 方位速度>30000/0x7530
					{
						senddata_CAN0[2]=(weizhi_fangwei_temp-0x7530)>>8;
						senddata_CAN0[3]=(weizhi_fangwei_temp-0x7530);
					}
				}
				else if(0x00==R_data_CAN0[3])	//fuyang正转
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x7530)				//输入俯仰速度>30000/0x7530
					{
						senddata_CAN0[5]=(weizhi_fuyang_temp-0x7530)>>8;
						senddata_CAN0[6]=(weizhi_fuyang_temp-0x7530);
					}
				}
				else if(0x01==R_data_CAN0[2])	//fangwei反转
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x7530)				//输入 方位速度>-30000/<0x7530
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp-0x7530)+0x0001)>>8);		//转换成有符号整形 就是 负数
						senddata_CAN0[3]=0xFFFF-((weizhi_fangwei_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN0[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN0[3])	//fuyang反转
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x7530)				//输入俯仰速度>-30000/<0x7530		65535-30000+1(uint16_t) == -30000(int16_t)
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp-0x7530)+0x0001)>>8);		//转换成有符号整形 就是 负数
						senddata_CAN0[6]=0xFFFF-((weizhi_fuyang_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN0[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
				
				
			}
			//带位置规划的位置环 
			else if(0x10==R_data_CAN0[1])																			//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN0[4]	=	0x23;																				//fangwei_kongzhizi	第5个字节		1100 1000
				senddata_CAN0[7]	=	0x23;																				//fuyang_kongzhizi	第8个字节
				
				senddata_CAN0[2] = R_data_CAN0[4]+(fangwei_cha>>8);							//fangwei_high			发送第3个字节
				senddata_CAN0[3] = R_data_CAN0[5]+(fangwei_cha);								//fangwei_low				第4个字节
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//输入方位角>36000/0x8CA0
				{
					senddata_CAN0[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN0[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN0[5] = R_data_CAN0[6]+(fuyang_cha>>8);							//fuyang_high				第6个字节
				senddata_CAN0[6] = R_data_CAN0[7]+(uint8_t)(fuyang_cha);				//fuyang_low				第7个字节
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
				if(weizhi_fuyang_temp>0x8CA0)				//输入俯仰角>36000/0x8CA0
				{
					senddata_CAN0[5]=(weizhi_fuyang_temp-0x8CA0)>>8;
					senddata_CAN0[6]=(weizhi_fuyang_temp-0x8CA0);
				}
			}
			//不带位置规划的位置环
			else if(0x11==R_data_CAN0[1])																			//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN0[4]	=	0x33;																				//fangwei_kongzhizi	第5个字节		1100 1100
				senddata_CAN0[7]	=	0x33;																				//fuyang_kongzhizi	第8个字节
				
				senddata_CAN0[2] = R_data_CAN0[4]+(fangwei_cha>>8);							//fangwei_high			发送第3个字节
				senddata_CAN0[3] = R_data_CAN0[5]+(fangwei_cha);								//fangwei_low				第4个字节
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//输入方位角>36000/0x8CA0
				{
					senddata_CAN0[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN0[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN0[5] = R_data_CAN0[6]+(fuyang_cha>>8);							//fuyang_high				第6个字节
				senddata_CAN0[6] = R_data_CAN0[7]+(uint8_t)(fuyang_cha);				//fuyang_low				第7个字节
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
				if(weizhi_fuyang_temp>0x1F40)				//输入俯仰角>8000/0x1F40
				{
					senddata_CAN0[5]=(weizhi_fuyang_temp-0x1F40)>>8;
					senddata_CAN0[6]=(weizhi_fuyang_temp-0x1F40);
				}
			}
			
			
//			if(senddata_CAN0[5]==0xff) senddata_CAN0[2]=0;									//如果角度=360，角度=0
//			if(senddata_CAN0[6]==0xff) senddata_CAN0[3]=0;
			
			if(0x11==R_data_CAN0[2])	//方位不使能
			{
				if(0x00==R_data_CAN0[1])//转矩环
				senddata_CAN0[4] = 0x02;																				//fangwei_kongzhizi	第5个字节		0000 0011使能 0000 0010不使能
				if(0x01==R_data_CAN0[1])//速度环
				senddata_CAN0[4] = 0x12;
				if(0x10==R_data_CAN0[1])//位置环 位置规划
				senddata_CAN0[4] = 0x22;
				if(0x11==R_data_CAN0[1])//位置环 不带位置规划
				senddata_CAN0[4] = 0x32;
				

//				senddata_CAN0[2] = 0x00;
//				senddata_CAN0[3] = 0x00;
			} 
			if(0x11==R_data_CAN0[3])	//俯仰不使能
			{
				if(0x00==R_data_CAN0[1]) //转矩环
				senddata_CAN0[7] = 0x02;																				//fangwei_kongzhizi	第5个字节		0000 0011使能 0000 0010不使能
				if(0x01==R_data_CAN0[1]) //速度环
				senddata_CAN0[7] = 0x12; 
				if(0x10==R_data_CAN0[1]) //位置环 位置规划
				senddata_CAN0[7] = 0x22; 
				if(0x11==R_data_CAN0[1]) //位置环 不带位置规划
				senddata_CAN0[7] = 0x32;
//				senddata_CAN0[5] = 0x00;
//				senddata_CAN0[6] = 0x00;
			}
			for(i=0;i<len-1;i++)
			{
				senddata_CAN0[8] += senddata_CAN0[i];//当前长度类型数据最后一位赋值
			}
			
		}
		
		else if(type == 0x06)//方位俯仰角度信息,type==0x06,数据来自CAN1				
		{
			/*CAN1*/
			//转矩环
			if(0x00==R_data_CAN1[1])																				//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN1[4]	=	0x03;																				//fangwei_kongzhizi	第5个字节  1100 0000
				senddata_CAN1[7]	=	0x03;																				//fuyang_kongzhizi	第8个字节
				if(0x00==R_data_CAN1[2])	//fangwei正转
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x4E20)				//输入 方位速度>20000/0x4E20
					{
						senddata_CAN1[2]=(weizhi_fangwei_temp-0x4E20)>>8;
						senddata_CAN1[3]=(weizhi_fangwei_temp-0x4E20);
					}
				}
				else if(0x00==R_data_CAN1[3])	//fuyang正转
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x4E20)				//输入俯仰速度>20000/0x0x4E20
					{
						senddata_CAN1[5]=(weizhi_fuyang_temp-0x4E20)>>8;
						senddata_CAN1[6]=(weizhi_fuyang_temp-0x4E20);
					}
				}
				else if(0x01==R_data_CAN1[2])	//fangwei反转
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x4E20)				//输入 方位速度>-20000/<0xB1E0	65535-20000+1(uint16_t) == -20000(int16_t) 0xB1E0
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp-0x4E20)+0x0001)>>8);		//转换成有符号整形 就是 负数
						senddata_CAN1[3]=0xFFFF-((weizhi_fangwei_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN1[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN1[3])	//fuyang反转
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x4E20)				//输入俯仰速度>-20000/<0xB1E0		65535-20000+1(uint16_t) == -20000(int16_t)	0xB1E0
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp-0x4E20)+0x0001)>>8);	//转换成有符号整形 就是 负数
						senddata_CAN1[6]=0xFFFF-((weizhi_fuyang_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN1[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
			}
			//速度环
			else if(0x01==R_data_CAN1[1])																			//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN1[4]	=	0x13;																				//fangwei_kongzhizi	第5个字节		1100 0100
				senddata_CAN1[7]	=	0x13;																				//fuyang_kongzhizi	第8个字节
				if(0x00==R_data_CAN1[2])	//fangwei正转
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x7530)				//输入 方位速度>30000/0x7530
					{
						senddata_CAN1[2]=(weizhi_fangwei_temp-0x7530)>>8;
						senddata_CAN1[3]=(weizhi_fangwei_temp-0x7530);
					}
				}
				else if(0x00==R_data_CAN1[3])	//fuyang正转
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x7530)				//输入俯仰速度>30000/0x7530
					{
						senddata_CAN1[5]=(weizhi_fuyang_temp-0x7530)>>8;
						senddata_CAN1[6]=(weizhi_fuyang_temp-0x7530);
					}
				}
				else if(0x01==R_data_CAN1[2])	//fangwei反转
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//方位速度高8位			发送第3个字节
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//方位速度低8位				第4个字节
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x7530)				//输入 方位速度>-30000/<0x7530
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp-0x7530)+0x0001)>>8);		//转换成有符号整形 就是 负数
						senddata_CAN1[3]=0xFFFF-((weizhi_fangwei_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN1[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN1[3])	//fuyang反转
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				第6个字节
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				第7个字节
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x7530)				//输入俯仰速度>-30000/<0x7530		65535-30000+1(uint16_t) == -30000(int16_t)
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp-0x7530)+0x0001)>>8);		//转换成有符号整形 就是 负数
						senddata_CAN1[6]=0xFFFF-((weizhi_fuyang_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN1[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
			}
			//带位置规划的位置环 
			else if(0x10==R_data_CAN1[1])																			//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN1[4]	=	0x23;																				//fangwei_kongzhizi	第5个字节		1100 1000
				senddata_CAN1[7]	=	0x23;																				//fuyang_kongzhizi	第8个字节
				
				senddata_CAN1[2] = R_data_CAN1[4]+(fangwei_cha>>8);							//fangwei_high			发送第3个字节
				senddata_CAN1[3] = R_data_CAN1[5]+(fangwei_cha);								//fangwei_low				第4个字节
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//输入方位角>36000/0x8CA0
				{
					senddata_CAN1[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN1[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN1[5] = R_data_CAN1[6]+(fuyang_cha>>8);							//fuyang_high				第6个字节
				senddata_CAN1[6] = R_data_CAN1[7]+(uint8_t)(fuyang_cha);				//fuyang_low				第7个字节
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
				if(weizhi_fuyang_temp>0x8CA0)				//输入俯仰角>36000/0x8CA0
				{
					senddata_CAN1[5]=(weizhi_fuyang_temp-0x8CA0)>>8;
					senddata_CAN1[6]=(weizhi_fuyang_temp-0x8CA0);
				}
			}
			//不带位置规划的位置环
			else if(0x11==R_data_CAN1[1])																			//第2位 驱动控制字 00 转矩环	01 速度环 10 带位置规划的位置环 11 不带位置规划的位置环
			{
				senddata_CAN1[4]	=	0x33;																				//fangwei_kongzhizi	第5个字节		1100 1100
				senddata_CAN1[7]	=	0x33;																				//fuyang_kongzhizi	第8个字节
				
				senddata_CAN1[2] = R_data_CAN1[4]+(fangwei_cha>>8);							//fangwei_high			发送第3个字节
				senddata_CAN1[3] = R_data_CAN1[5]+(fangwei_cha);								//fangwei_low				第4个字节
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//输入方位角>36000/0x8CA0
				{
					senddata_CAN1[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN1[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN1[5] = R_data_CAN1[6]+(fuyang_cha>>8);							//fuyang_high				第6个字节
				senddata_CAN1[6] = R_data_CAN1[7]+(uint8_t)(fuyang_cha);				//fuyang_low				第7个字节
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
				if(weizhi_fuyang_temp>0x1F40)				//输入俯仰角>8000/0x1F40
				{
					senddata_CAN1[5]=(weizhi_fuyang_temp-0x1F40)>>8;
					senddata_CAN1[6]=(weizhi_fuyang_temp-0x1F40);
				}
			}
			if(0x11==R_data_CAN1[2])	//方位不使能
			{
				if(0x00==R_data_CAN1[1])//转矩环
				senddata_CAN1[4] = 0x02;																				//fangwei_kongzhizi	第5个字节		0000 0011使能 0000 0010不使能
				if(0x01==R_data_CAN1[1])//速度环
				senddata_CAN1[4] = 0x12;
				if(0x10==R_data_CAN1[1])//位置环 位置规划
				senddata_CAN1[4] = 0x22;
				if(0x11==R_data_CAN1[1])//位置环 不带位置规划
				senddata_CAN1[4] = 0x32;
				

//				senddata_CAN0[2] = 0x00;
//				senddata_CAN0[3] = 0x00;
			} 
			if(0x11==R_data_CAN1[3])	//俯仰不使能
			{
				if(0x00==R_data_CAN1[1]) //转矩环
				senddata_CAN1[7] = 0x02;																				//fangwei_kongzhizi	第5个字节		0000 0011使能 0000 0010不使能
				if(0x01==R_data_CAN1[1]) //速度环
				senddata_CAN1[7] = 0x12; 
				if(0x10==R_data_CAN1[1]) //位置环 位置规划
				senddata_CAN1[7] = 0x22; 
				if(0x11==R_data_CAN1[1]) //位置环 不带位置规划
				senddata_CAN1[7] = 0x32;
//				senddata_CAN0[5] = 0x00;
//				senddata_CAN0[6] = 0x00;
			}
			
//			if(senddata_CAN1[5]==0xff) senddata_CAN1[2]=0;									//如果角度=360，角度=0
//			if(senddata_CAN1[6]==0xff) senddata_CAN1[3]=0;
			
			
			for(i=0;i<len-1;i++)
			{
				senddata_CAN1[8] += senddata_CAN1[i];//当前长度类型数据最后一位赋值
			}
			//*
		}
		
		
if(0x05==type){					//CAN0数据发送给USART1
	for(i=0;i<len;i++)
	{
		usart_data_transmit (USART1,senddata_CAN0[i]);
		
		while(usart_flag_get(USART1,USART_FLAG_TC)==RESET);	//等待发送完成
//		data_ok=1;
	}
}
if(0x06==type){					//CAN1数据发送给USART1
	for(i=0;i<len;i++)
	{
		usart_data_transmit (USART1,senddata_CAN1[i]);
		
		while(usart_flag_get(USART1,USART_FLAG_TC)==RESET);	//等待发送完成
//		data_ok=1;
	}
}
	data_ok=1;	//数据发送完成标志位置1
}

/*接收驱动器数据发送给CAN*/
void send_to_can(void)
{
	int i;
	if(SET==usart_rx_to_can_flag)		//422接收数据 接收到数据 usart_rx_to_can_flag==SET/没接收RESET
	{
		
		for(i=0;i<8;i++)
		{
			usart1_receive[i]=data[i];		//转存422数据
		}
//		if((0x55==usart1_receive[0])&&(0xAA==usart1_receive[1]))
//		{
		usart1_receive[8]=data_last;
		transmit_message.tx_sfid =1891;
//		transmit_message.tx_efid =0x1891;
		transmit_message.tx_ft = CAN_FT_DATA;
		transmit_message.tx_ff = CAN_FF_EXTENDED;
		transmit_message.tx_dlen = 8;
	//	if((0x55==data[0])&&(0xAA==data[1]))
	//	{
	//		transmit_message.tx_data[2]=0x05;
	//	}
	//	else
	//	{
	//		transmit_message.tx_data[2]=0x00;
	//	}
		transmit_message.tx_data[0]=(usart1_receive[0]<<4|usart1_receive[1]>>4);
		transmit_message.tx_data[1]=usart1_receive[2];
		transmit_message.tx_data[2]=usart1_receive[3];
		transmit_message.tx_data[3]=usart1_receive[4];
		transmit_message.tx_data[4]=usart1_receive[5];
		transmit_message.tx_data[5]=usart1_receive[6];
		transmit_message.tx_data[6]=usart1_receive[7];
		transmit_message.tx_data[7]=usart1_receive[8];
		while(SET==can_flag_get(CAN0,CAN_FLAG_TME0))
		{
			can_message_transmit(CAN0,&transmit_message);
		}
		while(SET==can_flag_get(CAN1,CAN_FLAG_TME1))
		{
			can_message_transmit(CAN1,&transmit_message);
		}
		usart_rx_to_can_flag=RESET;
	}
//}
}

/*!
    \brief      usart configure
    \param[in]  none
    \param[out] none
    \retval     none
*/
/*串口1初始化*/
static void usart1_config(void)
{
    /* enable GPIO clock */
    //rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);	

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    /* USART configure */
    usart_deinit(USART1);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);	//校验位
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
		nvic_irq_enable(USART1_IRQn,0,3);//接收中断使能
//		usart_interrupt_flag_clear(USART1,USART_INT_RBNE);
//		usart_interrupt_flag_clear(USART1,USART_INT_IDLE);

		usart_interrupt_enable(USART1,USART_INT_RBNE);	/*!< read data buffer not empty interrupt and overrun error interrupt */
	//	usart_interrupt_enable(USART1,USART_INT_IDLE);

}

/*串口3初始化*/
static void uart3_config(void)
{
    /* enable GPIO clock */
    //rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART3);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    /* USART configure */
    usart_deinit(UART3);
    usart_word_length_set(UART3, USART_WL_8BIT);
    usart_stop_bit_set(UART3, USART_STB_1BIT);
    usart_parity_config(UART3, USART_PM_NONE);
    usart_baudrate_set(UART3, 115200U);
    usart_receive_config(UART3, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART3, USART_TRANSMIT_ENABLE);
    usart_enable(UART3);
		nvic_irq_enable(UART3_IRQn,0,3);
//		usart_interrupt_flag_clear(USART1,USART_INT_RBNE);
//		usart_interrupt_flag_clear(USART1,USART_INT_IDLE);

		usart_interrupt_enable(UART3,USART_INT_RBNE);
	//	usart_interrupt_enable(USART1,USART_INT_IDLE);

}

//void can_config(can_parameter_struct can_parameter, can_filter_parameter_struct can_filter)
//{
//    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
//    can_struct_para_init(CAN_INIT_STRUCT, &can_filter);
//    /* initialize CAN register */
//    can_deinit(CAN0);
//    can_deinit(CAN1);
//    
//    /* initialize CAN parameters */
//    can_parameter.time_triggered = DISABLE;
//    can_parameter.auto_bus_off_recovery = DISABLE;
//    can_parameter.auto_wake_up = DISABLE;
//    can_parameter.auto_retrans = DISABLE;
//    can_parameter.rec_fifo_overwrite = DISABLE;
//    can_parameter.trans_fifo_order = DISABLE;
//    can_parameter.working_mode = CAN_NORMAL_MODE;
//    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
//    can_parameter.time_segment_1 = CAN_BT_BS1_3TQ;
//    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
//    
//    /* 1MBps */
//#if CAN_BAUDRATE == 1000
//    can_parameter.prescaler = 6;
//    /* 500KBps */
//#elif CAN_BAUDRATE == 500
//    can_parameter.prescaler = 12;
//    /* 250KBps */
//#elif CAN_BAUDRATE == 250
//    can_parameter.prescaler = 24;
//    /* 125KBps */
//#elif CAN_BAUDRATE == 125
//    can_parameter.prescaler = 48;
//    /* 100KBps */
//#elif  CAN_BAUDRATE == 100
//    can_parameter.prescaler = 60;
//    /* 50KBps */
//#elif  CAN_BAUDRATE == 50
//    can_parameter.prescaler = 120;
//    /* 20KBps */
//#elif  CAN_BAUDRATE == 20
//    can_parameter.prescaler = 300;
//#else
//    #error "please select list can baudrate in private defines in main.c "
//#endif  
//    /* initialize CAN */
//    can_init(CAN0, &can_parameter);
//    can_init(CAN1, &can_parameter);
//    
//    /* initialize filter */ 
//    can_filter.filter_number=0;
//    can_filter.filter_mode = CAN_FILTERMODE_MASK;
//    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
//    can_filter.filter_list_high = 0x0000;
//    can_filter.filter_list_low = 0x0000;
//    can_filter.filter_mask_high = 0x0000;
//    can_filter.filter_mask_low = 0x0000;
//    can_filter.filter_fifo_number = CAN_FIFO0;
//    can_filter.filter_enable = ENABLE;
//    
//    can_filter_init(&can_filter);
//    
//    /* CAN1 filter number */
//    can_filter.filter_number = 15;
//    can_filter_init(&can_filter);
//}

void can0_networking_init(void)
{
//		can_parameter_struct can_parameter;
//    can_filter_parameter_struct can_filter;
		can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
	
//		rcu_periph_clock_enable(RCU_GPIOA);
//		rcu_periph_clock_enable(RCU_CAN0);
	
	/* configure CAN1 GPIO, CAN1_TX(PA12) and CAN1_RX(PA11) */
//    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
//    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    //gpio_pin_remap_config(GPIO_CAN1_FULL_REMAP, ENABLE);
	
    
    /* initialize CAN register */
    can_deinit(CAN0);
//		can_deinit(CAN1);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;						//时间触发通信模式
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;//
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_3TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    /* baudrate 1Mbps */		// 1/(BT_SJWA+BS1+BS2)/*PRESCALER
    //    /* 1MBps */
#if CAN_BAUDRATE == 1000
    can_parameter.prescaler = 6;
    /* 500KBps */
#elif CAN_BAUDRATE == 500
    can_parameter.prescaler = 12;
    /* 250KBps */
#elif CAN_BAUDRATE == 250
    can_parameter.prescaler = 24;
    /* 125KBps */
#elif CAN_BAUDRATE == 125
    can_parameter.prescaler = 48;
    /* 100KBps */
#elif  CAN_BAUDRATE == 100
    can_parameter.prescaler = 60;
    /* 50KBps */
#elif  CAN_BAUDRATE == 50
    can_parameter.prescaler = 120;
    /* 20KBps */
#elif  CAN_BAUDRATE == 20
    can_parameter.prescaler = 300;
#else
    #error "please select list can baudrate in private defines in main.c "
#endif  
    can_init(CAN0, &can_parameter);
//		can_init(CAN1, &can_parameter);
    
        /* initialize filter */
//#ifdef  CAN0_USED
    /* CAN0 filter number */
    can_filter.filter_number = 0;
//#else
//    /* CAN1 filter number */
//    can_filter.filter_number = 15;
//#endif

    /* initialize filter */    
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
		
		
		/* enable CAN receive FIFO1 not empty interrupt */
//    can_interrupt_enable(CAN0, CAN_INT_RFNE1);
}

void can1_networking_init(void)
{
//		can_parameter_struct can_parameter;
//    can_filter_parameter_struct can_filter;
		can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
	
//		rcu_periph_clock_enable(RCU_GPIOA);
//		rcu_periph_clock_enable(RCU_CAN0);
	
	/* configure CAN1 GPIO, CAN1_TX(PA12) and CAN1_RX(PA11) */
//    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
//    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    //gpio_pin_remap_config(GPIO_CAN1_FULL_REMAP, ENABLE);
	
    
    /* initialize CAN register */
//    can_deinit(CAN0);
		can_deinit(CAN1);

    /* initialize CAN */
    can_parameter.time_triggered = DISABLE;						//时间触发通信模式
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;//
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_3TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    /* baudrate 1Mbps */		// 1/(BT_SJWA+BS1+BS2)/*PRESCALER
    //    /* 1MBps */
#if CAN_BAUDRATE == 1000
    can_parameter.prescaler = 6;
    /* 500KBps */
#elif CAN_BAUDRATE == 500
    can_parameter.prescaler = 12;
    /* 250KBps */
#elif CAN_BAUDRATE == 250
    can_parameter.prescaler = 24;
    /* 125KBps */
#elif CAN_BAUDRATE == 125
    can_parameter.prescaler = 48;
    /* 100KBps */
#elif  CAN_BAUDRATE == 100
    can_parameter.prescaler = 60;
    /* 50KBps */
#elif  CAN_BAUDRATE == 50
    can_parameter.prescaler = 120;
    /* 20KBps */
#elif  CAN_BAUDRATE == 20
    can_parameter.prescaler = 300;
#else
    #error "please select list can baudrate in private defines in main.c "
#endif  
//    can_init(CAN0, &can_parameter);
		can_init(CAN1, &can_parameter);
    
        /* initialize filter */
//#ifdef  CAN0_USED
//    /* CAN0 filter number */
//    can_filter.filter_number = 0;
//#else
    /* CAN1 filter number */
    can_filter.filter_number = 15;
//#endif

    /* initialize filter */    
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO1;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
		
		
		/* enable CAN receive FIFO1 not empty interrupt */
//    can_interrupt_enable(CAN0, CAN_INT_RFNE1);
}

void nvic_config(void)
{
//#ifdef  CAN0_USED
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn,0,0);
//#else
    /* configure CAN1 NVIC */
    nvic_irq_enable(CAN1_RX1_IRQn,0,0);
//#endif
}

void can_gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
		rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOA,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_12);
	//开启部分重映射：CAN1_RX/PB8,CAN1_TX/PB9;完全重映射：CAN1_RX/PD0,CAN1_TX/PD1;关闭重映射：CAN1_RX/PA11,CAN_TX/PA12
//    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP,ENABLE);		//
    
//		gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
//    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
//    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP,ENABLE);
    /* configure CAN1 GPIO */
    gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_12);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
	//关闭重映射：CAN1_RX/PB12,CAN1_TX/PB13;开启重映射：CAN1_RX/PB5,CAN_TX/PB6
//    gpio_pin_remap_config(GPIO_CAN1_REMAP,ENABLE);		
	
}


/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
    return ch;
}

/*发送单个字符*/
void usart_sendByte(uint32_t USARTX, uint8_t ch)
{
	usart_data_transmit(USARTX,ch);
	while(usart_flag_get(USARTX,USART_FLAG_TBE));
	
}

/*发送字符串*/
void usart_sendString(uint32_t USARTX, char *str)
{
	unsigned int k=0;
	do
	{
		usart_sendByte(USARTX,*(str+k));
		k++;

	}while(*(str+k)!='\0');
	while(usart_flag_get(USARTX,USART_FLAG_TC)==RESET);

}

void COMxSendData( uint32_t USARTX, uint8_t* p, uint8_t num )
{
	int i;

	for ( i = 0; i < num; i++ )
	{
		usart_data_transmit( USARTX, * ( p + i ) );
		while ( !usart_flag_get ( USARTX, USART_FLAG_TC ) );
	}
}

//uint8_t Can_Send_Msg(uint32_t can_periph, Frametype datatype, uint32_t Id, uint8_t* msg)
//{	
//	uint8_t mbox;
//	uint16_t i=0;
//	can_trasnmit_message_struct TxMessage;


//	if(datatype == StandFrame)
//	{
//		TxMessage.tx_sfid=Id;					// 标准标识符为0
//		TxMessage.tx_ff=CAN_FF_STANDARD;			// 使用标准帧标识符
//	}
//	else
//	{
//		TxMessage.tx_efid=Id;			 		// 设置扩展标示符（29位）
//		TxMessage.tx_ff=CAN_FF_EXTENDED;			// 使用扩展标识符
//	}

//	
//	TxMessage.tx_ft=CAN_FT_DATA;			// 消息类型为数据帧，一帧8位
//	TxMessage.tx_dlen=8;					// 发送两帧信息
//	
//	for(i=0;i<8;i++)
//	TxMessage.tx_data[i]=msg[i];			// 第一帧信息          
//	mbox= can_message_transmit(can_periph, &TxMessage);   
//	
//	i=0;
//	while((can_flag_get(can_periph, mbox)!=CAN_TRANSMIT_FAILED)&&(i<0XFFF)) i++;	//等待发送结束

//	if(i>=0XFFF)return 1;
//	return 0;		

//}


//void Controller_transfer_Data() //解析天线主机数据
//{

//	int8_t i,j;
//  SWJcount = 0;
//	for ( j = 0; j < 70; j++ )//使用循环解析数据
//		{
//			if((SWJdata[j] == 0x1e) && (SWJdata[j+1] == 0x1e)&& (SWJdata[j+2] != 0x00))//??????????
//			{
//				for ( i = 0; i < 10; i++ )//导出数据
//				{
//					SWJdata1[i] = SWJdata[j+i];
//				}
//					 if(SWJdata1[2] ==0x0a) //控制反馈
//					{
//						reback_flag =1;
//					}
//					else if(SWJdata1[2] ==0x0b)//设备状态
//					{
//							 zijian_return = 1;
//							 SWJdata1[2] = 0;
//					}
//					else if(SWJdata1[2] ==0x0c)//工作状态
//					{
//								 work_return = 1;
//					}
//					else if(SWJdata1[2] ==0x0d)//MY启动状态
//					{
//							 MY_return = 1;
//					}
//					else if(SWJdata1[2] ==0x0e)//HY启动状态
//					{
//							HY_return = 1;
//					}
//					else if(SWJdata1[2] ==0x0f)//校时状态
//					{
//							Time_return = 1;
//					}
//					else if(SWJdata1[2] ==0x10)//波束范围
//					{
//							boshu_flag = 1;
//					}
//				
//					CANsenddata();//发送至平台计算机数据
//			}
//		}
//	
//}
//void CANsenddata(void)
//{
//	uint8_t cansendata[8];//定义
//	
////		if((zhuji_return == 1)||(boshu_flag == 1))//主机返回信息
////		{
//			cansendata[0] = 0x01;//数据第1位赋值
//			cansendata[3] = 0;//数据第4位赋值
//			cansendata[4] = SWJdata1[5];//数据第5位赋值
//			cansendata[5] = SWJdata1[6];//数据第6位赋值
//			cansendata[6] = SWJdata1[7];//数据第7位赋值
//			cansendata[7] = SWJdata1[8];//数据第8位赋值
//	
//	
////  Can_Send_Msg ( CAN_1, StandFrame, 1891, cansendata );
//	
//		  if(reback_flag == 1)//控制反馈
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0xf0;//数据第3位赋值

//				reback_flag = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		  else if(zijian_return == 1)//设备状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x40;//数据第3位赋值
//				zijian_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(work_return == 1)//工作状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x41;//数据第3位赋值
//				work_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}	
//		 else if(MY_return == 1)//MY状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x42;//数据第3位赋值
//				MY_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(HY_return == 1)//HY状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x43;//数据第3位赋值
//				HY_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(Time_return == 1)//校时状态
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x44;//数据第3位赋值
//				Time_return = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}
//		 else if(boshu_flag == 1)//波束范围
//			{
//				cansendata[1] = 0x0a;//数据第2位赋值
//				cansendata[2] = 0x45;//数据第3位赋值
//				boshu_flag = 0;//赋初值
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//传输
//			}			
//}
//uint8_t Can_Send_Msg(uint32_t can_periph, Frametype datatype, uint32_t Id, uint8_t* msg)
//{	
//	uint8_t mbox;
//	uint16_t i=0;
//	can_trasnmit_message_struct TxMessage;


//	if(datatype == StandFrame)
//	{
//		TxMessage.tx_sfid=Id;					// 标准标识符为0
//		TxMessage.tx_ff=CAN_FF_STANDARD;			// 使用标准帧标识符
//	}
//	else
//	{
//		TxMessage.tx_efid=Id;			 		// 设置扩展标示符（29位）
//		TxMessage.tx_ff=CAN_FF_EXTENDED;			// 使用扩展标识符
//	}

//	
//	TxMessage.tx_ft=CAN_FT_DATA;			// 消息类型为数据帧，一帧8位
//	TxMessage.tx_dlen=8;					// 发送两帧信息
//	
//	for(i=0;i<8;i++)
//	TxMessage.tx_data[i]=msg[i];			// 第一帧信息          
//	mbox= can_message_transmit(can_periph, &TxMessage);   
//	
//	i=0;
//	while((can_flag_get(can_periph, mbox)!=CAN_TRANSMIT_FAILED)&&(i<0XFFF)) i++;	//等待发送结束

//	if(i>=0XFFF)return 1;
//	return 0;		

//}

