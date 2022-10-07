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

#define JianSuQu 200  //1�� �ߵͼ��ٶ�
#define FWJianSuQu 250  //1�� ��λ���ٶ�
#define ShanshaoJianSuQu 300  //1�� ��ɨ���ٶ�
#define SiQu     20   //0.1�� ������Χ
#define SHANSHAOSiQu     50   //0.1�� ��ɨ�ٶ�
#define ZhuliuZuiXiaoSuDu 100 //פ����С�ٶ�
#define ShanshaoZuiXiaoSuDu 1800 //��ɨ��С�ٶ�
#define JiaSuDu_CanShu  100 //a=30��/s2 ���ٶȳ���
//static int32_t  ttt,FWttt = 0;//�������ٱ���
double fwdata = 0, gddata = 0;//��λ���ݣ��ߵ�����
uint32_t time;//���ֵ�һ���ϵ�
uint32_t encoder_shujuFW,encoder_shujuGD;//���������ݣ���λ/�ߵͣ�
uint8_t test_data[6] = {0x11,0x22,0X33,0x06,00,0x0F};	//��������	
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

extern SWJMingLingtypedef tSWJMingLing;//��������
WeiZhiStatusTypeDef weizhistatus = WEIZHI_STATUS_BUTT;//�ߵ�λ��״̬
WeiZhiStatusTypeDef fwweizhistatus = FWWEIZHI_STATUS_BUTT;//��λλ��״̬

extern uint16_t SWJcount;
extern uint8_t SWJdata1[10],R_data_CAN0[8],R_data_CAN1[8],can1_flag,can2_flag,zhujizijian_flag,fenjizijian_flag,normal_flag,special_flag,test_flag;//���ñ���
extern uint8_t HY_flag,UTCtime_flag,Datatime_flag,long_flag,ceju_flag,data_fail,work_flag,MY_flag,dingxiang_flag,reback_flag,weizhi_flag_CAN0,weizhi_flag_CAN1,weizhi_flag,Controller_transfer_Data_flag;//���ñ���
extern uint8_t SWJdata[70],ZIJIAN[8],Daowei_flag,FW_motor,GD_motor,communication_error,zhuji_return,daowei_flag,daowei_stop,lingwei_flag,tansuo_flag,guiling_flag;//���ñ���
extern uint8_t  MY_return,HY_return,Time_return,boshu_flag,zijian_return,work_return,reback_flag,data_ok,data_recive_ok;
extern uint8_t time_flag_1,zhujizijian_flag_1,work_flag_1,dingxiang_flag_1,MY_flag_1,HY_flag_1,UTCtime_flag_1,Datatime_flag_1,dataflag_1,weizhi_flag_1,daowei_flag_1,long_flag_1,ceju_flag_1,fenjizijian_flag_1;


position_t zhuliuFW,zhuliuGD,mapandangqian;//פ����λ��פ���ߵͣ����̵�ǰֵ
position_t fwshansao_start, fwshansao_stop,gdshansao_start, gdshansao_stop;//��λ��ɨ���ã���λ��ɨֹͣ���ߵ���ɨ���ã��ߵ���ɨֹͣ
volatile position_t FWmapan0,FWmapanxiuzheng,FWmapanyuanshi,GDmapan0,GDmapanxiuzheng,GDmapanyuanshi,FW_last,GD_last;//���ñ���
speed_t shezhifwsudu,shezhigdsudu,aaaa;//���÷�λ�ٶȣ����øߵ��ٶȣ�����
speed_t fwshansaosudu, gdshansaosudu, fwsousuosudu, fwzhuliusudu, gdzhuliusudu,zidongduizhunsudu;//��λ��ɨ�ٶ�?���ߵ���ɨ�ٶ�?����λ�����ٶȣ���λפ���ٶȣ��ߵ�פ���ٶȣ��Զ���׼�ٶ�
uint16_t fangwei_cha=0x0000,fuyang_cha=0x2AAA;	//��λ0�͸���ֵ60 ԭʼ��λ��������λ��ֵ

void usart_send_char(uint32_t usart_periph,uint8_t data);
void UART_Write(uint8_t *pData, uint32_t dataLen);
void sendSWJdata(uint8_t len,uint8_t type);		//CAN0
void sendSWJdata_1(uint8_t len,uint8_t type);//��������������CAN1
void sendQDQdata(uint8_t len,uint8_t type);

void sendagain(uint8_t flag);
speed_t GDZhuLiuJianSu ( speed_t sudu, position_t mubiao, position_t dangqian );//�ߵ�פ������
speed_t FWZhuLiuJianSu ( speed_t FWsudu, position_t FWmubiao, position_t FWdangqian );//��λפ������
speed_t GDZhuLiuFuc ( position_t zhuliuweizhi, position_t dangqianweizhi, speed_t sd );
speed_t FWZhuLiuFuc ( position_t FWzhuliuweizhi, position_t FWdangqianweizhi, speed_t FWsd );

void sendzhujidata(void);//����������������
void send_to_can(void);
void COMxSendData( uint32_t USARTX, uint8_t* p, uint8_t num );

//void Controller_transfer_Data(void); //????????
//void CANsenddata(void);
//uint8_t Can_Send_Msg(uint32_t can_periph, Frametype datatype, uint32_t Id, uint8_t* msg);

position_t xianwei(position_t weizhi);//��λ

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
		/*ʱ��Ƶ����֤*/
			sys_clk_freq=rcu_clock_freq_get(CK_SYS);
			ahb_clk_freq=rcu_clock_freq_get(CK_AHB);
			apb1_clk_freq=rcu_clock_freq_get(CK_APB1);
			apb2_clk_freq=rcu_clock_freq_get(CK_APB2);

    while(1){

				sendzhujidata();//��������������
//			if((SWJdata[0] == 0x1e) && (SWJdata[1] == 0x1e)&& (SWJdata[2] != 0x00))
//			{
//				Controller_transfer_Data();
//			}
				
//				if(time == 0)//��һ���ϵ�ִ��
//			{
//				time = 1;//ʹ����Ϊ1���¸�ѭ�����ڽ���
//				COMxSendData(USART1,test_data,1);
//				delay_1ms(100);  //�ȴ�100ms
////				zijian_return = 1;//�Լ�״̬
////				sendSWJdata(10,0x13); //�Լ�
////				dataflag = 1;//����ָ��
////				tSWJMingLing=Guiling;
//			}
//					usart_data_transmit(UART3,0xab);
//				delay_1ms(1000);


		//				send_to_can();	//USART1���յ������ݷ��͸�CAN
}
}

/*********************************�����߷�������********************************************/

void sendzhujidata(void)//
{
	if(long_flag == 1)//��γ��
	{
		sendSWJdata(11,0x03);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(11,0x03);//���·���
		}
		else long_flag = 0;//���¸�ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(11,0x03);//���·���
		}
    long_flag = 0;//���¸�ֵ
	}
	else if(ceju_flag == 1)//���
	{
		sendSWJdata(10,0x04);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(10,0x04);//���·���
		}
		else ceju_flag = 0;//���¸�ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(10,0x04);//���·���
		}
    ceju_flag = 0;//���¸�ֵ 
	}
	else if(UTCtime_flag == 1)//UTCʱ��ת��
	{
		sendSWJdata(10,0x01);//ת������
		UTCtime_flag = 0;//���¸�ֵ
	}
	else if(Datatime_flag == 1)//Dataʱ��ת��
	{
		sendSWJdata(10,0x02);//ת������
		Datatime_flag = 0;//���¸�ֵ
	}
	else if(zhujizijian_flag == 1)//�����Լ�
	{
		sendSWJdata(6,0x05);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(6,0x05);//���·���
		}
		else work_flag = 0;//���¸���ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(6,0x05);//���·���
		}
		zhujizijian_flag = 0;//���¸���ֵ
		delay_1ms(100);//�ȴ�100ms
		fenjizijian_flag = 1;//�����ֻ��Լ�
		sendagain(zijian_return);//����ȷ��
	}
	else if(data_recive_ok == 1)//��������Ч�ж�
	{
		sendSWJdata(7,0);//ת������
		delay_1ms(100);//�ȴ�100ms
	  if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(7,0);//���·���
		}
		else data_recive_ok = 0;//���¸���ֵ
		delay_1ms(100);;//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(7,0);//���·���
		}
		data_recive_ok = 0;//���¸���ֵ
	}
	else if(work_flag == 1)//����ģʽ
	{
		sendSWJdata(10,0x06);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(10,0x06);//���·���
		}
		else work_flag = 0;//���¸���ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata(10,0x06);//���·���
		}
		work_flag = 0;//���¸���ֵ
	}
    	else if(dingxiang_flag == 1)//����ѯ��
	{
			sendSWJdata(10,0x07);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata(10,0x07);//���·���
			}
			else dingxiang_flag = 0;//���¸���ֵ
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata(10,0x07);//���·���
			}
			dingxiang_flag = 0;//���¸���ֵ
			sendagain(boshu_flag);//����ȷ��
	}
	else if(MY_flag == 1)//MY�л�
	{
			sendSWJdata(10,0x08);//ת������
		  delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata(10,0x08);//���·���
			}
			else work_flag = 0;//���¸���ֵ
			
		  delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata(10,0x08);//���·���
			}
      MY_flag = 0;//����ֵ
		  sendagain(MY_return);//����ȷ��
	}
	else if(HY_flag == 1)//HY�л�
	{
			sendSWJdata(6,0x09);//ת������
		  delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			 sendSWJdata(6,0x09);//ת������
			}
			else work_flag = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			 sendSWJdata(6,0x09);//ת������
			}
      HY_flag = 0;//����ֵ
		  sendagain(HY_return);
	}
	else if(fenjizijian_flag == 1)//�Լ�״̬�ϱ�
	{
		  sendSWJdata(10,0x13);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		    sendSWJdata(10,0x13);//ת������
			}
			else fenjizijian_flag = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		    sendSWJdata(10,0x13);//ת������
			}
		  fenjizijian_flag = 0;//����ֵ
	}
	else if(weizhi_flag == 1)//��λ�����Ƕ���Ϣ
	{
			sendSWJdata(10,0x11);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendSWJdata(10,0x11);//ת������
			}
			else weizhi_flag = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata(10,0x11);//ת������
			}
		  weizhi_flag = 0;//����ֵ
	}	
	else if(daowei_flag == 1)//��λ��Ϣ
	{
		if(daowei_stop == 1)//��λ����
		{
			sendSWJdata(10,0x12);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendSWJdata(10,0x12);//ת������
			}
			else daowei_stop = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendSWJdata(10,0x12);//ת������
			}
			daowei_stop = 0;//����ֵ
		}
	}
	else if(data_recive_ok == 1)//��������Ч�ж�
	{			
		sendSWJdata(7,0x00);//ת������
	}
	else if(Controller_transfer_Data_flag==1)
	{
		Controller_transfer_Data();
	}
	/********CAN1*******/
	if(long_flag_1 == 1)//��γ��
	{
		sendSWJdata_1(11,0x03);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(11,0x03);//���·���
		}
		else long_flag_1 = 0;//���¸�ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(11,0x03);//���·���
		}
    long_flag_1 = 0;//���¸�ֵ
	}
	else if(ceju_flag_1 == 1)//���
	{
		sendSWJdata_1(10,0x04);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(10,0x04);//���·���
		}
		else ceju_flag_1 = 0;//���¸�ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(10,0x04);//���·���
		}
    ceju_flag_1 = 0;//���¸�ֵ 
	}
	
	else if(UTCtime_flag_1 == 1)//UTCʱ��ת��
	{
		sendSWJdata_1(10,0x01);//ת������
		UTCtime_flag_1 = 0;//���¸�ֵ
	}
	else if(Datatime_flag_1 == 1)//Dataʱ��ת��
	{
		sendSWJdata_1(10,0x02);//ת������
		Datatime_flag_1 = 0;//���¸�ֵ
	}
	else if(zhujizijian_flag_1 == 1)//�����Լ�
	{
		sendSWJdata_1(6,0x05);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(6,0x05);//���·���
		}
		else work_flag_1 = 0;//���¸���ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(6,0x05);//���·���
		}
		zhujizijian_flag_1 = 0;//���¸���ֵ
		delay_1ms(100);//�ȴ�100ms
		fenjizijian_flag_1 = 1;//�����ֻ��Լ�
		sendagain(zijian_return);//����ȷ��
	}
	else if(data_recive_ok == 1)//��������Ч�ж�
	{
		sendSWJdata_1(7,0);//ת������
		delay_1ms(100);//�ȴ�100ms
	  if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(7,0);//���·���
		}
		else data_recive_ok = 0;//���¸���ֵ
		delay_1ms(100);;//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(7,0);//���·���
		}
		data_recive_ok = 0;//���¸���ֵ
	}
	else if(work_flag_1 == 1)//����ģʽ
	{
		sendSWJdata_1(10,0x06);//ת������
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(10,0x06);//���·���
		}
		else work_flag_1 = 0;//���¸���ֵ
		delay_1ms(100);//�ȴ�100ms
		if(data_ok == 0)//�ж��Ƿ���ɹ�
		{
		  sendSWJdata_1(10,0x06);//���·���
		}
		work_flag_1 = 0;//���¸���ֵ
	}
    	else if(dingxiang_flag_1 == 1)//����ѯ��
	{
			sendSWJdata_1(10,0x07);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata_1(10,0x07);//���·���
			}
			else dingxiang_flag_1 = 0;//���¸���ֵ
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata_1(10,0x07);//���·���
			}
			dingxiang_flag_1 = 0;//���¸���ֵ
			sendagain(boshu_flag);//����ȷ��
	}
	else if(MY_flag_1 == 1)//MY�л�
	{
			sendSWJdata_1(10,0x08);//ת������
		  delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata_1(10,0x08);//���·���
			}
			else work_flag_1 = 0;//���¸���ֵ
			
		  delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata_1(10,0x08);//���·���
			}
      MY_flag_1 = 0;//����ֵ
		  sendagain(MY_return);//����ȷ��
	}
	else if(HY_flag_1 == 1)//HY�л�
	{
			sendSWJdata_1(6,0x09);//ת������
		  delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			 sendSWJdata_1(6,0x09);//ת������
			}
			else work_flag_1 = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			 sendSWJdata_1(6,0x09);//ת������
			}
      HY_flag_1 = 0;//����ֵ
		  sendagain(HY_return);
	}
	else if(fenjizijian_flag_1 == 1)//�Լ�״̬�ϱ�
	{
		  sendSWJdata_1(10,0x13);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		    sendSWJdata_1(10,0x13);//ת������
			}
			else fenjizijian_flag_1 = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		    sendSWJdata_1(10,0x13);//ת������
			}
		  fenjizijian_flag_1 = 0;//����ֵ
	}
	
	else if(weizhi_flag_1 == 1)//��λ�����Ƕ���Ϣ
	{
			sendSWJdata_1(10,0x11);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendSWJdata_1(10,0x11);//ת������
			}
			else weizhi_flag_1 = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
			  sendSWJdata_1(10,0x11);//ת������
			}
		  weizhi_flag_1 = 0;//����ֵ
	}	
	else if(daowei_flag_1 == 1)//��λ��Ϣ
	{
		if(daowei_stop == 1)//��λ����
		{
			sendSWJdata_1(10,0x12);//ת������
			delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendSWJdata_1(10,0x12);//ת������
			}
			else daowei_stop = 0;//����ֵ
			
	  	delay_1ms(100);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendSWJdata_1(10,0x12);//ת������
			}
			daowei_stop = 0;//����ֵ
		}
	}
	else if(data_recive_ok == 1)//��������Ч�ж�
	{			
		sendSWJdata_1(7,0x00);//ת������
	}
//	else if(Controller_transfer_Data_flag==1)
//	{
//		Controller_transfer_Data();
//	}
	
	
	/******************/
	else if(weizhi_flag_CAN0 == 1)	//��λ�����Ƕ���Ϣ.�ж�����CAN0������λ��־
		{
			sendQDQdata(9,0x05);//ת������
			delay_1ms(10);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�,�ɹ���1
			{
		  	sendQDQdata(9,0x05);//ת������
			}
			
				weizhi_flag_CAN0=0;
		}
	else if(weizhi_flag_CAN1 == 1)	//��λ�����Ƕ���Ϣ.�ж�����CAN1������λ��־
		{
			sendQDQdata(9,0x06);//ת������
			delay_1ms(10);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�
			{
		  	sendQDQdata(9,0x06);//ת������
			}
			
				weizhi_flag_CAN1=0;
			
//	  	delay_1ms(100);//�ȴ�100ms
			
		}
		else if(lingwei_flag==1)				//������λ
		{
			fangwei_cha=((uint16_t )data[2]<<8)|data[3];
			fuyang_cha =((uint16_t )data[5]<<8)|data[6];
			lingwei_flag=0;
		}	
		else if(tansuo_flag==1)					//̽��ģʽ
		{
			sendQDQdata(9,0x12);//ת������
			delay_1ms(10);//�ȴ�100ms
			if(data_ok == 0)//�ж��Ƿ���ɹ�,�ɹ���1
			{
		  	sendQDQdata(9,0x12);//ת������
			}
			
				tansuo_flag=0;
		}	
		else if(guiling_flag==1)					//�Ƕȹ���
		{
			fangwei_cha=0X0000;
			fuyang_cha=0x2AAA;
			guiling_flag=0;
		}
		
}



/****************************��������������*����*CAN0*RS422**UART3**************************/
void sendSWJdata(uint8_t len,uint8_t type)//��������������
{
	int i;
  uint8_t senddata[11];//����
	senddata[0] = 0x1e;//���ݵ�һλ��ֵ
	senddata[1] = 0x1e;//���ݵڶ�λ��ֵ
	senddata[2] = type;//���ݵ���λ��ֵ
	senddata[3] = len&0x00ff;//���ݵ���λ��ֵ
	senddata[4] = (len&0xff00)>>8;//���ݵ���λ��ֵ
  senddata[len-1] = 0;//��ǰ���������������һλ��ֵ
  if(len == 7)//����ȷ��֡
	{ 
    if((data_ok == 1)&&(communication_error == 0))//||(boshu_flag == 1))	//��������Ч�ж�
		{			
		   senddata[5] = 0;	//���ݵ�6λ��ֵ����Ч
		}
    else senddata[5] = 0x80;	//���ݵ�6λ��ֵ����Ч
	}
	else if(len == 10)//����ȷ��֡
	{
		if((type == 0x01)&&(UTCtime_flag == 1))//UTCʱ����Ϣ
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if((type == 0x02)&&(Datatime_flag == 1))//dataʱ����Ϣ
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if((type == 0x04)&&(ceju_flag == 1))//Ŀ�����
		{
			//���ݸ�ֵת��*
		    senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x06)//����״̬����
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
	  else if(type == 0x07)//����ѯ��
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x08)//MY�л�
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x11)//��λ�����Ƕ���Ϣ
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[4];
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
			//*
		}
		else if(type == 0x12)//ת��״̬�ϱ�
		{
			if((zhuliuFW == ((uint16_t)(data[2]<<8)|data[3]))&&(zhuliuGD ==((uint16_t)(data[2]<<8)|data[3])))//ת����λ
			{
				senddata[5] = 0x55;//���ݸ�ֵת��
			  daowei_flag = 0;//��λ��־��λ
				daowei_stop = 1;//ֹͣ
			}
			else                                            //δ��λ
			{
				senddata[5] = 0xAA;//���ݸ�ֵת��
				daowei_stop = 0;//����
			}
			//���ݸ�ֵת��
			senddata[6] = R_data_CAN0[5];
			senddata[7] = R_data_CAN0[6];
			senddata[8] = R_data_CAN0[7];
		}
//		else if(type == 0x13)//�Լ�״̬�ϱ�   �ϵ��Լ� �����Լ�
//		{
//			if(fwzhuliusudu != 0)//�жϷ�λ�ٶ��Ƿ�Ϊ0
//			{
//				if(FW_motor == 1)//�жϵ���Ƿ�����
//				{
//		  	  ZIJIAN[4] = 0x00;//��λ�������
//				}
//				else ZIJIAN[4] = 0x10;//��λ����쳣
//			}
//			if(gdzhuliusudu != 0)//�жϸߵ��ٶ��Ƿ�Ϊ0
//			{
//				if(GD_motor == 1)//�жϵ���Ƿ�����
//				{
//		  	   ZIJIAN[3] = 0x00;//�ߵ͵������
//				}
//				else ZIJIAN[3] = 0x08;//�ߵ͵���쳣
//			}
//      senddata[6] = ZIJIAN[0]+ZIJIAN[1]+ZIJIAN[2]+ZIJIAN[3]+ZIJIAN[4];//���ϼ������
//			if(senddata[6] == 0x00)//�ŷ��ֻ�����
//			{
//				senddata[5] = 0x55;//���ݸ�ֵת��
//			}
//      else if(senddata[6]!= 0x00)//�ŷ��ֻ��쳣
//      {
//			  senddata[5] = 0xAA;//���ݸ�ֵת��
//			}
//			//���ݸ�ֵת��*
//			senddata[7] = R_data_CAN0[6];
//			senddata[8] = R_data_CAN0[7];
//            //* 
//		}
		
	}
	else if(len == 11)//Ŀ�꾭γ��
	{
		if(long_flag == 1)//ȷ������
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN0[3];
			senddata[6] = R_data_CAN0[4];
			senddata[7] = R_data_CAN0[5];
			senddata[8] = R_data_CAN0[6];
			senddata[9] = R_data_CAN0[7];
			//*
		}			
	}	
	for(i=0;i<len-1;i++)//�������
	{
		senddata[len-1] ^= senddata[i];//�������
	}
	
  COMxSendData(UART3,senddata,len);//422��������
}

/****************************��������������*����CAN1**RS422**UART3**************************/
void sendSWJdata_1(uint8_t len,uint8_t type)//��������������
{
	int i;
  uint8_t senddata[11];//����
	senddata[0] = 0x1e;//���ݵ�һλ��ֵ
	senddata[1] = 0x1e;//���ݵڶ�λ��ֵ
	senddata[2] = type;//���ݵ���λ��ֵ
	senddata[3] = len&0x00ff;//���ݵ���λ��ֵ
	senddata[4] = (len&0xff00)>>8;//���ݵ���λ��ֵ
  senddata[len-1] = 0;//��ǰ���������������һλ��ֵ
  if(len == 7)//����ȷ��֡
	{ 
    if((data_ok == 1)&&(communication_error == 0))//||(boshu_flag == 1))	//��������Ч�ж�
		{			
		   senddata[5] = 0;	//���ݵ�6λ��ֵ����Ч
		}
    else senddata[5] = 0x80;	//���ݵ�6λ��ֵ����Ч
	}
	else if(len == 10)//����ȷ��֡
	{
		if((type == 0x01)&&(UTCtime_flag_1 == 1))//UTCʱ����Ϣ
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if((type == 0x02)&&(Datatime_flag_1 == 1))//dataʱ����Ϣ
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if((type == 0x04)&&(ceju_flag_1 == 1))//Ŀ�����
		{
			//���ݸ�ֵת��*
		    senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x06)//����״̬����
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
	  else if(type == 0x07)//����ѯ��
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x08)//MY�л�
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x11)//��λ�����Ƕ���Ϣ
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[4];
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
			//*
		}
		else if(type == 0x12)//ת��״̬�ϱ�
		{
			if((zhuliuFW == ((uint16_t)(data[2]<<8)|data[3]))&&(zhuliuGD ==((uint16_t)(data[2]<<8)|data[3])))//ת����λ
			{
				senddata[5] = 0x55;//���ݸ�ֵת��
			  daowei_flag_1 = 0;//��λ��־��λ
				daowei_stop = 1;//ֹͣ
			}
			else                                            //δ��λ
			{
				senddata[5] = 0xAA;//���ݸ�ֵת��
				daowei_stop = 0;//����
			}
			//���ݸ�ֵת��
			senddata[6] = R_data_CAN1[5];
			senddata[7] = R_data_CAN1[6];
			senddata[8] = R_data_CAN1[7];
		}
//		else if(type == 0x13)//�Լ�״̬�ϱ�   �ϵ��Լ� �����Լ�
//		{
//			if(fwzhuliusudu != 0)//�жϷ�λ�ٶ��Ƿ�Ϊ0
//			{
//				if(FW_motor == 1)//�жϵ���Ƿ�����
//				{
//		  	  ZIJIAN[4] = 0x00;//��λ�������
//				}
//				else ZIJIAN[4] = 0x10;//��λ����쳣
//			}
//			if(gdzhuliusudu != 0)//�жϸߵ��ٶ��Ƿ�Ϊ0
//			{
//				if(GD_motor == 1)//�жϵ���Ƿ�����
//				{
//		  	   ZIJIAN[3] = 0x00;//�ߵ͵������
//				}
//				else ZIJIAN[3] = 0x08;//�ߵ͵���쳣
//			}
//      senddata[6] = ZIJIAN[0]+ZIJIAN[1]+ZIJIAN[2]+ZIJIAN[3]+ZIJIAN[4];//���ϼ������
//			if(senddata[6] == 0x00)//�ŷ��ֻ�����
//			{
//				senddata[5] = 0x55;//���ݸ�ֵת��
//			}
//      else if(senddata[6]!= 0x00)//�ŷ��ֻ��쳣
//      {
//			  senddata[5] = 0xAA;//���ݸ�ֵת��
//			}
//			//���ݸ�ֵת��*
//			senddata[7] = R_data_CAN1[6];
//			senddata[8] = R_data_CAN1[7];
//            //* 
//		}
		
	}
	else if(len == 11)//Ŀ�꾭γ��
	{
		if(long_flag_1 == 1)//ȷ������
		{
			//���ݸ�ֵת��*
			senddata[5] = R_data_CAN1[3];
			senddata[6] = R_data_CAN1[4];
			senddata[7] = R_data_CAN1[5];
			senddata[8] = R_data_CAN1[6];
			senddata[9] = R_data_CAN1[7];
			//*
		}			
	}	
	for(i=0;i<len-1;i++)//�������
	{
		senddata[len-1] ^= senddata[i];//�������
	}
	
  COMxSendData(UART3,senddata,len);//422��������
}



/********************************������ƽ̨���������*CAN*************************************/
//void CANsenddata(void)
//{
//	uint8_t cansendata[8];//����
//	
////		if((zhuji_return == 1)||(boshu_flag == 1))//����������Ϣ
////		{
//			cansendata[0] = 0x01;//���ݵ�1λ��ֵ
//			cansendata[3] = 0;//���ݵ�4λ��ֵ
//			cansendata[4] = SWJdata1[5];//���ݵ�5λ��ֵ
//			cansendata[5] = SWJdata1[6];//���ݵ�6λ��ֵ
//			cansendata[6] = SWJdata1[7];//���ݵ�7λ��ֵ
//			cansendata[7] = SWJdata1[8];//���ݵ�8λ��ֵ
//	
//	
////  Can_Send_Msg ( CAN_1, StandFrame, 1891, cansendata );
//	
//		  if(reback_flag == 1)//���Ʒ���
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0xf0;//���ݵ�3λ��ֵ

//				reback_flag = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		  else if(zijian_return == 1)//�豸״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x40;//���ݵ�3λ��ֵ
//				zijian_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(work_return == 1)//����״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x41;//���ݵ�3λ��ֵ
//				work_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}	
//		 else if(MY_return == 1)//MY״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x42;//���ݵ�3λ��ֵ
//				MY_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(HY_return == 1)//HY״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x43;//���ݵ�3λ��ֵ
//				HY_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(Time_return == 1)//Уʱ״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x44;//���ݵ�3λ��ֵ
//				Time_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(boshu_flag == 1)//������Χ
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x45;//���ݵ�3λ��ֵ
//				boshu_flag = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}			
//}

void sendagain(uint8_t flag)//����ȷ��
{
	uint8_t return_flag;//����
	return_flag = flag;//��ֵ
    if(data_ok == 1)//�ж��Ƿ���ɹ�
	 {
			 if(return_flag == 0)//ȷ�ϳɹ���λ�յ���Ӧ�Ļظ���Ϣ
			 {
				communication_error = 1;
			  zhuji_return = 0;
			 }
			 else //ȷ�ϳɹ����յ���Ӧ�Ļظ���Ϣ
			 {
				 zhuji_return = 1;
				 communication_error = 0;
		   }
	 }
}


///***********************************�������****************************************/	
//speed_t GDZhuLiuFuc ( position_t zhuliuweizhi, position_t dangqianweizhi, speed_t sd )
//{
//	speed_t p_sd = sd;//����

//	if ( ( (zhuliuweizhi - dangqianweizhi)   >  18000 ) \
//	     || ( (zhuliuweizhi < dangqianweizhi) && (zhuliuweizhi - dangqianweizhi) > -18000 ) ) //����·���ж�
//	{
//		p_sd = -sd;//����
//	}

//	return GDZhuLiuJianSu ( p_sd, zhuliuweizhi, dangqianweizhi);//�жϼ�������ִ��
//}

//speed_t FWZhuLiuFuc ( position_t FWzhuliuweizhi, position_t FWdangqianweizhi, speed_t FWsd )
//{
//	speed_t FW_sd = FWsd;

//	if ( ( (FWzhuliuweizhi - FWdangqianweizhi)   >  18000 ) \
//	     || ( (FWzhuliuweizhi < FWdangqianweizhi) && (FWzhuliuweizhi - FWdangqianweizhi) > -18000 ) ) //����·���ж�
//	{
//		FW_sd = -FWsd;//����
//	}

//	return FWZhuLiuJianSu ( FW_sd, FWzhuliuweizhi, FWdangqianweizhi);//�жϼ�������ִ��
//}

//speed_t GDZhuLiuJianSu ( speed_t sudu, position_t mubiao, position_t dangqian )//�ߵ�פ������
//{

//	speed_t s = 0;//����
//	speed_t biansu=0,ZuiXiaoSuDu;//����
//		
//	 ZuiXiaoSuDu = ZhuliuZuiXiaoSuDu;//����ֵ

//	if ( sudu == 0 )//�жϸߵ��ٶ��Ƿ�Ϊ0
//	{
//		return 0;//����
//	}

//	if ( (abs ( mubiao - dangqian ) <= JianSuQu )&& (abs ( mubiao - dangqian ) >= SiQu )) //�ж��Ƿ��ڼ������ڣ��Ҳ�������
//	{

//		s = mubiao - dangqian;//�����ֵ

//		if ( abs ( s ) < ZuiXiaoSuDu )//�жϲ�ֵ�Ƿ�С����С�ٶ�
//		{
//			s = ( abs ( s ) / s ) * ZuiXiaoSuDu;//ʹ�������С�ٶȣ��Ҽ����ٶȷ���
//		}
//		else if ( abs ( s ) > abs ( sudu ) )//�ж��Ƿ�����趨�ٶ�
//		{
//			s = sudu;//ʹ������趨�ٶ�
//		}

//	}
//	else if ( abs ( mubiao - dangqian ) < SiQu )//�ж��Ƿ�������
//	{
//		s = 0;//�����ٶ�Ϊ0
//		ttt = 0;//���ü��ٳ���
//        //��ɨ*
//		if (tSWJMingLing == gdshansao)//�ж������Ƿ�Ϊ��ɨ
//		{
//			if (mubiao == gdshansao_start)//�ж���ɨ�Ƿ�����
//			{
//				weizhistatus = OUT_OF_START;//����
//			}
//			else if (mubiao == gdshansao_stop)//�ж���ɨ�Ƿ�ͣ��
//			{
//				weizhistatus = OUT_OF_STOP;//ͣ��
//			}
//		}
//		//*
//	}
//	else 
//	{
//		//��������
//		biansu = abs ( sudu ) / sudu * JiaSuDu_CanShu;//��ȡ���ٶ�ֵ
//		s = ( ttt++ ) * biansu;//���ٶȱ��������Լ�
//		if ( abs ( s ) >= abs ( sudu ) )//�ж��Ƿ�����趨�ٶ�
//		{
//			s = sudu;//ʹ������趨�ٶ�
//		}
//		//*
//	}

//	return s;//�����ٶ�ֵ

//}

//speed_t FWZhuLiuJianSu ( speed_t FWsudu, position_t FWmubiao, position_t FWdangqian )//��λפ������
//{

//	speed_t FWs = 0;//������λ�ٶȱ���
//	speed_t FWbiansu=0,FWZuiXiaoSuDu;//��������
//	int jiansuqu;//����
//	int tingzhi = 0;//����
//		tingzhi = SiQu;//��ֵ
//		FWZuiXiaoSuDu = ZhuliuZuiXiaoSuDu;//��ֵ
//		jiansuqu = FWJianSuQu;//��ֵ

//	if ( FWsudu == 0 )//�жϷ�λ�ٶ��Ƿ�Ϊ0
//	{
//		return 0;//����
//	}

//	if (( abs ( FWmubiao - FWdangqian ) <= jiansuqu) && abs (( FWmubiao - FWdangqian ) >= tingzhi) ) //�ж��Ƿ��ڼ������ڣ��Ҳ�������
//	{

//		FWs = FWmubiao - FWdangqian;//�����ֵ

//		if ( abs ( FWs ) < FWZuiXiaoSuDu )//�жϲ�ֵ�Ƿ�С����С�ٶ�
//		{
//			FWs = ( abs ( FWs ) / FWs ) * FWZuiXiaoSuDu;//ʹ�������С�ٶȣ��Ҽ����ٶȷ���
//		}
//		else if ( abs ( FWs ) > abs ( FWsudu ) )//�ж��Ƿ�����趨�ٶ�
//		{
//			FWs = FWsudu;//ʹ������趨�ٶ�
//		}

//	}
//	else if ( abs ( FWmubiao - FWdangqian ) < tingzhi )//�ж��Ƿ�������
//	{
//		FWs = 0;//�����ٶ�Ϊ0
//		FWttt = 0;//���ü��ٳ���
//		//��ɨ*
//		if (tSWJMingLing == fwshansao)//�ж������Ƿ�Ϊ��ɨ
//		{
//			if ((FWmubiao == fwshansao_start)||((FWmubiao == gdshansao_start))) //�ж���ɨ�Ƿ�����
//			{
//				fwweizhistatus = FWOUT_OF_START;//����
//			}
//			else if ((FWmubiao == fwshansao_stop)||((FWmubiao == gdshansao_stop)))//�ж���ɨ�Ƿ�ͣ��
//			{
//				fwweizhistatus = FWOUT_OF_STOP;//ͣ��
//			}
//		}
//		//*
//	}
//	else 
//	{
//		//��������
//			FWbiansu = abs ( FWsudu ) / FWsudu * JiaSuDu_CanShu;//��ȡ���ٶ�ֵ
//		
//			FWs = ( FWttt++ ) * FWbiansu;//���ٶȱ��������Լ�

//			if ( abs ( FWs ) >= abs ( FWsudu ) )//�ж��Ƿ�����趨�ٶ�
//			{
//				FWs = FWsudu;//ʹ������趨�ٶ�
//			}
//			//*
//	}

//	return FWs;

//}



position_t xianwei(position_t weizhi)//��λ
{
	  position_t geidingweizhi;//�������
	  geidingweizhi = weizhi;//��ֵ
		if((geidingweizhi >= 8000) && (geidingweizhi <18000))//�жϸߵ���λ�Ƕ�
		{
			geidingweizhi = 8000;//�ߵ���λ��ֵ
		}
		else if((geidingweizhi >= 18000) && (geidingweizhi <=36000))//�жϸߵ���λ�Ƕ�
		{
			geidingweizhi = 0;//�ߵ���λ��ֵ
		}
		
		return geidingweizhi;
}

/****************************����������������**RS422**USART1****************************/
void sendQDQdata(uint8_t len,uint8_t type)//����������������
{
	int i;
	uint8_t senddata_CAN0[9]={0};
//  uint8_t WeiZhiHuan[9]={0};//����
//	int8_t SuDuHuan[9]={0};
	uint8_t senddata_CAN1[9]={0};
//	uint8_t sum;
	senddata_CAN0[0] = senddata_CAN1[0] = 0x55;//���ݵ�һ֡��ֵ
	senddata_CAN0[1] = senddata_CAN1[1] = 0xAA;//���ݵڶ�֡��ֵ
//	senddata_CAN0[2] = type;//���ݵ���λ��ֵ
//	senddata_CAN0[3] = len&0x00ff;//���ݵ���λ��ֵ
//	senddata_CAN0[4] = (len&0xff00)>>8;//���ݵ���λ��ֵ
//	for(i=0;i<len-1;i++)
//  {
//		senddata_CAN0[8] += senddata_CAN0[i];//��ǰ���������������һλ��ֵ
//	}

		/*else*/ if(type == 0x05)//��λ�����Ƕ���Ϣ��type==0x05,��������CAN0		
		{
			
			//���ݸ�ֵת��*
			/*CAN0*/	
			
			//ת�ػ�
			if(0x00==R_data_CAN0[1])																				//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN0[4]	=	0x03;																				//fangwei_kongzhizi	��5���ֽ�  1100 0000
				senddata_CAN0[7]	=	0x03;																				//fuyang_kongzhizi	��8���ֽ�
				if(0x00==R_data_CAN0[2])	//��λ��ת
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x4E20)				//���� ��λ�ٶ�>20000/0x4E20
					{
						senddata_CAN0[2]=(weizhi_fangwei_temp-0x4E20)>>8;
						senddata_CAN0[3]=(weizhi_fangwei_temp-0x4E20);
					}
				}
				else if(0x00==R_data_CAN0[3])	//������ת
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x4E20)				//���븩���ٶ�>20000/0x0x4E20
					{
						senddata_CAN0[5]=(weizhi_fuyang_temp-0x4E20)>>8;
						senddata_CAN0[6]=(weizhi_fuyang_temp-0x4E20);
					}
				}
				else if(0x01==R_data_CAN0[2])	//��λ��ת
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x4E20)				//���� ��λ�ٶ�>-20000/<0xB1E0	65535-20000+1(uint16_t) == -20000(int16_t) 0xB1E0
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp-0x4E20)+0x0001)>>8);		//ת�����з������� ���� ����
						senddata_CAN0[3]=0xFFFF-((weizhi_fangwei_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN0[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN0[3])	//������ת
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x4E20)				//���븩���ٶ�>-20000/<0xB1E0		65535-20000+1(uint16_t) == -20000(int16_t)	0xB1E0
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp-0x4E20)+0x0001)>>8);	//ת�����з������� ���� ����
						senddata_CAN0[6]=0xFFFF-((weizhi_fuyang_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN0[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
//				else if(0x11==R_data_CAN0[2])	//��λ��ʹ��
//				{
//					senddata_CAN0[4] = 0x02;																				//fangwei_kongzhizi	��5���ֽ�		0000 0011ʹ�� 0000 0010��ʹ��
////					senddata_CAN0[2] = 0x00;
////					senddata_CAN0[3] = 0x00;
//				} 
//				else if(0x11==R_data_CAN0[2])	//������ʹ��
//				{
//					senddata_CAN0[7] = 0x02;																				//fuyang_kongzhizi	��8���ֽ�
////					senddata_CAN0[5] = 0x00;
////					senddata_CAN0[6] = 0x00;
//				}
			}
			//�ٶȻ�
			else if(0x01==R_data_CAN0[1])																			//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN0[4]	=	0x13;																				//fangwei_kongzhizi	��5���ֽ�		0001 0011ʹ�� 0001 0010��ʹ��
				senddata_CAN0[7]	=	0x13;																				//fuyang_kongzhizi	��8���ֽ�
				if(0x00==R_data_CAN0[2])	//fangwei��ת
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x7530)				//���� ��λ�ٶ�>30000/0x7530
					{
						senddata_CAN0[2]=(weizhi_fangwei_temp-0x7530)>>8;
						senddata_CAN0[3]=(weizhi_fangwei_temp-0x7530);
					}
				}
				else if(0x00==R_data_CAN0[3])	//fuyang��ת
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x7530)				//���븩���ٶ�>30000/0x7530
					{
						senddata_CAN0[5]=(weizhi_fuyang_temp-0x7530)>>8;
						senddata_CAN0[6]=(weizhi_fuyang_temp-0x7530);
					}
				}
				else if(0x01==R_data_CAN0[2])	//fangwei��ת
				{
					senddata_CAN0[2] = R_data_CAN0[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN0[3] = R_data_CAN0[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
					if(weizhi_fangwei_temp>0x7530)				//���� ��λ�ٶ�>-30000/<0x7530
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp-0x7530)+0x0001)>>8);		//ת�����з������� ���� ����
						senddata_CAN0[3]=0xFFFF-((weizhi_fangwei_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN0[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN0[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN0[3])	//fuyang��ת
				{
					senddata_CAN0[5] = R_data_CAN0[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN0[6] = R_data_CAN0[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
					if(weizhi_fuyang_temp>0x7530)				//���븩���ٶ�>-30000/<0x7530		65535-30000+1(uint16_t) == -30000(int16_t)
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp-0x7530)+0x0001)>>8);		//ת�����з������� ���� ����
						senddata_CAN0[6]=0xFFFF-((weizhi_fuyang_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN0[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN0[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
				
				
			}
			//��λ�ù滮��λ�û� 
			else if(0x10==R_data_CAN0[1])																			//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN0[4]	=	0x23;																				//fangwei_kongzhizi	��5���ֽ�		1100 1000
				senddata_CAN0[7]	=	0x23;																				//fuyang_kongzhizi	��8���ֽ�
				
				senddata_CAN0[2] = R_data_CAN0[4]+(fangwei_cha>>8);							//fangwei_high			���͵�3���ֽ�
				senddata_CAN0[3] = R_data_CAN0[5]+(fangwei_cha);								//fangwei_low				��4���ֽ�
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//���뷽λ��>36000/0x8CA0
				{
					senddata_CAN0[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN0[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN0[5] = R_data_CAN0[6]+(fuyang_cha>>8);							//fuyang_high				��6���ֽ�
				senddata_CAN0[6] = R_data_CAN0[7]+(uint8_t)(fuyang_cha);				//fuyang_low				��7���ֽ�
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
				if(weizhi_fuyang_temp>0x8CA0)				//���븩����>36000/0x8CA0
				{
					senddata_CAN0[5]=(weizhi_fuyang_temp-0x8CA0)>>8;
					senddata_CAN0[6]=(weizhi_fuyang_temp-0x8CA0);
				}
			}
			//����λ�ù滮��λ�û�
			else if(0x11==R_data_CAN0[1])																			//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN0[4]	=	0x33;																				//fangwei_kongzhizi	��5���ֽ�		1100 1100
				senddata_CAN0[7]	=	0x33;																				//fuyang_kongzhizi	��8���ֽ�
				
				senddata_CAN0[2] = R_data_CAN0[4]+(fangwei_cha>>8);							//fangwei_high			���͵�3���ֽ�
				senddata_CAN0[3] = R_data_CAN0[5]+(fangwei_cha);								//fangwei_low				��4���ֽ�
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN0[2]<<8) +senddata_CAN0[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//���뷽λ��>36000/0x8CA0
				{
					senddata_CAN0[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN0[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN0[5] = R_data_CAN0[6]+(fuyang_cha>>8);							//fuyang_high				��6���ֽ�
				senddata_CAN0[6] = R_data_CAN0[7]+(uint8_t)(fuyang_cha);				//fuyang_low				��7���ֽ�
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN0[5]<<8) +senddata_CAN0[6]);
				if(weizhi_fuyang_temp>0x1F40)				//���븩����>8000/0x1F40
				{
					senddata_CAN0[5]=(weizhi_fuyang_temp-0x1F40)>>8;
					senddata_CAN0[6]=(weizhi_fuyang_temp-0x1F40);
				}
			}
			
			
//			if(senddata_CAN0[5]==0xff) senddata_CAN0[2]=0;									//����Ƕ�=360���Ƕ�=0
//			if(senddata_CAN0[6]==0xff) senddata_CAN0[3]=0;
			
			if(0x11==R_data_CAN0[2])	//��λ��ʹ��
			{
				if(0x00==R_data_CAN0[1])//ת�ػ�
				senddata_CAN0[4] = 0x02;																				//fangwei_kongzhizi	��5���ֽ�		0000 0011ʹ�� 0000 0010��ʹ��
				if(0x01==R_data_CAN0[1])//�ٶȻ�
				senddata_CAN0[4] = 0x12;
				if(0x10==R_data_CAN0[1])//λ�û� λ�ù滮
				senddata_CAN0[4] = 0x22;
				if(0x11==R_data_CAN0[1])//λ�û� ����λ�ù滮
				senddata_CAN0[4] = 0x32;
				

//				senddata_CAN0[2] = 0x00;
//				senddata_CAN0[3] = 0x00;
			} 
			if(0x11==R_data_CAN0[3])	//������ʹ��
			{
				if(0x00==R_data_CAN0[1]) //ת�ػ�
				senddata_CAN0[7] = 0x02;																				//fangwei_kongzhizi	��5���ֽ�		0000 0011ʹ�� 0000 0010��ʹ��
				if(0x01==R_data_CAN0[1]) //�ٶȻ�
				senddata_CAN0[7] = 0x12; 
				if(0x10==R_data_CAN0[1]) //λ�û� λ�ù滮
				senddata_CAN0[7] = 0x22; 
				if(0x11==R_data_CAN0[1]) //λ�û� ����λ�ù滮
				senddata_CAN0[7] = 0x32;
//				senddata_CAN0[5] = 0x00;
//				senddata_CAN0[6] = 0x00;
			}
			for(i=0;i<len-1;i++)
			{
				senddata_CAN0[8] += senddata_CAN0[i];//��ǰ���������������һλ��ֵ
			}
			
		}
		
		else if(type == 0x06)//��λ�����Ƕ���Ϣ,type==0x06,��������CAN1				
		{
			/*CAN1*/
			//ת�ػ�
			if(0x00==R_data_CAN1[1])																				//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN1[4]	=	0x03;																				//fangwei_kongzhizi	��5���ֽ�  1100 0000
				senddata_CAN1[7]	=	0x03;																				//fuyang_kongzhizi	��8���ֽ�
				if(0x00==R_data_CAN1[2])	//fangwei��ת
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x4E20)				//���� ��λ�ٶ�>20000/0x4E20
					{
						senddata_CAN1[2]=(weizhi_fangwei_temp-0x4E20)>>8;
						senddata_CAN1[3]=(weizhi_fangwei_temp-0x4E20);
					}
				}
				else if(0x00==R_data_CAN1[3])	//fuyang��ת
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x4E20)				//���븩���ٶ�>20000/0x0x4E20
					{
						senddata_CAN1[5]=(weizhi_fuyang_temp-0x4E20)>>8;
						senddata_CAN1[6]=(weizhi_fuyang_temp-0x4E20);
					}
				}
				else if(0x01==R_data_CAN1[2])	//fangwei��ת
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x4E20)				//���� ��λ�ٶ�>-20000/<0xB1E0	65535-20000+1(uint16_t) == -20000(int16_t) 0xB1E0
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp-0x4E20)+0x0001)>>8);		//ת�����з������� ���� ����
						senddata_CAN1[3]=0xFFFF-((weizhi_fangwei_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN1[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN1[3])	//fuyang��ת
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x4E20)				//���븩���ٶ�>-20000/<0xB1E0		65535-20000+1(uint16_t) == -20000(int16_t)	0xB1E0
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp-0x4E20)+0x0001)>>8);	//ת�����з������� ���� ����
						senddata_CAN1[6]=0xFFFF-((weizhi_fuyang_temp-0x4E20))+0x0001;
					}
					else
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN1[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
			}
			//�ٶȻ�
			else if(0x01==R_data_CAN1[1])																			//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN1[4]	=	0x13;																				//fangwei_kongzhizi	��5���ֽ�		1100 0100
				senddata_CAN1[7]	=	0x13;																				//fuyang_kongzhizi	��8���ֽ�
				if(0x00==R_data_CAN1[2])	//fangwei��ת
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x7530)				//���� ��λ�ٶ�>30000/0x7530
					{
						senddata_CAN1[2]=(weizhi_fangwei_temp-0x7530)>>8;
						senddata_CAN1[3]=(weizhi_fangwei_temp-0x7530);
					}
				}
				else if(0x00==R_data_CAN1[3])	//fuyang��ת
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x7530)				//���븩���ٶ�>30000/0x7530
					{
						senddata_CAN1[5]=(weizhi_fuyang_temp-0x7530)>>8;
						senddata_CAN1[6]=(weizhi_fuyang_temp-0x7530);
					}
				}
				else if(0x01==R_data_CAN1[2])	//fangwei��ת
				{
					senddata_CAN1[2] = R_data_CAN1[4]/*+(fangwei_cha>>8)*/;							//��λ�ٶȸ�8λ			���͵�3���ֽ�
					senddata_CAN1[3] = R_data_CAN1[5]/*+(fangwei_cha)*/;								//��λ�ٶȵ�8λ				��4���ֽ�
					weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
					if(weizhi_fangwei_temp>0x7530)				//���� ��λ�ٶ�>-30000/<0x7530
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp-0x7530)+0x0001)>>8);		//ת�����з������� ���� ����
						senddata_CAN1[3]=0xFFFF-((weizhi_fangwei_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN1[2]=((0xFFFF-(weizhi_fangwei_temp)+0x0001)>>8);
						senddata_CAN1[3]=0xFFFF-(weizhi_fangwei_temp)+0x0001;
					}
				}
				else if(0x01==R_data_CAN1[3])	//fuyang��ת
				{
					senddata_CAN1[5] = R_data_CAN1[6]/*+(fuyang_cha>>8)*/;							//fuyang_high				��6���ֽ�
					senddata_CAN1[6] = R_data_CAN1[7]/*+(uint8_t)(fuyang_cha)*/;				//fuyang_low				��7���ֽ�
					weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
					if(weizhi_fuyang_temp>0x7530)				//���븩���ٶ�>-30000/<0x7530		65535-30000+1(uint16_t) == -30000(int16_t)
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp-0x7530)+0x0001)>>8);		//ת�����з������� ���� ����
						senddata_CAN1[6]=0xFFFF-((weizhi_fuyang_temp-0x7530))+0x0001;
					}
					else
					{
						senddata_CAN1[5]=((0xFFFF-(weizhi_fuyang_temp)+0x0001)>>8);
						senddata_CAN1[6]=0xFFFF-(weizhi_fuyang_temp)+0x0001;
					}
				}
			}
			//��λ�ù滮��λ�û� 
			else if(0x10==R_data_CAN1[1])																			//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN1[4]	=	0x23;																				//fangwei_kongzhizi	��5���ֽ�		1100 1000
				senddata_CAN1[7]	=	0x23;																				//fuyang_kongzhizi	��8���ֽ�
				
				senddata_CAN1[2] = R_data_CAN1[4]+(fangwei_cha>>8);							//fangwei_high			���͵�3���ֽ�
				senddata_CAN1[3] = R_data_CAN1[5]+(fangwei_cha);								//fangwei_low				��4���ֽ�
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//���뷽λ��>36000/0x8CA0
				{
					senddata_CAN1[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN1[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN1[5] = R_data_CAN1[6]+(fuyang_cha>>8);							//fuyang_high				��6���ֽ�
				senddata_CAN1[6] = R_data_CAN1[7]+(uint8_t)(fuyang_cha);				//fuyang_low				��7���ֽ�
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
				if(weizhi_fuyang_temp>0x8CA0)				//���븩����>36000/0x8CA0
				{
					senddata_CAN1[5]=(weizhi_fuyang_temp-0x8CA0)>>8;
					senddata_CAN1[6]=(weizhi_fuyang_temp-0x8CA0);
				}
			}
			//����λ�ù滮��λ�û�
			else if(0x11==R_data_CAN1[1])																			//��2λ ���������� 00 ת�ػ�	01 �ٶȻ� 10 ��λ�ù滮��λ�û� 11 ����λ�ù滮��λ�û�
			{
				senddata_CAN1[4]	=	0x33;																				//fangwei_kongzhizi	��5���ֽ�		1100 1100
				senddata_CAN1[7]	=	0x33;																				//fuyang_kongzhizi	��8���ֽ�
				
				senddata_CAN1[2] = R_data_CAN1[4]+(fangwei_cha>>8);							//fangwei_high			���͵�3���ֽ�
				senddata_CAN1[3] = R_data_CAN1[5]+(fangwei_cha);								//fangwei_low				��4���ֽ�
				weizhi_fangwei_temp=(((uint16_t)senddata_CAN1[2]<<8) +senddata_CAN1[3]);
				if(weizhi_fangwei_temp>0x8CA0)				//���뷽λ��>36000/0x8CA0
				{
					senddata_CAN1[2]=(weizhi_fangwei_temp-0x8CA0)>>8;
					senddata_CAN1[3]=(weizhi_fangwei_temp-0x8CA0);
				}
				senddata_CAN1[5] = R_data_CAN1[6]+(fuyang_cha>>8);							//fuyang_high				��6���ֽ�
				senddata_CAN1[6] = R_data_CAN1[7]+(uint8_t)(fuyang_cha);				//fuyang_low				��7���ֽ�
				weizhi_fuyang_temp=(((uint16_t)senddata_CAN1[5]<<8) +senddata_CAN1[6]);
				if(weizhi_fuyang_temp>0x1F40)				//���븩����>8000/0x1F40
				{
					senddata_CAN1[5]=(weizhi_fuyang_temp-0x1F40)>>8;
					senddata_CAN1[6]=(weizhi_fuyang_temp-0x1F40);
				}
			}
			if(0x11==R_data_CAN1[2])	//��λ��ʹ��
			{
				if(0x00==R_data_CAN1[1])//ת�ػ�
				senddata_CAN1[4] = 0x02;																				//fangwei_kongzhizi	��5���ֽ�		0000 0011ʹ�� 0000 0010��ʹ��
				if(0x01==R_data_CAN1[1])//�ٶȻ�
				senddata_CAN1[4] = 0x12;
				if(0x10==R_data_CAN1[1])//λ�û� λ�ù滮
				senddata_CAN1[4] = 0x22;
				if(0x11==R_data_CAN1[1])//λ�û� ����λ�ù滮
				senddata_CAN1[4] = 0x32;
				

//				senddata_CAN0[2] = 0x00;
//				senddata_CAN0[3] = 0x00;
			} 
			if(0x11==R_data_CAN1[3])	//������ʹ��
			{
				if(0x00==R_data_CAN1[1]) //ת�ػ�
				senddata_CAN1[7] = 0x02;																				//fangwei_kongzhizi	��5���ֽ�		0000 0011ʹ�� 0000 0010��ʹ��
				if(0x01==R_data_CAN1[1]) //�ٶȻ�
				senddata_CAN1[7] = 0x12; 
				if(0x10==R_data_CAN1[1]) //λ�û� λ�ù滮
				senddata_CAN1[7] = 0x22; 
				if(0x11==R_data_CAN1[1]) //λ�û� ����λ�ù滮
				senddata_CAN1[7] = 0x32;
//				senddata_CAN0[5] = 0x00;
//				senddata_CAN0[6] = 0x00;
			}
			
//			if(senddata_CAN1[5]==0xff) senddata_CAN1[2]=0;									//����Ƕ�=360���Ƕ�=0
//			if(senddata_CAN1[6]==0xff) senddata_CAN1[3]=0;
			
			
			for(i=0;i<len-1;i++)
			{
				senddata_CAN1[8] += senddata_CAN1[i];//��ǰ���������������һλ��ֵ
			}
			//*
		}
		
		
if(0x05==type){					//CAN0���ݷ��͸�USART1
	for(i=0;i<len;i++)
	{
		usart_data_transmit (USART1,senddata_CAN0[i]);
		
		while(usart_flag_get(USART1,USART_FLAG_TC)==RESET);	//�ȴ��������
//		data_ok=1;
	}
}
if(0x06==type){					//CAN1���ݷ��͸�USART1
	for(i=0;i<len;i++)
	{
		usart_data_transmit (USART1,senddata_CAN1[i]);
		
		while(usart_flag_get(USART1,USART_FLAG_TC)==RESET);	//�ȴ��������
//		data_ok=1;
	}
}
	data_ok=1;	//���ݷ�����ɱ�־λ��1
}

/*�������������ݷ��͸�CAN*/
void send_to_can(void)
{
	int i;
	if(SET==usart_rx_to_can_flag)		//422�������� ���յ����� usart_rx_to_can_flag==SET/û����RESET
	{
		
		for(i=0;i<8;i++)
		{
			usart1_receive[i]=data[i];		//ת��422����
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
/*����1��ʼ��*/
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
    usart_parity_config(USART1, USART_PM_NONE);	//У��λ
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
		nvic_irq_enable(USART1_IRQn,0,3);//�����ж�ʹ��
//		usart_interrupt_flag_clear(USART1,USART_INT_RBNE);
//		usart_interrupt_flag_clear(USART1,USART_INT_IDLE);

		usart_interrupt_enable(USART1,USART_INT_RBNE);	/*!< read data buffer not empty interrupt and overrun error interrupt */
	//	usart_interrupt_enable(USART1,USART_INT_IDLE);

}

/*����3��ʼ��*/
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
    can_parameter.time_triggered = DISABLE;						//ʱ�䴥��ͨ��ģʽ
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
    can_parameter.time_triggered = DISABLE;						//ʱ�䴥��ͨ��ģʽ
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
	//����������ӳ�䣺CAN1_RX/PB8,CAN1_TX/PB9;��ȫ��ӳ�䣺CAN1_RX/PD0,CAN1_TX/PD1;�ر���ӳ�䣺CAN1_RX/PA11,CAN_TX/PA12
//    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP,ENABLE);		//
    
//		gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
//    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
//    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP,ENABLE);
    /* configure CAN1 GPIO */
    gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_12);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13);
	//�ر���ӳ�䣺CAN1_RX/PB12,CAN1_TX/PB13;������ӳ�䣺CAN1_RX/PB5,CAN_TX/PB6
//    gpio_pin_remap_config(GPIO_CAN1_REMAP,ENABLE);		
	
}


/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
    return ch;
}

/*���͵����ַ�*/
void usart_sendByte(uint32_t USARTX, uint8_t ch)
{
	usart_data_transmit(USARTX,ch);
	while(usart_flag_get(USARTX,USART_FLAG_TBE));
	
}

/*�����ַ���*/
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
//		TxMessage.tx_sfid=Id;					// ��׼��ʶ��Ϊ0
//		TxMessage.tx_ff=CAN_FF_STANDARD;			// ʹ�ñ�׼֡��ʶ��
//	}
//	else
//	{
//		TxMessage.tx_efid=Id;			 		// ������չ��ʾ����29λ��
//		TxMessage.tx_ff=CAN_FF_EXTENDED;			// ʹ����չ��ʶ��
//	}

//	
//	TxMessage.tx_ft=CAN_FT_DATA;			// ��Ϣ����Ϊ����֡��һ֡8λ
//	TxMessage.tx_dlen=8;					// ������֡��Ϣ
//	
//	for(i=0;i<8;i++)
//	TxMessage.tx_data[i]=msg[i];			// ��һ֡��Ϣ          
//	mbox= can_message_transmit(can_periph, &TxMessage);   
//	
//	i=0;
//	while((can_flag_get(can_periph, mbox)!=CAN_TRANSMIT_FAILED)&&(i<0XFFF)) i++;	//�ȴ����ͽ���

//	if(i>=0XFFF)return 1;
//	return 0;		

//}


//void Controller_transfer_Data() //����������������
//{

//	int8_t i,j;
//  SWJcount = 0;
//	for ( j = 0; j < 70; j++ )//ʹ��ѭ����������
//		{
//			if((SWJdata[j] == 0x1e) && (SWJdata[j+1] == 0x1e)&& (SWJdata[j+2] != 0x00))//??????????
//			{
//				for ( i = 0; i < 10; i++ )//��������
//				{
//					SWJdata1[i] = SWJdata[j+i];
//				}
//					 if(SWJdata1[2] ==0x0a) //���Ʒ���
//					{
//						reback_flag =1;
//					}
//					else if(SWJdata1[2] ==0x0b)//�豸״̬
//					{
//							 zijian_return = 1;
//							 SWJdata1[2] = 0;
//					}
//					else if(SWJdata1[2] ==0x0c)//����״̬
//					{
//								 work_return = 1;
//					}
//					else if(SWJdata1[2] ==0x0d)//MY����״̬
//					{
//							 MY_return = 1;
//					}
//					else if(SWJdata1[2] ==0x0e)//HY����״̬
//					{
//							HY_return = 1;
//					}
//					else if(SWJdata1[2] ==0x0f)//Уʱ״̬
//					{
//							Time_return = 1;
//					}
//					else if(SWJdata1[2] ==0x10)//������Χ
//					{
//							boshu_flag = 1;
//					}
//				
//					CANsenddata();//������ƽ̨���������
//			}
//		}
//	
//}
//void CANsenddata(void)
//{
//	uint8_t cansendata[8];//����
//	
////		if((zhuji_return == 1)||(boshu_flag == 1))//����������Ϣ
////		{
//			cansendata[0] = 0x01;//���ݵ�1λ��ֵ
//			cansendata[3] = 0;//���ݵ�4λ��ֵ
//			cansendata[4] = SWJdata1[5];//���ݵ�5λ��ֵ
//			cansendata[5] = SWJdata1[6];//���ݵ�6λ��ֵ
//			cansendata[6] = SWJdata1[7];//���ݵ�7λ��ֵ
//			cansendata[7] = SWJdata1[8];//���ݵ�8λ��ֵ
//	
//	
////  Can_Send_Msg ( CAN_1, StandFrame, 1891, cansendata );
//	
//		  if(reback_flag == 1)//���Ʒ���
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0xf0;//���ݵ�3λ��ֵ

//				reback_flag = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		  else if(zijian_return == 1)//�豸״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x40;//���ݵ�3λ��ֵ
//				zijian_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(work_return == 1)//����״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x41;//���ݵ�3λ��ֵ
//				work_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}	
//		 else if(MY_return == 1)//MY״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x42;//���ݵ�3λ��ֵ
//				MY_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(HY_return == 1)//HY״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x43;//���ݵ�3λ��ֵ
//				HY_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(Time_return == 1)//Уʱ״̬
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x44;//���ݵ�3λ��ֵ
//				Time_return = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}
//		 else if(boshu_flag == 1)//������Χ
//			{
//				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
//				cansendata[2] = 0x45;//���ݵ�3λ��ֵ
//				boshu_flag = 0;//����ֵ
//				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
//			}			
//}
//uint8_t Can_Send_Msg(uint32_t can_periph, Frametype datatype, uint32_t Id, uint8_t* msg)
//{	
//	uint8_t mbox;
//	uint16_t i=0;
//	can_trasnmit_message_struct TxMessage;


//	if(datatype == StandFrame)
//	{
//		TxMessage.tx_sfid=Id;					// ��׼��ʶ��Ϊ0
//		TxMessage.tx_ff=CAN_FF_STANDARD;			// ʹ�ñ�׼֡��ʶ��
//	}
//	else
//	{
//		TxMessage.tx_efid=Id;			 		// ������չ��ʾ����29λ��
//		TxMessage.tx_ff=CAN_FF_EXTENDED;			// ʹ����չ��ʶ��
//	}

//	
//	TxMessage.tx_ft=CAN_FT_DATA;			// ��Ϣ����Ϊ����֡��һ֡8λ
//	TxMessage.tx_dlen=8;					// ������֡��Ϣ
//	
//	for(i=0;i<8;i++)
//	TxMessage.tx_data[i]=msg[i];			// ��һ֡��Ϣ          
//	mbox= can_message_transmit(can_periph, &TxMessage);   
//	
//	i=0;
//	while((can_flag_get(can_periph, mbox)!=CAN_TRANSMIT_FAILED)&&(i<0XFFF)) i++;	//�ȴ����ͽ���

//	if(i>=0XFFF)return 1;
//	return 0;		

//}

