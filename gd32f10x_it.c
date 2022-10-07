/*!
    \file    gd32f10x_it.c
    \brief   interrupt service routines

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

#include "gd32f10x_it.h"
#include "systick.h"

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
SWJMingLingtypedef tSWJMingLing;//��������

extern uint32_t time2;//����
extern 	uint8_t senddata[11];//������������
uint8_t R_data_CAN0[8],R_data_CAN1[8],CAN0_flag,CAN1_flag,zhujizijian_flag,fenjizijian_flag,normal_flag,special_flag,test_flag,mode1_flag,mode2_flag,mode3_flag;//����
uint8_t MY1_flag,MY2_flag,MY3_flag,MY4_flag,HY_flag,UTCtime_flag,Datatime_flag,long_flag,ceju_flag,work_flag,MY_flag,dingxiang_flag,weizhi_flag,weizhi_flag_CAN0,weizhi_flag_CAN1,time_flag,zijian_flag,lingwei_flag,tansuo_flag,guiling_flag,Controller_transfer_Data_flag;//����
uint8_t ZIJIAN[8],Daowei_flag,FW_motor,GD_motor,communication_error,zhuji_return,daowei_flag,daowei_stop,SWJdata1[10];//���ñ���
uint8_t time_flag_1,zhujizijian_flag_1,work_flag_1,dingxiang_flag_1,MY_flag_1,HY_flag_1,UTCtime_flag_1,Datatime_flag_1,dataflag_1,weizhi_flag_1,daowei_flag_1,long_flag_1,ceju_flag_1,fenjizijian_flag_1;
uint8_t SWJdata[70], FWencoder_data[10],GDencoder_data[4],SWJdata1[10]; 
uint16_t dataflag, SWJcount=0, FWcount,GDcount;

//extern uint8_t daowei_flag;//����

FlagStatus receive_flag,usart_rx_to_can_flag;

can_receive_message_struct receive_message;
uint8_t zijian_ok,checkwork,normalwork,specialwork,reback_flag,boshu_flag,zijian_return,work_return,MY_return,HY_return,Time_return,data_ok,data_fail,data_recive_ok;

uint8_t QuDongShuJu_1_High,QuDongShuJu_1_Low,QuDongShuJu_2_High,QuDongShuJu_2_Low,QuDongKongZhiZi_1,QuDongKongZhiZi_2,JiaoYanHe;
uint32_t CAN0id,CAN1id;



int i=0;
uint8_t data[9]={0};
uint8_t data_sum=0;
uint8_t data_last=0;
void USART1_IRQHandler(void)
{
	
	if(RESET!=usart_interrupt_flag_get(USART1,USART_INT_FLAG_RBNE))
		{
		
			data[i]=usart_data_receive(USART1);
			
		}

//		usart_data_transmit(USART1, data[i]);
//		while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));//��������ж�
		if(i!=8)
		{data_sum=data_sum+data[i];}
		
		data_last=data[8];
		if((data[0]==0x55)&&(data[1]==0xAA)&&(data_last==data_sum))
			{

				usart_rx_to_can_flag=SET;
				
			}
			i++;
		if(i==9)
		{
			i=0;
			data_sum=0;
			data[8]=0;
		}
////if(SET == usart_flag_get(USART1 ,USART_FLAG_ORERR))
////		{		
////			usart_interrupt_flag_clear(USART1,USART_INT_ERR) ;
//			usart_interrupt_flag_clear(USART1,USART_INT_RBNE);
//		
////		}


//    unsigned char data;
//		if(SET == usart_flag_get(USART1 ,USART_FLAG_ORERR))
//		{		usart_interrupt_flag_clear(USART1,USART_INT_ERR) ;
//				usart_interrupt_flag_clear(USART1,USART_INT_RBNE);
//		
//		}	
//    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
//    {
//        data = usart_data_receive(USART1);
//          
//       usart_data_transmit(USART1, (uint8_t)data);
//        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));//��������ж�
//    
//    }
}
/********************************************************************/
/***************************����������ͨ��*********************************/
void UART3_IRQHandler(void)
{
	/* ?????? */
	uint8_t Clear = Clear,yihuohe;

	if ( usart_flag_get ( UART3, USART_FLAG_RBNE ) != RESET )
	{
//		SWJcount=0;
//		SWJdata[SWJcount] = usart_data_receive ( UART3 );
		SWJdata[SWJcount++] = usart_data_receive ( UART3 );
//		SWJcount++;
		if(SWJcount == 10)
		{
		 Controller_transfer_Data();	//���ͽ��յ������ݸ������
		
		}
	}

//		else if ( usart_flag_get ( UART3, USART_FLAG_IDLEF ) != RESET )
//	{
//		usart_interrupt_flag_clear(UART3,USART_INT_ERR) ;
//		usart_interrupt_flag_clear(UART3,USART_INT_RBNE);
//	  Controller_transfer_Data();
//		SWJcount = 0;
//	}  
}

void Controller_transfer_Data() //����������������
{

	int8_t i,j;
  SWJcount = 0;
	for ( j = 0; j < 70; j++ )//ʹ��ѭ����������
		{
			if((SWJdata[j] == 0x1e) && (SWJdata[j+1] == 0x1e)&& (SWJdata[j+2] != 0x00))//�жϸ�ʽ�Ƿ���Ч
			{
				for ( i = 0; i < 10; i++ )//��������
				{
					SWJdata1[i] = SWJdata[j+i];
				}
					 if(SWJdata1[2] ==0x0a) //���Ʒ���
					{
						reback_flag =1;
					}
					else if(SWJdata1[2] ==0x0b)//�豸״̬
					{
							 zijian_return = 1;
							 SWJdata1[2] = 0;
					}
					else if(SWJdata1[2] ==0x0c)//����״̬
					{
								 work_return = 1;
					}
					else if(SWJdata1[2] ==0x0d)//MY����״̬
					{
							 MY_return = 1;
					}
					else if(SWJdata1[2] ==0x0e)//HY����״̬
					{
							HY_return = 1;
					}
					else if(SWJdata1[2] ==0x0f)//Уʱ״̬
					{
							Time_return = 1;
					}
					else if(SWJdata1[2] ==0x10)//������Χ
					{
							boshu_flag = 1;
					}
				
					CANsenddata();//������ƽ̨���������
			}
		}
	
}
void CANsenddata(void)
{
	uint8_t cansendata[8];//����
	
//		if((zhuji_return == 1)||(boshu_flag == 1))//����������Ϣ
//		{
			cansendata[0] = 0x01;//���ݵ�1λ��ֵ
			cansendata[3] = 0;//���ݵ�4λ��ֵ
			cansendata[4] = SWJdata1[5];//���ݵ�5λ��ֵ
			cansendata[5] = SWJdata1[6];//���ݵ�6λ��ֵ
			cansendata[6] = SWJdata1[7];//���ݵ�7λ��ֵ
			cansendata[7] = SWJdata1[8];//���ݵ�8λ��ֵ
	
	
//  Can_Send_Msg ( CAN_1, StandFrame, 1891, cansendata );
	
		  if(reback_flag == 1)//���Ʒ���
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0xf0;//���ݵ�3λ��ֵ

				reback_flag = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}
		  else if(zijian_return == 1)//�豸״̬
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0x40;//���ݵ�3λ��ֵ
				zijian_return = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}
		 else if(work_return == 1)//����״̬
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0x41;//���ݵ�3λ��ֵ
				work_return = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}	
		 else if(MY_return == 1)//MY״̬
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0x42;//���ݵ�3λ��ֵ
				MY_return = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}
		 else if(HY_return == 1)//HY״̬
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0x43;//���ݵ�3λ��ֵ
				HY_return = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}
		 else if(Time_return == 1)//Уʱ״̬
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0x44;//���ݵ�3λ��ֵ
				Time_return = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}
		 else if(boshu_flag == 1)//������Χ
			{
				cansendata[1] = 0x0a;//���ݵ�2λ��ֵ
				cansendata[2] = 0x45;//���ݵ�3λ��ֵ
				boshu_flag = 0;//����ֵ
				Can_Send_Msg ( CAN0, StandFrame, 1891, cansendata );//����
			}			
}
uint8_t Can_Send_Msg(uint32_t can_periph, Frametype datatype, uint32_t Id, uint8_t* msg)
{	
	uint8_t mbox;
	uint16_t n=0;
	can_trasnmit_message_struct TxMessage;


	if(datatype == StandFrame)
	{
		TxMessage.tx_sfid=Id;					// ��׼��ʶ��Ϊ0
		TxMessage.tx_ff=CAN_FF_STANDARD;			// ʹ�ñ�׼֡��ʶ��
	}
	else
	{
		TxMessage.tx_efid=Id;			 		// ������չ��ʾ����29λ��
		TxMessage.tx_ff=CAN_FF_EXTENDED;			// ʹ����չ��ʶ��
	}

	
	TxMessage.tx_ft=CAN_FT_DATA;			// ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.tx_dlen=8;					// ������֡��Ϣ
	
	for(n=0;n<8;n++)
	TxMessage.tx_data[n]=msg[n];			// ��һ֡��Ϣ          
	mbox= can_message_transmit(can_periph, &TxMessage);   
	
	n=0;
	while((can_flag_get(can_periph, mbox)!=CAN_TRANSMIT_FAILED)&&(n<0XFFF)) n++;	//�ȴ����ͽ���

	if(n>=0XFFF)return 1;
	return 0;		

}

void CAN0_RX0_IRQHandler(void)
{
	
	can_receive_message_struct RxMessage;
	
	uint8_t i = 0;
//	uint8_t j = 0;
//	can_trasnmit_message_struct Tx_Msg_1891;
//	Tx_Msg_1891.tx_ff=0;
//	Tx_Msg_1891.tx_efid=1891;
//	Tx_Msg_1891.tx_ff = CAN_FF_EXTENDED;
//	
	can_message_receive( CAN0, CAN_FIFO0, &RxMessage );//��ȡcan
	CAN0id = RxMessage.rx_sfid;//��׼֡ID
//	CAN0id = RxMessage.rx_efid;//��չ֡ID

	for ( i=0; i < 8; i++ )//ͨ��ѭ����������
	{
		R_data_CAN0[i] = RxMessage.rx_data[i];
	}
	if(1810==CAN0id)///ȷ������
	{
		if(R_data_CAN0[2] == 0xf0)//ʱ��
		{
      time_flag = 1;//״̬
		}
		usart_data_transmit(UART3,1810);
	}
	else if(1890==CAN0id)//ȷ������
	{
		if(R_data_CAN0[2] == 0x01)//�Լ�       
		{
      zhujizijian_flag = 1;
		}
		else if(R_data_CAN0[2] == 0x02)//����״̬
		{
			work_flag = 1;
			
		}
		else if(R_data_CAN0[2] == 0x03)//����ѯ��
		{
			dingxiang_flag = 1;
		}
		else if(R_data_CAN0[2] == 0x04)//MY�л�
		{
			MY_flag = 1;
		}
		Can_Send_Msg ( CAN0, StandFrame, 1891, R_data_CAN0 );//��������
		
//	can_message_transmit ( CAN0, &Tx_Msg_1891 );//�������ݸ������
//	while(RESET==can_flag_get(CAN0,CAN_FLAG_TME0));
	}
	else if(1800==CAN0id)//ȷ������
	{
		if(R_data_CAN0[2] == 0xA5)//HY
		{
			HY_flag = 1;
		}
		Can_Send_Msg ( CAN0, StandFrame, 1891, R_data_CAN0 );//��������
//		for ( j=0; j < 8; j++ )//ͨ��ѭ����������
//		{
//			Tx_Msg_1891.tx_data[j]=R_data_CAN0[j];
//		}
//		can_message_transmit ( CAN0, &Tx_Msg_1891 );//��������
//		while(RESET==can_flag_get(CAN0,CAN_FLAG_TME0));
	}
	else if(1524==CAN0id)//ʱ����//ȷ������
	{
		UTCtime_flag = 1;
	}
	else if(1525==CAN0id)//������//ȷ������
	{
		Datatime_flag = 1;
	}
	else if(1815==CAN0id)//��λĿ��//ȷ������
	{
		if(R_data_CAN0[2] == 0x05)//��λĿ��
		{
		  dataflag = 1;
			weizhi_flag = 1;
			daowei_flag = 1;
			tSWJMingLing = ZhuLiu;
//			zhuliuFW = ( int32_t )(R_data_CAN1[4] + (R_data_CAN1[5] << 8));
//			zhuliuGD = ( int32_t )(R_data_CAN1[6] + (R_data_CAN1[7] << 8));
		}
		else if(R_data_CAN0[2] == 0x13)//��γ��
		{
       long_flag =1;//״̬
		}
		else if(R_data_CAN0[2] == 0x0d)//���
		{
			if(((R_data_CAN0[4]!=0)&&(R_data_CAN0[5]!=0)&&(R_data_CAN0[6]!=0)&&(R_data_CAN0[7]!=0))||((R_data_CAN0[4]!=0xff)&&(R_data_CAN0[5]!=0xff)&&(R_data_CAN0[6]!=0xff)&&(R_data_CAN0[7]!=0xff)))//ȷ������
			{
				ceju_flag = 1;//״̬
			}
			else ceju_flag = 0;//״̬
		}
	}
	
	else if(1234==CAN0id)//��λĿ��//ȷ������
	{
		if(R_data_CAN0[0] == 0x05)//��λĿ��
		{
		  dataflag = 1;	//��USART�ж���
			weizhi_flag_CAN0 = 1;		//ȷ��CAN0���յ�λ������
			daowei_flag = 1;
			tSWJMingLing = ZhuLiu;	//��MAIN�ж���
//			zhuliuFW = ( int32_t )(R_data_CAN0[4] + (R_data_CAN0[5] << 8));
//			zhuliuGD = ( int32_t )(R_data_CAN0[6] + (R_data_CAN0[7] << 8));
		}
	}
	else if(1816==CAN0id)//��λ����
	{
		if(R_data_CAN0[0] == 0x11)//
		{
		  lingwei_flag=1;

		}
	}
	else if(1817==CAN0id)//̽��ģʽ���ޣ�
	{
		if(R_data_CAN0[0] == 0x12)//
		{
		  tansuo_flag=1;

		}
	}
	else if(1818==CAN0id)//�Ƕȹ���
	{
		if(R_data_CAN0[0] == 0x13)//
		{
		  guiling_flag=1;

		}
	}
		
		
		
		

//	else if(1234==CAN0id)
//	{
//		dataflag = 1;
//    tSWJMingLing = ( SWJMingLingtypedef )R_data_CAN0[0];//ִ������
//	}
//    /* check the receive message */
//    can_message_receive(CAN0, CAN_FIFO0, &receive_message);
//    if((0x1810 == receive_message.rx_sfid) && (8 == receive_message.rx_dlen)&&(0xf0==receive_message.rx_data[2]))
//    {time_flag = SET;}
//		if((0x1890 == receive_message.rx_sfid) && (8 == receive_message.rx_dlen)&&(0x01==receive_message.rx_data[2]))
//		{zijian_flag=SET;}
//		if((0x1890 == receive_message.rx_sfid) && (8 == receive_message.rx_dlen)&&(0x02==receive_message.rx_data[2]))
//		{zijian_flag=SET;}
    
}

void CAN1_RX1_IRQHandler(void)
{
	can_receive_message_struct RxMessage_1;
	
	uint8_t i = 0;
//	uint8_t j = 0;
//	can_trasnmit_message_struct Tx_Msg_1891;
//	Tx_Msg_1891.tx_ff=0;
//	Tx_Msg_1891.tx_sfid=1891;
	//transmit_message_1891.tx_data=R_data_CAN0;
	
	can_message_receive( CAN1, CAN_FIFO1, &RxMessage_1 );//��ȡcan
	CAN1id = RxMessage_1.rx_sfid;//��׼֡ID
//	CAN1id = RxMessage_1.rx_efid;//��չ֡ID

	for ( i=0; i < 8; i++ )//ͨ��ѭ����������
	{
		R_data_CAN1[i] = RxMessage_1.rx_data[i];
	}
	
		if(1810==CAN1id)///ȷ������
	{
		if(R_data_CAN1[2] == 0xf0)//ʱ��
		{
      time_flag_1 = 1;//״̬
		}
		usart_data_transmit(UART3,1810);
	}
	else if(1890==CAN1id)//ȷ������
	{
		if(R_data_CAN1[2] == 0x01)//�Լ�       
		{
      zhujizijian_flag_1 = 1;
		}
		else if(R_data_CAN1[2] == 0x02)//����״̬
		{
			work_flag_1 = 1;
			
		}
		else if(R_data_CAN1[2] == 0x03)//����ѯ��
		{
			dingxiang_flag_1 = 1;
		}
		else if(R_data_CAN1[2] == 0x04)//MY�л�
		{
			MY_flag_1 = 1;
		}
		Can_Send_Msg ( CAN1, StandFrame, 1891, R_data_CAN1 );//��������
		
//	can_message_transmit ( CAN0, &Tx_Msg_1891 );//�������ݸ������
//	while(RESET==can_flag_get(CAN0,CAN_FLAG_TME0));
	}
	else if(1800==CAN1id)//ȷ������
	{
		if(R_data_CAN1[2] == 0xA5)//HY
		{
			HY_flag_1 = 1;
		}
		Can_Send_Msg ( CAN1, StandFrame, 1891, R_data_CAN1 );//��������
//		for ( j=0; j < 8; j++ )//ͨ��ѭ����������
//		{
//			Tx_Msg_1891.tx_data[j]=R_data_CAN0[j];
//		}
//		can_message_transmit ( CAN0, &Tx_Msg_1891 );//��������
//		while(RESET==can_flag_get(CAN0,CAN_FLAG_TME0));
	}
	else if(1524==CAN1id)//ʱ����//ȷ������
	{
		UTCtime_flag_1 = 1;
	}
	else if(1525==CAN1id)//������//ȷ������
	{
		Datatime_flag_1 = 1;
	}
	else if(1815==CAN1id)//��λĿ��//ȷ������
	{
		if(R_data_CAN1[2] == 0x05)//��λĿ��
		{
		  dataflag_1 = 1;
			weizhi_flag_1 = 1;
			daowei_flag_1 = 1;
			tSWJMingLing = ZhuLiu;
//			zhuliuFW = ( int32_t )(R_data_CAN1[4] + (R_data_CAN1[5] << 8));
//			zhuliuGD = ( int32_t )(R_data_CAN1[6] + (R_data_CAN1[7] << 8));
		}
		else if(R_data_CAN1[2] == 0x13)//��γ��
		{
       long_flag_1 =1;//״̬
		}
		else if(R_data_CAN1[2] == 0x0d)//���
		{
			if(((R_data_CAN1[4]!=0)&&(R_data_CAN1[5]!=0)&&(R_data_CAN1[6]!=0)&&(R_data_CAN1[7]!=0))||((R_data_CAN1[4]!=0xff)&&(R_data_CAN1[5]!=0xff)&&(R_data_CAN1[6]!=0xff)&&(R_data_CAN1[7]!=0xff)))//ȷ������
			{
				ceju_flag_1 = 1;//״̬
			}
			else ceju_flag_1 = 0;//״̬
		}
	}
	
		else if(1234==CAN1id)//��λĿ��//ȷ������
	{
		if(R_data_CAN1[0] == 0x05)//��λĿ��
		{
		  dataflag = 1;	//��USART�ж���
			weizhi_flag_CAN1 = 1;
			daowei_flag = 1;
			tSWJMingLing = ZhuLiu;	//��MAIN�ж���
//			zhuliuFW = ( int32_t )(R_data_CAN0[4] + (R_data_CAN0[5] << 8));
//			zhuliuGD = ( int32_t )(R_data_CAN0[6] + (R_data_CAN0[7] << 8));
		}
	}
	else if(1816==CAN1id)//��λ����
	{
		if(R_data_CAN1[0] == 0x11)//
		{
		  lingwei_flag=1;

		}
	}
	else if(1817==CAN1id)//̽��ģʽ
	{
		if(R_data_CAN1[0] == 0x12)//
		{
		  tansuo_flag=1;

		}
	}
	else if(1818==CAN1id)//�Ƕȹ���
	{
		if(R_data_CAN1[0] == 0x13)//
		{
		  guiling_flag=1;

		}
	}
}

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}


