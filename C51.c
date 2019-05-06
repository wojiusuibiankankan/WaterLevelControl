#include<reg52.h>				 //ͷ�ļ�
#include<intrins.h>
#include"eeprom52.h"			 //STC89C52 EEPROM   �����ļ�
#define uchar unsigned char		 //�궨��
#define uint unsigned int

#define LCD1602_dat P0	 //LCD1602���ݿں궨��


sbit LCD1602_rs=P2^5;		//LCD1602��������IO��
sbit LCD1602_rw=P2^6;
sbit LCD1602_e=P2^7;
sbit beep=P1^3;		//������  IO
sbit led_1=P1^4;		//LEDָʾ��  IO
sbit led_2=P1^6;
sbit key_1=P3^2;		//ϵͳ���ư���IO��
sbit key_2=P3^3;
sbit key_3=P3^4;
sbit alarm_1=P2^0;		//���Ƽ̵���IO��//


sbit ADC0832_CS=P1^2;	//ADC0832  ����IO��	  ʹ�ܿ�
sbit ADC0832_CLK=P1^1;	//ʱ��IO��
sbit ADC0832_DIO=P1^0;	//�����������IO�� 

uint sum, waterCount;			  //10��ADֵ���ۺϱ���,ˮ��ͳ��
uchar RH_H=12,RH_L=8,state,ms,cs,motorTimeCount,aimLevel;  //��ǰˮλ��  ˮλ���ޣ����ޣ�  �����������50ms����   ��cs Ϊ�ƴ�������    ,���ʱ�����,��ˮ����
bit beep1,s1,overLimit,pidFlag;	  //������־λ�� ��˸��־λ ,��ˮ������־��pid���б�־

	static float ki = 0.3;//����ϵ��

int motorRunTime = -1;//��Ϊ-1��������������0����������

uchar pidCount,pidTime;

uchar zt;//ģʽ��־
float motorKP;//����ϵ��
float RH;//��ǰˮλ
float Ad_dat,Ad_datN;
 

/*
    ADC0832��һ��8λAdоƬ����Ϊ��Ƭ������ֱ�Ӵ���ģ���źţ���ѹ�������Ե�Ƭ�����ѹ��ʱ����������Ⱦ���һ��ģ��ת��оƬ����ģ����
ת������������Ȼ����ADC0832�����ĵ�ѹ��Χ��0-5V�����ܹ���0��5V�ĵ�ѹת���ɶ�Ӧ������ϵ��0-255��8λ��0-255�������ݣ���Ƭ��ֱ
�Ӷ�ȡADC0832�����ݻ�ȡADֵ���ݣ�Ȼ����Ϊ0-5V��Ӧ0-255���ݣ�����1V���Ӧ��ADֵ����51���ͻ������¹�ʽ

  ��ѹ=ADֵ/51��

�����ѵ�ѹ���ݾ�ȷ��С�����һλ����   ��ѹ=ADֵ/5.1��
					  С�������λ����   ��ѹ=ADֵ/0.51��

��Ҫ����Ϊʲô������ѧ��Сѧ�������㡣
 
*/
/********************************************************************
* ���� : delay()
* ���� : С��ʱ��													 
* ���� : ��
* ��� : ��
***********************************************************************/




void delay(uint T)					  //��ʱ����
{
	while(T--);
}
void saveWaterCount()
{
	SectorErase(0x2400);	 //��������ֵ  ���浽��Ƭ����EEPROM
	byte_write(0x2400,waterCount);
}

void openMotorTime(uchar time)//�������time��
{
	motorRunTime = time;
}

uint motorControl(uchar flag)//��=1����=0���л�=2,ʲô���������һ�ȡ��ǰ״̬=3
{
	bit a;
	if(flag == 0)//��
	{
		alarm_1 = 1;
		saveWaterCount();
	}
	else if(flag == 1&&alarm_1 == 1)//��
	{
		alarm_1 = 0;
	}
	else if(flag == 2)
	{
		alarm_1 = !alarm_1;
	//	if(alarm_1 == 1)
		//	saveWaterCount();
	}
	else if(flag == 3)
	{
		return !alarm_1;
	}
	delay(500);
	return 2;
}
unsigned int  A_D()	    //ADC0832   ��ֵ����
{
	unsigned char i;
	unsigned char dat;					 
	ADC0832_CS=1;   //һ��ת�����ڿ�ʼ
	ADC0832_CLK=0;  //Ϊ��һ��������׼��
	ADC0832_CS=0;  //CS��0��Ƭѡ��Ч
	ADC0832_DIO=1;    //DIO��1���涨����ʼ�ź�  
	ADC0832_CLK=1;   //��һ������
	ADC0832_CLK=0;   //��һ��������½��أ���ǰDIO�����Ǹߵ�ƽ
	ADC0832_DIO=1;   //DIO��1�� ͨ��ѡ���ź�  
	ADC0832_CLK=1;   //�ڶ������壬��2��3�������³�֮ǰ��DI�������������λ��������ѡ��ͨ��������ѡͨ��RH0 
	ADC0832_CLK=0;   //�ڶ��������½��� 
	ADC0832_DIO=0;   //DI��0��ѡ��ͨ��0
	ADC0832_CLK=1;    //����������
	ADC0832_CLK=0;    //�����������½��� 
	ADC0832_DIO=1;    //�����������³�֮�������DIOʧȥ���ã�Ӧ��1
	ADC0832_CLK=1;    //���ĸ�����
	for(i=0;i<8;i++)  //��λ��ǰ
	{
		ADC0832_CLK=1;         //���ĸ�����
		ADC0832_CLK=0; 
		dat<<=1;       //�����洢��ĵ�λ����������
		dat|=(unsigned char)ADC0832_DIO; 	 //���������DIOͨ�������㴢����dat���λ 
	}	  		        
	ADC0832_CS=1;          //Ƭѡ��Ч 
	return dat;	 //����������ݷ���     
}


/*
    1602Һ�����ǳ��õ���ʾ������һ����16���ܽţ������а˸��ܽ������ݴ���ܽţ��������ܽ�����������ʹ�ܶ˹ܽţ����������Դ�ܽţ�
����һ���Դ�ܽ��Ǹ�����Һ�����й���ģ�����һ���Դ�ǵ����ı������Դ����ʣ�µ����һ���ܽ��ǶԱȶȵ��ڹܽţ�һ�����һ��3K��
���ٽӵؼ��ɡ�
 
*/


/********************************************************************
* ���� : LCD1602_write(uchar order,dat)
* ���� : 1602д�����ݺ���
* ���� : ���������ֵ
* ��� : ��
***********************************************************************/
void LCD1602_write(uchar order,dat)				  //1602 һ���ֽ�  ����
{
    LCD1602_e=0;//ʹ���źţ�1��ȡ��Ϣ���½���ִ��ָ���������
    LCD1602_rs=order;//rs,0����ָ�1��������
    LCD1602_dat=dat;//P0��
    LCD1602_rw=0;//read/write,0д1��
    LCD1602_e=1;
    delay(1);
    LCD1602_e=0;																								     
}
/********************************************************************
* ���� : LCD1602_writebye(uchar *prointer)
* ���� : 1602д�����ݺ���  ָ��ʽ
* ���� : ���������ֵ
* ��� : ��
***********************************************************************/
void LCD1602_writebyte(uchar *prointer)				   //1602 �ַ���    ����
{
    while(*prointer!='\0')
    {
        LCD1602_write(1,*prointer);
        prointer++;
    }
}
/********************************************************************
* ���� : LCD1602_cls()
* ���� : ��ʼ��1602Һ�� 
* ���� : ��
* ��� : ��
***********************************************************************/
void LCD1602_cls()									 //1602 ��ʼ��
{
	LCD1602_write(0,0x01);     //1602 ���� ָ��
	delay(1500);
	LCD1602_write(0,0x38);     // �������� 8λ��5*7����001(ָ���ʶ������Ϊ1)��1(���ݽӿ�λ����1=8��=4)�� 1(��������)000��
	delay(1500);
	LCD1602_write(0,0x0c);     //ԭ0x0c,���� ���  00001100 �����λ��ʶ����4���붨��Ϊ1������ʾ���ء�����ʾ��ꡢ�ַ�����˸
	LCD1602_write(0,0x06);		//ԭx06,0000 0110 ����ģʽ����ָ���2 д�����ݺ�����/���ƣ���1��ʾ�ƶ�/����ʾ
	LCD1602_write(0,0xd0);    
	delay(1500);
}
/********************************************************************
* ���� : show()
* ���� : LCD1602Һ����ʾ���� 
* ���� : ��
* ��� : ��
***********************************************************************/
 
/*
������ʾ��ʱ��һ��Ĵ���

    ���ȣ��������������ʾ����Һ����ʾ��������ʾ��ʱ����Զ���һ��һ��������ʾ�ģ���ô������˵һ������123��һ�ٶ�ʮ����
������ʾ��ʱ��Ҫ����ʾ1��Ȼ����2��Ȼ����3����ô��ô��������ȡ��������   
��ȡ��λ    123/100=1
��ȡʮλ    123/10=12      12%10=2     ��%����ȡ�����˼�������������12��10ȡ�࣬���仰˵��12����10��Ȼ��ȡ����������2
��ȡ��λ    123%10=3       ����ͬ��

ȡ����÷�Ҳ�кܶ��֣����ֻҪ֪�����������ʱ��һ�㶼�ǽ���������ȡ�ľ���


Ȼ��
��������������ʾ���ݣ�����ȡ�����ݷŵ��������������͸�IO���ɣ�
�����Һ����ʾ����Ҫ������ת�����ַ�����ΪҺ�����ַ�����ֻ����ʾ�ַ����ݣ�����0��Ӧ���ַ���0x30������1��Ӧ���ַ���0x31��
���Խ���ȡ��������ֱ�Ӽ���0x30�͸�Һ�����ɣ����߼���'0' Ҳ��һ���� 


 
*/


void show()
{
	if(state==0)		//��ǰˮλ������ģʽ��ʾ,��ˮ������ʾ
	{
		
		LCD1602_write(0,0x80);//����DDRAM�Դ��ַָ�Ч�������������ù�꣬1+xxx,xxxx ��λ����1��ʣ�¼�λ��ʶ��ַ�����Ϊ0ʱ����һ�У�Ϊ1ʱ��2��//
		LCD1602_writebyte("WaterLevel:");	//��ǰˮλ
		LCD1602_write(0,0x80+11);
		if(RH>=10)LCD1602_write(1,0x30+((int)RH)/10);//dat��rh
		else LCD1602_writebyte(" ");
		LCD1602_write(0,0x80+12);
		LCD1602_write(1,0x30+((int)RH)%10);
		LCD1602_write(0,0x80+13);
		LCD1602_writebyte(".");
		LCD1602_write(1,0x30+((int)(RH*10))%10);
		//LCD1602_writebyte("cm");

		if(0)//overLimit == 1),������ͳ�ƹ��ܣ���д�ˣ���ɾ
		{
			LCD1602_write(0,0xC0);	LCD1602_writebyte("WARN:Over Using");	  //������ˮ
		}
		else
		{
			LCD1602_write(0,0xC0);	
			if(zt==0)
			{
				LCD1602_writebyte("State:Manul     ");
				
			}else if(zt==1)
			{
				LCD1602_writebyte("State:auto      ");
				if(RH>RH_H)	   //�����ǰˮλֵ�ﵽˮλ����ֵ��
				{
					 
					LCD1602_write(0,0xC0);	
					LCD1602_writebyte("Over Toplimit");	
					
				}
				else	if(RH<=RH_L)	   //�����ǰˮλֵ����ˮλ����ֵ��
				{	
					LCD1602_writebyte("Over LowerLimit");	
				}else	    //���� 
				{
					LCD1602_writebyte("State:Auto     ");
				}

			}
			else if(zt == 3)
			{
				LCD1602_writebyte("State:PidNormal  ");
			}
			else if(zt == 2)
			{
				LCD1602_writebyte("State:PIDAuto   ");
			}
		}
			
	}else if(state == 1 || state == 2)	  //ˮλ���������ý��棬
	{
		LCD1602_write(0,0x80);
		LCD1602_writebyte("Water_H:");   //ˮλ���ޣ�state == 1
		LCD1602_write(0,0x80+8);
		
		if(state==1&&s1==1)		   //ͨ����˸��־Ϊ  �ﵽ��˸��Ч��,250ms����һ��
		{
			LCD1602_writebyte("  ");
		}else
		{	LCD1602_write(0,0x80+8);
			if(RH_H>9)LCD1602_write(1,0x30+RH_H/10%10);
			else LCD1602_writebyte(" ");
			LCD1602_write(0,0x80+9);
			LCD1602_write(1,0x30+RH_H%10);		
		}
		
		LCD1602_write(0,0x80+10);
		LCD1602_writebyte("cm    ");

		LCD1602_write(0,0xC0);
		LCD1602_writebyte("Water_L:");  //ˮλ����
		if(state==2&&s1==1)		 //ͨ����˸��־Ϊ  �ﵽ��˸��Ч��
		{
			LCD1602_write(0,0xC0+8);
			LCD1602_writebyte("  "); 
		}else
		{
			LCD1602_write(0,0xC0+8);
			if(RH_L>9)LCD1602_write(1,0x30+RH_L/10%10);
			else LCD1602_writebyte(" ");
			LCD1602_write(0,0xC0+9);
			LCD1602_write(1,0x30+RH_L%10);
		}
		LCD1602_write(0,0xC0+10);
		LCD1602_writebyte("cm    ");	
	}
	else if(state == 3)
	{
		LCD1602_write(0,0x80);//����DDRAM�Դ��ַָ�Ч�������������ù�꣬1+xxx,xxxx ��λ����1��ʣ�¼�λ��ʶ��ַ�����Ϊ0ʱ����һ�У�Ϊ1ʱ��2��//
	/*	LCD1602_writebyte("Count:");	//��ǰˮλ
		LCD1602_write(0,0x80+6);
		LCD1602_write(1,0x30+waterCount/1000%10);
		LCD1602_write(0,0x80+7);
		LCD1602_write(1,0x30+waterCount/100%10);
		LCD1602_write(0,0x80+8);
		LCD1602_write(1,0x30+waterCount/10%10);
		LCD1602_write(0,0x80+9);
		LCD1602_write(1,0x30+waterCount%10);
		LCD1602_write(0,0x80+10);
		LCD1602_writebyte("Ton");
		LCD1602_write(0,0xC0);	
		LCD1602_writebyte("Press k2 reset");
		*/
		LCD1602_write(0,0x80);
		LCD1602_write(1,0x30+((int)motorKP)/10);
		LCD1602_write(0,0x81);
		LCD1602_write(1,0x30+((int)motorKP)%10);
		LCD1602_writebyte(".");
		LCD1602_write(0,0x80+3);
		LCD1602_write(1,0x30+((int)(motorKP*10))%10);
		LCD1602_writebyte("  ");
		LCD1602_write(0,0x80+6);
		LCD1602_write(1,0x30+((uint)ki)/10);
		LCD1602_write(0,0x80+7);
		LCD1602_write(1,0x30+((uint)ki)%10);
		LCD1602_writebyte(".");
		LCD1602_write(0,0x80+9);
		LCD1602_write(1,0x30+((uint)(ki*10))%10);
		LCD1602_write(0,0x80+10);
		LCD1602_write(1,0x30+((uint)(ki*100))%10);
		LCD1602_writebyte("   "); 
		
		
		LCD1602_write(0,0xC0);
		LCD1602_writebyte("PID_KI:");  //ˮλ����
		if(state==3&&s1==1)		 //ͨ����˸��־Ϊ  �ﵽ��˸��Ч��
		{
			LCD1602_write(0,0xC0+7);
			LCD1602_writebyte("       "); 
		}else
		{
			LCD1602_write(0,0xC0+7);
			LCD1602_writebyte(" ");
			LCD1602_write(0,0xC0+8);
			LCD1602_write(1,0x30+((uint)ki)%10);
			LCD1602_writebyte(".");
			LCD1602_write(0,0xC0+10);
			LCD1602_write(1,0x30+((uint)(ki*10))%10);
			LCD1602_write(0,0xC0+11);
			LCD1602_write(1,0x30+((uint)(ki*100))%10);
		}
		

		
	}
	else if(state == 4 || state == 5)	  //����ٶ����ý��棬
	{
		LCD1602_write(0,0x80);
		LCD1602_writebyte("Motor_K:");   //����ϵ����state == 1
		LCD1602_write(0,0x80+8);
		
		if(state==4&&s1==1)		   //ͨ����˸��־Ϊ  �ﵽ��˸��Ч��,250ms����һ��
		{
			LCD1602_writebyte("        ");
		}else
		{	
			LCD1602_write(1,0x30+((int)motorKP)/10);
			LCD1602_write(0,0x80+9);
			LCD1602_write(1,0x30+((int)motorKP)%10);
			LCD1602_writebyte(".");
			LCD1602_write(0,0x80+11);
			LCD1602_write(1,0x30+((int)(motorKP*10))%10);
		}
		

		LCD1602_write(0,0xC0);
		LCD1602_writebyte("AimLevel:");  //��ˮ����
		if(state==5&&s1==1)		 //ͨ����˸��־Ϊ  �ﵽ��˸��Ч��
		{
			LCD1602_write(0,0xC0+9);
			LCD1602_writebyte("   "); 
		}else
		{
			LCD1602_write(0,0xC0+9);
			if(aimLevel>9)LCD1602_write(1,0x30+aimLevel/10%10);
			else LCD1602_writebyte(" ");
			LCD1602_write(0,0xC0+10);
			LCD1602_write(1,0x30+aimLevel%10);
		}
		LCD1602_write(0,0xC0+11);
		LCD1602_writebyte(" Ton");	
	}
}
/********************************************************************
* ���� : key()
* ���� : �������Ƴ���     ʵ��ϵͳ�������ƹ��� 
* ���� : ��
* ��� : ��
***********************************************************************/
void key()
{
	if(!key_1) //���ð���  ����  ���ܣ��л���ʾ�����õ�ѡ��
	{
		delay(888);   //����ȥ��
		if(!key_1)	//�ٴ��жϰ���
		{
			while(!key_1) show();  //�����ж��Ƿ��ͷ�
			state=(state+1)%6;	  //ִ�а�������   �л�������,�������ù���
		}
	}

	if(!key_2)   //�л�������ֵ�Ĵ�С    ��������ģʽ���л�ϵͳ����ģʽ����
	{
		delay(888);   //����ȥ��
		if(!key_2)   //�ٴ��жϰ���
		{
			while(!key_2)show();  //�����ж��Ƿ��ͷ�
			if(state==1)		  //ִ�а�������   �л�������
			{
				if(RH_H<100)RH_H++;		   //��������ֵ
				SectorErase(0x2000);	 //��������ֵ	 ���浽��Ƭ����EEPROM
				byte_write(0x2000,RH_H);
			}else if(state==2)
			{
				if(RH_L<RH_H-1)RH_L++;	  //��������ֵ
				SectorErase(0x2200);	 //��������ֵ	 ���浽��Ƭ����EEPROM
				byte_write(0x2200,RH_L);
			}else if(state==3)
			{
				if(ki < 20)ki+=1;	  //��������ֵ
				SectorErase(0x2400);	 //���浽��Ƭ����EEPROM
				byte_write(0x2400,ki);
				
			}else if(state==4)
			{
				if(motorKP < 9)motorKP++;	  //��������ֵ
				SectorErase(0x2600);	 //���浽��Ƭ����EEPROM
				byte_write(0x2600,motorKP);
			}else if(state==5)
			{
				if(aimLevel < 99)aimLevel++;	  //��������ֵ
				SectorErase(0x2800);	 //	 ���浽��Ƭ����EEPROM
				byte_write(0x2800,aimLevel);
			}
			else if(state == 0)
			{

				zt=(zt+1)%4;				 //�л�ϵͳ�Ĺ���ģʽ   �Զ�   �ֶ�
				motorRunTime = 0;
				motorControl(0);
				
				//delay(500);
			}	
		}
	}

	if(!key_3)    //�л�������ֵ�Ĵ�С    ���ֶ�ģʽ���л�ϵͳ����״̬�����ء�
	{
		delay(888);	//����ȥ��
		if(!key_3)    //�ٴ��жϰ���
		{		
			while(!key_3)show();  //�����ж��Ƿ��ͷ�
			if(state==1)		  //ִ�а�������   �л�������
			{
				if(RH_H>RH_L+1)RH_H--;
				SectorErase(0x2000);	 //��������ֵ  ���浽��Ƭ����EEPROM
				byte_write(0x2000,RH_H);
			}else if(state==2)
			{
				if(RH_L>0)RH_L--;
				SectorErase(0x2200);	 //��������ֵ	���浽��Ƭ����EEPROM
				byte_write(0x2200,RH_L);
			}else if(state==3)
			{
				if(ki>0)ki-=1;
				SectorErase(0x2400);	 //��������ֵ	���浽��Ƭ����EEPROM
				byte_write(0x2400,ki);
			}
			else if(state==4)
			{
				if(motorKP > 1)motorKP--;	  //��������ֵ
				SectorErase(0x2600);	 //��������ֵ	 ���浽��Ƭ����EEPROM
				byte_write(0x2600,motorKP);
			}else if(state==5)
			{
				if(aimLevel > 1)aimLevel--;	  //��������ֵ
				SectorErase(0x2800);	 //��������ֵ	 ���浽��Ƭ����EEPROM
				byte_write(0x2800,aimLevel);
			}
			else if(state==0)
			{
				if(zt==0)
				{
					motorControl(2);	   //�ֶ�ģʽ�л�ϵͳ�Ĺ���״̬
					led_1 = !led_1;
					
				}else if(zt > 1)
				{
						motorKP = byte_read(0x2600);

						ki = byte_read(0x2400);
				}
				
			}
		}
	}		
}
/********************************************************************
* ���� : proc()
* ���� : ϵͳ������򲿷� 
* ���� : ��
* ��� : ��
***********************************************************************/
void proc()
{

		/////////�����߼�
		if(zt==1)	  //zt==1  Ϊ�Զ�ģʽ    ���ϵͳ���Զ�ģʽ��
		{
			pidFlag = 0;
			//motorControl(0);
			if(RH>=RH_H) 	motorControl(0);   //�رռ̵���
			if(RH>RH_H)	   //�����ǰˮλֵ�ﵽˮλ����ֵ��
			{
			   
				led_1=0;		//��ʾ��Ӧ��ָʾ��
				
			}else		  //���� 
			{
				led_1=1;	   //��ʾ��Ӧ��ָʾ��
			}

			if(RH<=RH_L)	   //�����ǰˮλֵ����ˮλ����ֵ��
			{
				motorControl(1);   //�����̵��� 
				led_2=0;	   //��ʾ��Ӧ��ָʾ��
			}else	    //���� 
			{
				led_2=1;	   //--------------------------------------\��ʾ��Ӧ��ָʾ��
			}

			if(RH>RH_H||RH<=RH_L)  //������������   �����ǰˮλ����ˮλ������  ��
			{
				beep1=1;		   //��ʼ����	
				}
			else		 //����
			{
				overLimit = 0;
				beep1=0;	   //ֹͣ����
			}
		}else if(zt == 2)//pid����ģʽ
		{
			pidFlag = 1;
			led_1=led_2=1;
			beep1=0;
		}
		else if(zt == 0)
		{
			pidFlag = 0;
			//motorControl(0);
			beep1=0;		 //�ֶ�ģʽ�ر�ָʾ�Ƽ�������
			led_1=led_2=1;
		}
		else if(zt == 3)
		{
			if(pidFlag == 0)
			{
				pidFlag = 1;
			}
		}
}



float getWaterLevel()//��ȡ��ǰ�˲�֮���ˮλ��ͬʱ����ȫ�ֱ���RH
{
	if(cs<20)	 //�ж��Ƿ��ȡ20�ε������ˣ�  �������ʼ���㵱ǰˮλ
		{
			cs++;			 //ÿ�μ��α�־λÿ��ȡһ������
			Ad_datN+=A_D();	 //������ȡϵͳ����20�Σ�ˮλADֵ��������
		}else
		{
			 cs=0;			 //�Լ��α�����0
		   Ad_dat=Ad_dat*0.9+Ad_datN*0.1;	 //�����ݵģ�ϵͳ�˲�����	   ����¼�����㡢�˲����ϵͳ���ݣ�ˮλAD���ݣ����ú������С�����ݱ���
		   Ad_datN=0;		//���
		  // RH = Ad_dat/51-1.5;
			if(Ad_dat>80) 
		    {
			   RH=(Ad_dat-80.0)/48.0;  //����ˮλ
			} 
		   
		}
		return RH;
}


float MOD(float a)
{
	float b;
	if(a >= 0)
		return a;
	else
	{
		b = 0 - a ;
		return b;
	}
		
}

void pidControl()
{
	static float e[2] = {0,0};//���
	static float u;//ˮ����
	static uchar s = 1;//�������ʱ��
	static int i = -1;
	static float ji = 0;//����e
	static float kd = 0.3;//΢��ϵ��
	static float we = 0;//΢�ֲ�
	i = (i+1)%2;
	e[0] = e[1];	//�ϴε����
	e[1] = aimLevel - RH;
	ji += e[1];
	we = e[0] - e[1];
	
	if(zt == 2)
	{
		motorKP = 1 +  MOD(e[1])/2;
		if(we > 0)//�����С��
		{
			ki = motorKP * ki *1.2;
		}
		else
		{
			ki = motorKP * ki / 1.2;
		}
	}else if(zt == 3)
	{
		//motorKP = byte_read(0x2600);
		
		//ki = byte_read(0x2400);
	}
 
	
	u = e[i] * motorKP + ki/10 * ji + kd * we;	
  if(u > 1)
	{
		openMotorTime((int)(u));
	}
	else
	{
		//motorControl(0);
		motorRunTime = 0;
	}
}


void main()
{	

	alarm_1 = 1;//�̵���״̬��ʼ��
	TMOD=0x01;    //��ʱ�����ó�ʼ��
	TH0=0x3c;	    //16λ��ʱ   ��ʱ50ms
	TL0=0xb0;
	ET0=1;
	TR0=1;	   //�������ж�    ���򿪶�ʱ��0
	EA=1;
	LCD1602_cls();	   //LCD1602  ��ʼ��
	RH_H=byte_read(0x2000);	   //��ȡEEPROM�е�ˮλ�����޵�ֵ
	RH_L=byte_read(0x2200);

	
	////////////////coffee//////////

	pidFlag = 0;
	pidTime = 2;//5sһ�εĲ����ж�Ƶ��
	/////////////
	motorKP = byte_read(0x2600);
	if((motorKP>10)||(motorKP<1))
	{
		motorKP=0;
	}
	ki = byte_read(0x2400);
	if((ki>9.9)||(ki<0.1))
	{
		ki=0.5;
	}
	waterCount = byte_read(0x2400);
	if((waterCount>99)||(waterCount<1))
	{
		waterCount=0;
	}
	motorKP = byte_read(0x2600);
	if((motorKP>9)||(motorKP<1))
	{
		motorKP=5;
	}
	aimLevel = byte_read(0x2800);
	if((aimLevel>99)||(aimLevel<1))
	{
		aimLevel=16;
	}
	overLimit = 0;
	
	//////////////////////////////
	if((RH_H>99)||(RH_L>99)||(RH_L>=RH_H))   {RH_H=14;  RH_L=7;} //�������ˮλ���������õķ�Χ�������¸�ֵ
	   
	while(1)
	{
		/******************************************************
					�� ˮλ�������˲��ȶ�������
		******************************************************/
		
		getWaterLevel();
		show();	  //�����ӳ���
		key();	  //���ð���ɨ���Ӻ���
		proc();	  //���ñ����Ӻ���
	}
}



void UART_1() interrupt 1  //��ʱ��0 �ж�
{
	TH0=0x3c;			 //���¸�ֵ
	TL0=0xb0;
	ms++;		    //50ms����
	if(ms%5==0)		//250ms  ��ʱ
	{
	    s1=!s1;	    //�ı���˸��־Ϊ��ֵ
	}
	if(ms%10==0)	   //500ms��ʱ
	{
		if(beep1==1) //��������������   
		{
			beep=!beep;
		}else
		{
			beep=1;
		}	
		
		if(overLimit==1) //��ˮ��������  ,˫���� 
		{
			led_1=!led_1;
			led_2=!led_2;
			
		}else
		{
			led_1 = led_2 = 1;
		}
	}
	if(ms>19)
	{
		if(pidFlag == 1 &&( zt == 2 || zt == 3))//������PID,ÿ��pidTime������һ��pid�����ƺ���
		{
			if(	pidCount == 0)
			{
				pidControl();
				pidCount = 0;
			}
			pidCount = (pidCount+1)% pidTime;
			
		}
		
		if(motorRunTime > 0 && motorControl(3) == 0)//�������N�����Σ�����ΪֻҪ����openMotorTime()��������
		{
			motorControl(1);
			motorRunTime--;
		}else if(motorRunTime > 0 && motorControl(3) == 1)
		{
			motorRunTime--;
		}
		else if(motorRunTime == 0 && motorControl(3) == 1)
		{
			motorControl(0);
			motorRunTime--;
		}
		
	/*	if(motorControl(3))//��ȡ���״̬���������ʱ
		{
			//motorTimeCount++;
			//motorTimeCount = motorTimeCount % (int)motorKP;
			//if(motorTimeCount == 0)
			//	waterCount++;
			//motorTimeCount = motorTimeCount % 256; //����Ҫ��0����ΪmotorTimeCount���޶��ľ���256��uchar����
		}*/
		ms=0;
	}	
}




