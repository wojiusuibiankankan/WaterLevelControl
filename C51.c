#include<reg52.h>				 //头文件
#include<intrins.h>
#include"eeprom52.h"			 //STC89C52 EEPROM   程序文件
#define uchar unsigned char		 //宏定义
#define uint unsigned int

#define LCD1602_dat P0	 //LCD1602数据口宏定义


sbit LCD1602_rs=P2^5;		//LCD1602控制数据IO口
sbit LCD1602_rw=P2^6;
sbit LCD1602_e=P2^7;
sbit beep=P1^3;		//蜂鸣器  IO
sbit led_1=P1^4;		//LED指示灯  IO
sbit led_2=P1^6;
sbit key_1=P3^2;		//系统控制按键IO口
sbit key_2=P3^3;
sbit key_3=P3^4;
sbit alarm_1=P2^0;		//控制继电器IO口//


sbit ADC0832_CS=P1^2;	//ADC0832  控制IO口	  使能口
sbit ADC0832_CLK=P1^1;	//时钟IO口
sbit ADC0832_DIO=P1^0;	//数据输入输出IO口 

uint sum, waterCount;			  //10次AD值的综合变量,水量统计
uchar RH_H=12,RH_L=8,state,ms,cs,motorTimeCount,aimLevel;  //当前水位，  水位上限，下限，  设置项变量，50ms变量   ，cs 为计次数变量    ,电机时间计量,用水上限
bit beep1,s1,overLimit,pidFlag;	  //报警标志位， 闪烁标志位 ,用水过量标志，pid运行标志

	static float ki = 0.3;//积分系数

int motorRunTime = -1;//若为-1，不动作，大于0，开启几秒

uchar pidCount,pidTime;

uchar zt;//模式标志
float motorKP;//比例系数
float RH;//当前水位
float Ad_dat,Ad_datN;
 

/*
    ADC0832是一款8位Ad芯片，因为单片机不能直接处理模拟信号（电压），所以单片机测电压的时候基本都是先经过一个模数转换芯片，将模拟量
转化成数字量，然后处理，ADC0832测量的电压范围是0-5V，它能够将0—5V的电压转化成对应比例关系的0-255（8位是0-255）的数据，单片机直
接读取ADC0832的数据获取AD值数据，然后因为0-5V对应0-255数据，所以1V电对应的AD值就是51，就会有如下公式

  电压=AD值/51；

如果想把电压数据精确到小数点后一位就是   电压=AD值/5.1；
					  小数点后两位就是   电压=AD值/0.51；

不要问我为什么，纯数学，小学生都会算。
 
*/
/********************************************************************
* 名称 : delay()
* 功能 : 小延时。													 
* 输入 : 无
* 输出 : 无
***********************************************************************/




void delay(uint T)					  //延时函数
{
	while(T--);
}
void saveWaterCount()
{
	SectorErase(0x2400);	 //保存上限值  保存到单片机中EEPROM
	byte_write(0x2400,waterCount);
}

void openMotorTime(uchar time)//开启电机time秒
{
	motorRunTime = time;
}

uint motorControl(uchar flag)//开=1，关=0，切换=2,什么都不做并且获取当前状态=3
{
	bit a;
	if(flag == 0)//关
	{
		alarm_1 = 1;
		saveWaterCount();
	}
	else if(flag == 1&&alarm_1 == 1)//开
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
unsigned int  A_D()	    //ADC0832   读值程序
{
	unsigned char i;
	unsigned char dat;					 
	ADC0832_CS=1;   //一个转换周期开始
	ADC0832_CLK=0;  //为第一个脉冲作准备
	ADC0832_CS=0;  //CS置0，片选有效
	ADC0832_DIO=1;    //DIO置1，规定的起始信号  
	ADC0832_CLK=1;   //第一个脉冲
	ADC0832_CLK=0;   //第一个脉冲的下降沿，此前DIO必须是高电平
	ADC0832_DIO=1;   //DIO置1， 通道选择信号  
	ADC0832_CLK=1;   //第二个脉冲，第2、3个脉冲下沉之前，DI必须跟别输入两位数据用于选择通道，这里选通道RH0 
	ADC0832_CLK=0;   //第二个脉冲下降沿 
	ADC0832_DIO=0;   //DI置0，选择通道0
	ADC0832_CLK=1;    //第三个脉冲
	ADC0832_CLK=0;    //第三个脉冲下降沿 
	ADC0832_DIO=1;    //第三个脉冲下沉之后，输入端DIO失去作用，应置1
	ADC0832_CLK=1;    //第四个脉冲
	for(i=0;i<8;i++)  //高位在前
	{
		ADC0832_CLK=1;         //第四个脉冲
		ADC0832_CLK=0; 
		dat<<=1;       //将下面储存的低位数据向右移
		dat|=(unsigned char)ADC0832_DIO; 	 //将输出数据DIO通过或运算储存在dat最低位 
	}	  		        
	ADC0832_CS=1;          //片选无效 
	return dat;	 //将读书的数据返回     
}


/*
    1602液晶，是常用的显示器件，一共是16个管脚，其中有八个管脚是数据传输管脚，有三个管脚是数据命令使能端管脚，还有两组电源管脚，
其中一组电源管脚是给整个液晶进行供电的，还有一组电源是单纯的背景光电源，还剩下的最后一个管脚是对比度调节管脚，一般接上一个3K电
阻再接地即可。
 
*/


/********************************************************************
* 名称 : LCD1602_write(uchar order,dat)
* 功能 : 1602写如数据函数
* 输入 : 输入的命令值
* 输出 : 无
***********************************************************************/
void LCD1602_write(uchar order,dat)				  //1602 一个字节  处理
{
    LCD1602_e=0;//使能信号，1读取信息，下降沿执行指令，先行置零
    LCD1602_rs=order;//rs,0输入指令，1输入数据
    LCD1602_dat=dat;//P0口
    LCD1602_rw=0;//read/write,0写1读
    LCD1602_e=1;
    delay(1);
    LCD1602_e=0;																								     
}
/********************************************************************
* 名称 : LCD1602_writebye(uchar *prointer)
* 功能 : 1602写入数据函数  指针式
* 输入 : 输入的命令值
* 输出 : 无
***********************************************************************/
void LCD1602_writebyte(uchar *prointer)				   //1602 字符串    处理
{
    while(*prointer!='\0')
    {
        LCD1602_write(1,*prointer);
        prointer++;
    }
}
/********************************************************************
* 名称 : LCD1602_cls()
* 功能 : 初始化1602液晶 
* 输入 : 无
* 输出 : 无
***********************************************************************/
void LCD1602_cls()									 //1602 初始化
{
	LCD1602_write(0,0x01);     //1602 清屏 指令
	delay(1500);
	LCD1602_write(0,0x38);     // 功能设置 8位、5*7点阵001(指令标识，必须为1)，1(数据接口位数，1=8，=4)， 1(点阵设置)000，
	delay(1500);
	LCD1602_write(0,0x0c);     //原0x0c,设置 光标  00001100 最后三位标识，右4必须定义为1，不显示开关、不显示光标、字符不闪烁
	LCD1602_write(0,0x06);		//原x06,0000 0110 进入模式设置指令，右2 写入数据后左移/右移，右1显示移动/不显示
	LCD1602_write(0,0xd0);    
	delay(1500);
}
/********************************************************************
* 名称 : show()
* 功能 : LCD1602液晶显示程序 
* 输入 : 无
* 输出 : 无
***********************************************************************/
 
/*
数据显示的时候一般的处理：

    首先，无论是数码管显示还是液晶显示，进行显示的时候绝对都是一个一个进行显示的，那么，比如说一个数据123，一百二十三，
进行显示的时候，要先显示1，然后是2，然后是3，那么怎么把数据提取出来？？   
提取百位    123/100=1
提取十位    123/10=12      12%10=2     “%”是取余的意思，像这个，就是12对10取余，换句话说，12除以10，然后取余数，就是2
提取个位    123%10=3       解释同上

取余的用法也有很多种，大家只要知道出现这个的时候，一般都是进行数据提取的就行


然后
如果您是数码管显示数据，将提取的数据放到段码数组里面送给IO即可，
如果是液晶显示，需要将数据转化成字符，因为液晶是字符屏，只能显示字符数据，数据0对应的字符是0x30，数据1对应的字符是0x31，
所以将提取出的数据直接加上0x30送给液晶即可，或者加上'0' 也是一样的 


 
*/


void show()
{
	if(state==0)		//当前水位及工作模式显示,用水过度显示
	{
		
		LCD1602_write(0,0x80);//设置DDRAM显存地址指令，效果上类似于设置光标，1+xxx,xxxx 首位必须1，剩下几位标识地址，左二为0时，第一行，为1时第2行//
		LCD1602_writebyte("WaterLevel:");	//当前水位
		LCD1602_write(0,0x80+11);
		if(RH>=10)LCD1602_write(1,0x30+((int)RH)/10);//dat换rh
		else LCD1602_writebyte(" ");
		LCD1602_write(0,0x80+12);
		LCD1602_write(1,0x30+((int)RH)%10);
		LCD1602_write(0,0x80+13);
		LCD1602_writebyte(".");
		LCD1602_write(1,0x30+((int)(RH*10))%10);
		//LCD1602_writebyte("cm");

		if(0)//overLimit == 1),舍弃了统计功能，不写了，待删
		{
			LCD1602_write(0,0xC0);	LCD1602_writebyte("WARN:Over Using");	  //过度用水
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
				if(RH>RH_H)	   //如果当前水位值达到水位上限值则
				{
					 
					LCD1602_write(0,0xC0);	
					LCD1602_writebyte("Over Toplimit");	
					
				}
				else	if(RH<=RH_L)	   //如果当前水位值低于水位下限值则
				{	
					LCD1602_writebyte("Over LowerLimit");	
				}else	    //否则 
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
			
	}else if(state == 1 || state == 2)	  //水位上下限设置界面，
	{
		LCD1602_write(0,0x80);
		LCD1602_writebyte("Water_H:");   //水位上限，state == 1
		LCD1602_write(0,0x80+8);
		
		if(state==1&&s1==1)		   //通过闪烁标志为  达到闪烁的效果,250ms更改一次
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
		LCD1602_writebyte("Water_L:");  //水位下限
		if(state==2&&s1==1)		 //通过闪烁标志为  达到闪烁的效果
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
		LCD1602_write(0,0x80);//设置DDRAM显存地址指令，效果上类似于设置光标，1+xxx,xxxx 首位必须1，剩下几位标识地址，左二为0时，第一行，为1时第2行//
	/*	LCD1602_writebyte("Count:");	//当前水位
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
		LCD1602_writebyte("PID_KI:");  //水位下限
		if(state==3&&s1==1)		 //通过闪烁标志为  达到闪烁的效果
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
	else if(state == 4 || state == 5)	  //电机速度设置界面，
	{
		LCD1602_write(0,0x80);
		LCD1602_writebyte("Motor_K:");   //比例系数，state == 1
		LCD1602_write(0,0x80+8);
		
		if(state==4&&s1==1)		   //通过闪烁标志为  达到闪烁的效果,250ms更改一次
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
		LCD1602_writebyte("AimLevel:");  //用水上限
		if(state==5&&s1==1)		 //通过闪烁标志为  达到闪烁的效果
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
* 名称 : key()
* 功能 : 按键控制程序     实现系统各个控制功能 
* 输入 : 无
* 输出 : 无
***********************************************************************/
void key()
{
	if(!key_1) //设置按键  设置  功能：切换显示及设置的选项
	{
		delay(888);   //按键去抖
		if(!key_1)	//再次判断按键
		{
			while(!key_1) show();  //按键判断是否释放
			state=(state+1)%6;	  //执行按键功能   切换设置项,并且设置归零
		}
	}

	if(!key_2)   //切换设置项值的大小    及非设置模式下切换系统工作模式按键
	{
		delay(888);   //按键去抖
		if(!key_2)   //再次判断按键
		{
			while(!key_2)show();  //按键判断是否释放
			if(state==1)		  //执行按键功能   切换设置项
			{
				if(RH_H<100)RH_H++;		   //设置上限值
				SectorErase(0x2000);	 //保存上限值	 保存到单片机中EEPROM
				byte_write(0x2000,RH_H);
			}else if(state==2)
			{
				if(RH_L<RH_H-1)RH_L++;	  //设置下限值
				SectorErase(0x2200);	 //保存下限值	 保存到单片机中EEPROM
				byte_write(0x2200,RH_L);
			}else if(state==3)
			{
				if(ki < 20)ki+=1;	  //设置上限值
				SectorErase(0x2400);	 //保存到单片机中EEPROM
				byte_write(0x2400,ki);
				
			}else if(state==4)
			{
				if(motorKP < 9)motorKP++;	  //设置上限值
				SectorErase(0x2600);	 //保存到单片机中EEPROM
				byte_write(0x2600,motorKP);
			}else if(state==5)
			{
				if(aimLevel < 99)aimLevel++;	  //设置上限值
				SectorErase(0x2800);	 //	 保存到单片机中EEPROM
				byte_write(0x2800,aimLevel);
			}
			else if(state == 0)
			{

				zt=(zt+1)%4;				 //切换系统的工作模式   自动   手动
				motorRunTime = 0;
				motorControl(0);
				
				//delay(500);
			}	
		}
	}

	if(!key_3)    //切换设置项值的大小    及手动模式下切换系统工作状态‘开关’
	{
		delay(888);	//按键去抖
		if(!key_3)    //再次判断按键
		{		
			while(!key_3)show();  //按键判断是否释放
			if(state==1)		  //执行按键功能   切换设置项
			{
				if(RH_H>RH_L+1)RH_H--;
				SectorErase(0x2000);	 //保存上限值  保存到单片机中EEPROM
				byte_write(0x2000,RH_H);
			}else if(state==2)
			{
				if(RH_L>0)RH_L--;
				SectorErase(0x2200);	 //保存下限值	保存到单片机中EEPROM
				byte_write(0x2200,RH_L);
			}else if(state==3)
			{
				if(ki>0)ki-=1;
				SectorErase(0x2400);	 //保存下限值	保存到单片机中EEPROM
				byte_write(0x2400,ki);
			}
			else if(state==4)
			{
				if(motorKP > 1)motorKP--;	  //设置下限值
				SectorErase(0x2600);	 //保存下限值	 保存到单片机中EEPROM
				byte_write(0x2600,motorKP);
			}else if(state==5)
			{
				if(aimLevel > 1)aimLevel--;	  //设置下限值
				SectorErase(0x2800);	 //保存下限值	 保存到单片机中EEPROM
				byte_write(0x2800,aimLevel);
			}
			else if(state==0)
			{
				if(zt==0)
				{
					motorControl(2);	   //手动模式切换系统的工作状态
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
* 名称 : proc()
* 功能 : 系统处理程序部分 
* 输入 : 无
* 输出 : 无
***********************************************************************/
void proc()
{

		/////////处理逻辑
		if(zt==1)	  //zt==1  为自动模式    如果系统在自动模式下
		{
			pidFlag = 0;
			//motorControl(0);
			if(RH>=RH_H) 	motorControl(0);   //关闭继电器
			if(RH>RH_H)	   //如果当前水位值达到水位上限值则
			{
			   
				led_1=0;		//显示对应的指示灯
				
			}else		  //否则 
			{
				led_1=1;	   //显示对应的指示灯
			}

			if(RH<=RH_L)	   //如果当前水位值低于水位下限值则
			{
				motorControl(1);   //开启继电器 
				led_2=0;	   //显示对应的指示灯
			}else	    //否则 
			{
				led_2=1;	   //--------------------------------------\显示对应的指示灯
			}

			if(RH>RH_H||RH<=RH_L)  //蜂鸣器处理部分   如果当前水位超出水位上下限  则
			{
				beep1=1;		   //开始报警	
				}
			else		 //否则
			{
				overLimit = 0;
				beep1=0;	   //停止报警
			}
		}else if(zt == 2)//pid控制模式
		{
			pidFlag = 1;
			led_1=led_2=1;
			beep1=0;
		}
		else if(zt == 0)
		{
			pidFlag = 0;
			//motorControl(0);
			beep1=0;		 //手动模式关闭指示灯及蜂鸣器
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



float getWaterLevel()//获取当前滤波之后的水位，同时更新全局变量RH
{
	if(cs<20)	 //判断是否读取20次的数据了？  如果是则开始计算当前水位
		{
			cs++;			 //每次几次标志位每读取一次数据
			Ad_datN+=A_D();	 //连续读取系统数据20次（水位AD值）的数据
		}else
		{
			 cs=0;			 //对几次变量清0
		   Ad_dat=Ad_dat*0.9+Ad_datN*0.1;	 //总数据的，系统滤波运算	   并记录，运算、滤波后的系统数据（水位AD数据），让后清除最小总数据变量
		   Ad_datN=0;		//清除
		  // RH = Ad_dat/51-1.5;
			if(Ad_dat>80) 
		    {
			   RH=(Ad_dat-80.0)/48.0;  //计算水位
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
	static float e[2] = {0,0};//误差
	static float u;//水流量
	static uchar s = 1;//电机开启时间
	static int i = -1;
	static float ji = 0;//积分e
	static float kd = 0.3;//微分系数
	static float we = 0;//微分差
	i = (i+1)%2;
	e[0] = e[1];	//上次的误差
	e[1] = aimLevel - RH;
	ji += e[1];
	we = e[0] - e[1];
	
	if(zt == 2)
	{
		motorKP = 1 +  MOD(e[1])/2;
		if(we > 0)//误差缩小了
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

	alarm_1 = 1;//继电器状态初始化
	TMOD=0x01;    //定时器配置初始化
	TH0=0x3c;	    //16位定时   定时50ms
	TL0=0xb0;
	ET0=1;
	TR0=1;	   //开启总中断    及打开定时器0
	EA=1;
	LCD1602_cls();	   //LCD1602  初始化
	RH_H=byte_read(0x2000);	   //读取EEPROM中的水位上下限的值
	RH_L=byte_read(0x2200);

	
	////////////////coffee//////////

	pidFlag = 0;
	pidTime = 2;//5s一次的采样判断频率
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
	if((RH_H>99)||(RH_L>99)||(RH_L>=RH_H))   {RH_H=14;  RH_L=7;} //如果超出水位上下限设置的范围，则重新赋值
	   
	while(1)
	{
		/******************************************************
					了 水位计算了滤波稳定数据算
		******************************************************/
		
		getWaterLevel();
		show();	  //调用子程序
		key();	  //调用按键扫描子函数
		proc();	  //调用报警子函数
	}
}



void UART_1() interrupt 1  //定时器0 中断
{
	TH0=0x3c;			 //重新赋值
	TL0=0xb0;
	ms++;		    //50ms计数
	if(ms%5==0)		//250ms  计时
	{
	    s1=!s1;	    //改变闪烁标志为的值
	}
	if(ms%10==0)	   //500ms定时
	{
		if(beep1==1) //蜂鸣器报警处理   
		{
			beep=!beep;
		}else
		{
			beep=1;
		}	
		
		if(overLimit==1) //用水过量处理  ,双灯闪 
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
		if(pidFlag == 1 &&( zt == 2 || zt == 3))//若启动PID,每隔pidTime秒启动一次pid主控制函数
		{
			if(	pidCount == 0)
			{
				pidControl();
				pidCount = 0;
			}
			pidCount = (pidCount+1)% pidTime;
			
		}
		
		if(motorRunTime > 0 && motorControl(3) == 0)//开启电机N秒代码段，抽象为只要调用openMotorTime()就能设置
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
		
	/*	if(motorControl(3))//获取电机状态，电机开启时
		{
			//motorTimeCount++;
			//motorTimeCount = motorTimeCount % (int)motorKP;
			//if(motorTimeCount == 0)
			//	waterCount++;
			//motorTimeCount = motorTimeCount % 256; //不需要回0，因为motorTimeCount上限定的就是256，uchar类型
		}*/
		ms=0;
	}	
}




