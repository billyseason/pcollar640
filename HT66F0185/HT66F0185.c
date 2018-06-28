/*
  a_tx[4]:待发送的4个字节数据存储变量，与FIFO_WRITE(4)中的参数对应

项目：640训狗器遥控软件。
编译器 HT-V3,芯片：HT66F0185 
功能：LCD显示，六个按键，433M发送数据,充电

修改历史：
--2018/06/13
  1、增加AD检测功能。
  2、增加充电检测功能，以配合电量显示锁定功能。该功能有待进一步完善。
  3、初始化上述增加部分的相关变量。  
*/

#include "HT66F0185.h"
#include "head.h"
#include "cmt2300a_defs.h"

volatile unsigned char a_lcd_count,lcd_data[2],a_100ms,a_count,a_10count,a_10ms,a_500ms,a_1min;
unsigned char a_k1_high,a_k1_low,a_k2_high,a_k2_low,a_k3_high,a_k3_low,a_k4_high,a_k4_low,a_k5_high,a_k5_low;
volatile flag_byte f_flag;
volatile unsigned char a_data,a_tx[4],a_tx_count,a_set_count;

#define c_count2	2
#define c_count8	8


volatile unsigned char ad_data,a_voltage_count,a_voltage_level;
volatile unsigned int  ad_voltage_buf;

//Vref=2.08V,AD=Hight 8bit
#define c_voltage_3V51 	215		//3.51		
#define c_voltage_3V6	221		//3.6		
#define c_voltage_3V9   239     //3.9


volatile unsigned char a_last_channel;

//1250us
void __attribute((interrupt(0x10)))	TM1_int()
{
	if(_t1af)
	{
		_t1af=0;
		a_10ms++;
		if(a_10ms>=8)
		{
			a_10ms=0;
			f_10ms=1;	
			a_500ms++;
			if(a_500ms>=50)
			{
				a_500ms=0;
				f_voltage_500ms=1;	
				a_1min++;
				if(a_1min>=12)		//120
				{
					a_1min=0;
					f_1min=1;
					f_halt=1;	
				}
			}
		}
		
		a_lcd_count++;
		if(a_lcd_count>=9)	a_lcd_count=0;
		if(a_lcd_count<=7)
		{
			SEG3C=0;
			COM0=0;
			COM1=0;
			COM2=0;
			COM3=0;
			SEG0=0;
			SEG1=0;
			SEG2=0;
			SEG3=0;
			_slcdc0|=0b00001111;
			_slcdc1|=0b01001000;
			_slcdc3|=0b01100000;
			_slcdc4|=0b00000001;
			switch(a_lcd_count)									//Output com
			{
				case 0: 
					_frame=0;
					if(lcd_data[0]&0x10)	{_seg20en=0;SEG1=0;}
					if(lcd_data[0]&0x01)	{_seg22en=0;SEG2=0;}
					if(lcd_data[1]&0x10)	{_com3en=0;SEG3=0;}
					if(lcd_data[1]&0x01)	{_seg19en=0;SEG0=0;}
					_com0en=0; COM0=1; 
					break;
				case 1: 	
					_frame=1;			
					if(lcd_data[0]&0x10)	{_seg20en=0;SEG1=1;}
					if(lcd_data[0]&0x01)	{_seg22en=0;SEG2=1;}
					if(lcd_data[1]&0x10)	{_com3en=0;SEG3=1;}
					if(lcd_data[1]&0x01)	{_seg19en=0;SEG0=1;}
					_com0en=0; COM0=0; 
					break;				
				case 2: 				
					_frame=0;
					if(lcd_data[0]&0x20)	{_seg20en=0;SEG1=0;}
					if(lcd_data[0]&0x02)	{_seg22en=0;SEG2=0;}
					if(lcd_data[1]&0x20)	{_com3en=0;SEG3=0;}
					if(lcd_data[1]&0x02)	{_seg19en=0;SEG0=0;}
					_com1en=0; COM1=1; 
					break;
				case 3: 				
					_frame=1;
					if(lcd_data[0]&0x20)	{_seg20en=0;SEG1=1;}
					if(lcd_data[0]&0x02)	{_seg22en=0;SEG2=1;}
					if(lcd_data[1]&0x20)	{_com3en=0;SEG3=1;}
					if(lcd_data[1]&0x02)	{_seg19en=0;SEG0=1;}
					_com1en=0; COM1=0; 
					break;
				case 4: 				
					_frame=0;
					if(lcd_data[0]&0x40)	{_seg20en=0;SEG1=0;}
					if(lcd_data[0]&0x04)	{_seg22en=0;SEG2=0;}
					if(lcd_data[1]&0x40)	{_com3en=0;SEG3=0;}
					if(lcd_data[1]&0x04)	{_seg19en=0;SEG0=0;}
					_com2en=0; COM2=1; 
					break;
				case 5: 				
					_frame=1;
					if(lcd_data[0]&0x40)	{_seg20en=0;SEG1=1;}
					if(lcd_data[0]&0x04)	{_seg22en=0;SEG2=1;}
					if(lcd_data[1]&0x40)	{_com3en=0;SEG3=1;}
					if(lcd_data[1]&0x04)	{_seg19en=0;SEG0=1;}
					_com2en=0; COM2=0; 
					break;
				case 6: 				
					_frame=0;
					if(lcd_data[0]&0x80)	{_seg20en=0;SEG1=0;}
					if(lcd_data[0]&0x08)	{_seg22en=0;SEG2=0;}
					if(lcd_data[1]&0x80)	{_com3en=0;SEG3=0;}
					if(lcd_data[1]&0x08)	{_seg19en=0;SEG0=0;}
					_com4en=0; COM3=1; 
					break;
				case 7: 				
					_frame=1;
					if(lcd_data[0]&0x80)	{_seg20en=0;SEG1=1;}
					if(lcd_data[0]&0x08)	{_seg22en=0;SEG2=1;}
					if(lcd_data[1]&0x80)	{_com3en=0;SEG3=1;}
					if(lcd_data[1]&0x08)	{_seg19en=0;SEG0=1;}
					_com4en=0; COM3=0; 	
					break;	 
			}
		}
		else
		{
			_com3en=0;
			K1C=1;
			K1UP=1;
			_com4en=1;

			if(!K1)
			{
				a_k1_high=0;
				if(f_k1_buf==0)
				{
					a_k1_low++;
					if(a_k1_low>=5)
					{
						a_k1_low=0;
						f_k1=1;
						f_k1_buf=1;	
					}
				}
			}
			else 
			{	
				a_k1_low=0;
				if(f_k1_buf==1)
				{
					a_k1_high++;
					if(a_k1_high>=5)
					{
						a_k1_high=0;
						f_k1_buf=0;
					}	
				}
			}	
		}



	}
}

void FIFO_WRITE(unsigned char len)
{	
	unsigned char i,j,data;
	CSB=1;
	FCSB=0;
	SCLK=0;
	SDIOC=0;
	for(j=0;j<len;j++)
	{
		FCSB=0;
		data=a_tx[j];
		for(i=0;i<8;i++)
		{
			SCLK=0;
			if(data&0x80)	SDIO=1;
			else 			SDIO=0;
			SCLK=1;
			data<<=1;
		}
		SCLK=0;
		GCC_DELAY(10);
		FCSB=1;
		GCC_DELAY(10);
	}
	SCLK=0;
	FCSB=1;
}

void SPI_WRITE(unsigned char address,unsigned char data)
{
	unsigned char i;
	address&=0x7f;
	FCSB=1;
	CSB=0;
	SDIOC=0;
	SCLK=0;
	for(i=0;i<8;i++)
	{
		SCLK=0;
		if(address&0x80)	SDIO=1;
		else                SDIO=0;
		SCLK=1;
		address<<=1;
	}
	for(i=0;i<8;i++)
	{
		SCLK=0;
		if(data&0x80)		SDIO=1;
		else				SDIO=0;
		data<<=1;
		SCLK=1;
	}
	SCLK=0;
	GCC_DELAY(5);
	SDIO=0;
	CSB=1;
}

unsigned char SPI_READ(unsigned char address)
{
	unsigned char i,data;
	FCSB=1;
	CSB=0;	
	SCLK=0;
	SDIOC=0;
	address|=0x80;
	for(i=0;i<8;i++)
	{
		SCLK=0;
		if(address&0x80)	SDIO=1;
		else 				SDIO=0;
		_nop();
		SCLK=1;
		address<<=1;
	}
	SDIOC=1;
	SCLK=0;
	data=0;
	for(i=0;i<8;i++)
	{
		data<<=1;
		SCLK=1;
		if(SDIO)		data|=1;
		_nop();
		SCLK=0;
	}
	SCLK=0;
	SDIOC=0;
	SDIO=0;
	CSB=1;	
	return data;
}

void CMT_init()
{
	unsigned char tmp,back;
	SPI_WRITE(0x7F,0xFF);		//Soft Reset 
	GCC_DELAY(40000);			//20ms
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
	
	tmp=SPI_READ(CMT2300A_CUS_MODE_STA);				//address 0x61
	tmp|=CMT2300A_MASK_CFG_RETAIN;						//Enable CFG_RETAIN 
	tmp&=(unsigned char)~CMT2300A_MASK_RSTN_IN_EN;		//Disable RSTN_IN 
	SPI_WRITE(CMT2300A_CUS_MODE_STA,tmp);
	
	tmp=SPI_READ(CMT2300A_CUS_EN_CTL);		//adress 0x60
	tmp|=CMT2300A_MASK_LOCKING_EN;			//LOCKING_EN=1;
	SPI_WRITE(CMT2300A_CUS_EN_CTL, tmp);
	
	tmp=SPI_READ(CMT2300A_CUS_SYS2);
    tmp &= (unsigned char)~CMT2300A_MASK_LFOSC_RECAL_EN;
    tmp &= (unsigned char)~CMT2300A_MASK_LFOSC_CAL1_EN;
    tmp &=(unsigned char) ~CMT2300A_MASK_LFOSC_CAL2_EN;
	SPI_WRITE(CMT2300A_CUS_SYS2,tmp);
	
	SPI_WRITE(CMT2300A_CUS_INT_CLR1,0xff);
	SPI_WRITE(CMT2300A_CUS_INT_CLR2,0xff);
	
	for(tmp=0;tmp<0x60;tmp++)
	{
		SPI_WRITE(tmp,c_cmt_init[tmp]);
		_clrwdt();	
	}

//	tmp=((unsigned char)~0x07)&SPI_READ(CMT2300A_CUS_CMT10);		//RFPDK 1.46以后，可忽略
//	SPI_WRITE(CMT2300A_CUS_CMT10, tmp|0x02);
//	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep	

	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO1_SEL_DOUT|CMT2300A_GPIO2_SEL_DOUT|CMT2300A_GPIO3_SEL_INT2);	//INT1 > GPIO1   ,INT2 > GPIO3
	tmp=CMT2300A_INT_SEL_TX_FIFO_NMTY;
	tmp|=CMT2300A_INT_POLAR_SEL_1;
	SPI_WRITE(CMT2300A_CUS_INT1_CTL, tmp);
	
//	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO3_SEL_INT2);	//INT2 > GPIO3
//	tmp=SPI_READ(CMT2300A_CUS_INT2_CTL);
	tmp=CMT2300A_INT_SEL_TX_DONE;						//0有效
	SPI_WRITE(CMT2300A_CUS_INT2_CTL, tmp);				//CMT2300A_INT1_SEL_TX_DONE
	SPI_WRITE(CMT2300A_CUS_INT_EN,CMT2300A_MASK_TX_DONE_EN);	//CMT2300A_MASK_TX_DONE_EN
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
	GCC_DELAY(60000);
	//测试CMT是否存在
	back = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, 0xAA);
    tmp = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, back);
    if(tmp!=0xAA) while(1);
  	
}

void CMT_TX()
{
	unsigned char tmp;
	if(f_txen)
	{
		if(!f_tx)             //发射使用标志位进行控制
		{
		   	a_tx_count++;
		   	if(a_tx_count>=a_set_count)
		   	{
		   		f_txen=0;
		   		a_tx_count=0;
		   		a_count++;                          //发射次数LCD显示，用于调试
				if(a_count>16)	
                {            
                    a_count=0;
                }

                if(a_count>9)
                {
                    a_10count=1;
                }
                lcd_data[0]=c_num[a_count-10*a_10count];
                lcd_data[0]&=0x7F;
                if(a_10count)
                {
                    lcd_data[0]|=0x80;
                    a_10count=0;
                }
		   	}	
			f_tx=1;
			SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
			SPI_WRITE(CMT2300A_CUS_INT_CLR1,CMT2300A_MASK_TX_DONE_CLR);  //clr TX_DONE int flag	
			//enable write fifo
			tmp=SPI_READ(CMT2300A_CUS_FIFO_CTL);
			tmp |= (unsigned char)CMT2300A_MASK_SPI_FIFO_RD_WR_SEL; 
		   	tmp |= (unsigned char)CMT2300A_MASK_FIFO_RX_TX_SEL;
		   	SPI_WRITE(CMT2300A_CUS_FIFO_CTL,tmp);
		   	//clr tx fifo
		   	SPI_WRITE(CMT2300A_CUS_FIFO_CLR,CMT2300A_MASK_FIFO_CLR_TX);
			FIFO_WRITE(4);
			//go_tx
		   	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_TX);
		   	LED=1;
		}	
	}
	if(f_tx)
	{
		LED^=1;
		if(!RF_IO3)
		{
			f_tx=0;	
			LED=0;
			SPI_WRITE(CMT2300A_CUS_INT_CLR1,CMT2300A_MASK_TX_DONE_CLR); //clr TX_DONE int flag
			SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
		}	
	}	
}

void KEY()
{
	K2C=1;
	K2UP=1;
	if(!K2)
	{
		a_k2_high=0;
		if(!f_k2_buf)
		{
			a_k2_low++;
			if(a_k2_low>=5)
			{
				f_k2=1;
				f_k2_buf=1;
			}
		}
	}
	else 
	{
		a_k2_low=0;
		if(f_k2_buf)
		{
			a_k2_high++;
			if(a_k2_high>=5)
			{
				f_k2_buf=0;
			}	
		}
	}	
	
	K3C=1;
	K3UP=1;
	if(!K3)
	{
		a_k3_high=0;
		if(f_k3_buf==0)
		{
			a_k3_low++;
			if(a_k3_low>=5)
			{
				a_k3_low=0;
				f_k3=1;
				f_k3_buf=1;	
			}
		}
	}
	else 
	{	
		a_k3_low=0;
		if(f_k3_buf==1)
		{
			a_k3_high++;
			if(a_k3_high>=5)
			{
				a_k3_high=0;
				f_k3_buf=0;
            }
		}
	}
}

void initail()
{
	_dmbp0=0;
	for(_mp1=0x80;_mp1<0xff;_mp1++)	_iar1=0;
	_iar1=0;
	_dmbp0=1;
	for(_mp1=0x80;_mp1<0xff;_mp1++)	_iar1=0;
	_iar1=0;
	//
	_csel=0;
	_cos=1;
	_acerl=0;
	COM0C=0;
	COM1C=0;
	COM2C=0;
	COM3C=0;
	SEG0C=0;
	SEG1C=0;
	SEG2C=0;
	SEG3C=0;
	
	COM0=0;
	COM1=0;
	COM2=0;
	COM3=0;
	SEG0=0;
	SEG1=0;
	SEG2=0;
	SEG3=0;
	_lcden=1;
	_isel1=1;
	_isel0=1;
	
	RF_IO3C=1;	
	RF_IO3UP=1;
	FCSBC=0;
	CSBC=0;
	SCLKC=0;
	SDIOC=0;
	SCLK=1;
	LEDC=0;
	LED=0;
	BLC=0;
	//
	_adcen=1;
	//high 8bit
	_adrfs=0;
	//external input
	_sains2=0;
	_sains1=0;
	_sains0=0;
	//
	_adpgaen=1;
	_vbgen=1;
	//Vref=1.04*2
	_savrs3=1;
	_savrs2=0;
	_savrs1=1;
	_savrs0=0;
	//fsys/8
	_sacks2=0;
	_sacks1=1;
	_sacks0=1;
	//AD PIN AN7
	_ace7=1;
	//AN7 channle
	_sacs2=1;
	_sacs1=1;
	_sacs0=1;

	
	//1250us
	_tm1al=0x71;
	_tm1ah=0x2;
	//fsys/16
	_t1ck2=0;
	_t1ck1=1;
	_t1ck0=0;
	//TM0
	_t1m1=1;
	_t1m0=1;
	//a cclr
	_t1cclr=1;
	//
	_t1on=1;
	_t1ae=1;
	_mf1e=1;
	_emi=1;
	lcd_data[0]=0;
	lcd_data[1]=0;
	
	BL=1;
	CMT_init();             //2119B初始化设置，可以上电时初始化一次，后续无需再初始化
	a_tx[0]=0x55;
	a_tx[1]=0xaa;
	a_tx[2]=0xaa;
	a_tx[3]=0x55;
	lcd_data[0]=c_num[a_count];
	a_set_count=3;
    a_last_channel=1;
	
	_idlen=0;
	_lvden=0;

    K1WU=1;
	K2WU=1;
    K3WU=1;
	
    SWICHC=1;
    SWICHUP=1;
    SWICHWU=1;
    
}

#if 0

//MCU 3.3V 分压：104 104(R17,R16) 告警电压：3.51V
//       A/D:4096
//bat   1:充电完成未进行电量检测;0:正常状态
//elec_level 0:告警(<3.51) 1:低电量(3.51~3.6)  2:中电量(3.6~3.9) 3:满格(>3.9)
void check_vr()
{
	uchar vr_high8=0;
	//uchar vr_low4=0;
	
	_ade = 0;
	_sadc1 = 0x03;    //设置AD时钟
	_adcen = 1;       //ad使能
	_acerl = 0x07;    //AN7
	_sadc0 = 0x27;    //Enable, AN7 
    _adrfs = 0;
    
//	_acerl=ADER_1channel;
//	_adcr0=ADCR_1channel;
//	_adcr1=AD_clk_fsys;
//	
//	//_delay(1000);    //added on 2017/07/06
//	_delay(1000);    //added on 2017/07/06
	
	_start=0;
	_start=1;
	_start=0;
	
	//while(_eocb);
	while(_adbz);   //等待转换结束

	vr_high8 = _sadoh;
	//vr_low4 = _sadol>>4;
		
	//if(VrData>2662)  //3.9V以上
	//if ((vr_high8>0xA6)||((vr_high8==0xA6)&&(vr_low4>0x06)))
	//if ((vr_high8>=0xA6))  //精简算法
	if ((vr_high8>=0x97))  //精简算法
	{
		if(f_bat)     //锁定保护
		{
			elec_level = 3;
			//bat = 0;
		}
	}
	//else if (VrData>2587)  //3.79V~3.9V
	//else if ((vr_high8>0xA1)||((vr_high8==0xA1)&&(vr_low4>0x0B)))
	//else if ((vr_high8>0xA1))  //精简算法
	else if ((vr_high8>=0x8C))  //精简算法
	{
		if ((f_bat) || (elec_level>=2))
		{
			elec_level = 2;
			//bat = 0;
		}
	}
	//else if (VrData>2450)  //3.6V~3.79V
	//else if ((vr_high8>0x99)||((vr_high8==0x99)&&(vr_low4>0x02)))
	//else if ((vr_high8>=0x99))  //精简算法
	else if ((vr_high8>0x88))  //精简算法
	{	
		if ((f_bat) || (elec_level>=1))
		{
			elec_level = 1;
			//bat = 0;
		}
	}
	else
	{
		elec_level = 0;
	}
	
	f_bat = 0;
	
	lcd_data[1]=(lcd_data[1]|0xF0)&c_eleclevel[elec_level];     //LCD显示电量级别 for debug
	
	_adcen = 0;   //ad除能 
	//_adonb = 1; //added on 2017/07/19			
}

//充电检测,仅进行检测，不进行状态指示
void charge_detect()
{
	   //第一次充电点亮背光灯
	   if ((!chargein)&&(!DCinflag))
	   {
	   	  GCC_DELAY(10);               
	   	  if ((!chargein)&&(!DCinflag))
	   	  {
//	   	  	Timer1min_high = 0;
//	   	  	Timer1min_low = 0;
//	   	  	
//	   	  	led_count_high = 0;
//	   	  	led_count_low = 0;

            BL = 1;         //点亮背光

            DCinflag = 1;	   	    //set DCinflag
            f_bat = 1;              //set f_bat
            //chargeflag = 1;       //added on 2016/07/15

            //_adcen = 1;   //ad使能
	   	  }
	   }

       //点亮LED，充电时每次都要进来？？？
	   if ((!chargein) && (chargeout))
	   {
	   	  GCC_DELAY(10); 
	   	  //if ((!chargest) && (!DCin))     //added on 2016/06/30
	   	  if ((!chargein) && (chargeout))
	   	  {
	   	  
	   	    //bat = 1;     //noted on 2016/03/29
	   	    chargeflag = 1;
	   	    
	   	    //是否需要在这里点亮？与DCin检测有点近
			//ledc = 0;        //noted on 2016/02/02
			//_delay(20);
  					
			LED = 1; 				
	    	//ledc = 0;
	     	//led = 1;
	   	  }	     	
	   }
	   
	   if ((chargein) && (chargeout))   
	   {
	   	 GCC_DELAY(10); 
	   	 if ((chargein) && (chargeout))    
	   	 { 
	   	   chargeoutc = 1;
	   	   chargeoutpu = 0;
	   	   
	   	   GCC_DELAY(100); 
	   	   
	   	   if (!chargein)    //已充满
	   	   {
	       		//bat = 1;         //noted on 2016/03/29
	       		chargeflag = 2;
	       
	   	   		DCinflag = 1;	   	  //set DCinflag
	   	   
  		   		//if (led_count > 500)   //1s刷新一次LED,内置，不应该在这里处理
  		   		if (led_count_high >= 2)
  		   		{
  			  		//ledc = 0;         //noted on 2016/02/02
  			  		//_delay(20);
  					
  			  		//ledstatus = ~ledstatus;  			  
  			  		//led = ledstatus;
  			  		LED = ~LED;
  			  
  			  		//led_count = 0;
  			  		led_count_high = 0;
  			  		led_count_low = 0;
  		   		}
	   	   }
	   	   else if (chargein)     //充电插已拔下
	   	   {
	   	   	   GCC_DELAY(10); 
	   	   	   
	   	   	   if (chargein)
	   	   	   {
       	   			chargeflag = 0;
       	   			LED = 0;    
       	   
       	   			DCinflag = 0;  //clr DCinflag
           
           			if(f_bat)  //如果是充电完成，且拔下充电插，则计数器清零，并进行电量检测
           			{
           				BL = 1;          
       	   				
//       	   				Timer1min_high = 0;
//       	   				Timer1min_low = 0;
       	   				
       	   				check_vr();  //added on 2016/05/13
           			}
           			
           			_adcen = 0;    //clr ad
	   	   	   }
	   	   }
	   	   
	   	   chargeoutc = 0;
	   	   chargeout = 1;
	   	 }	       
	   }
}
#endif

unsigned int AD_channel()
{
	unsigned char i;
	unsigned int  sum,ad_max,ad_min;
//	_adcen=1;
	sum=0;
	ad_max=0;
	ad_min=0xffff;
	for(i=0;i<18;i++)
	{
		_start=0;
		_start=1;
		_start=0;
		while(_adbz);	
		sum+=_sadoh;
		if(_sadoh>ad_max) 		ad_max=_sadoh;
		else if(_sadoh<ad_min) 	ad_min=_sadoh;
	}
	sum-=ad_max;
	sum-=ad_min;
	sum>>=4;
//	_adoff=1;
	return sum;
}	

void SetVoltageLevel(unsigned char level)
{
	switch(level)
	{
		case 0:
			lcd_data[1]&=0x1f;
			if(f_voltage_500ms)
			{
				f_voltage_500ms=0;
				lcd_data[1]^=0x10;
			}
			break;
		case 1:
			lcd_data[1]&=0x0f;
			lcd_data[1]|=0x50;
			break;
		case 2:
			lcd_data[1]&=0x0f;
			lcd_data[1]|=0xd0;
			break;
		case 3:
			lcd_data[1]|=0xf0;
			break;				
	}	
}

void Voltage()
{
	ad_data=AD_channel();
	if(a_voltage_count<16)
	{
		a_voltage_count++;
		ad_voltage_buf+=ad_data;
	}
	else
	{
		a_voltage_count=0;
		ad_voltage_buf>>=4;
		if(ad_voltage_buf>=c_voltage_3V9)
		{
			if(!f_voltage_buf0)	a_voltage_level=3;
		}
		else if(ad_voltage_buf>=c_voltage_3V6)
		{
			if(!f_voltage_buf1)
			{
				a_voltage_level=2;
				f_voltage_buf0=1;
			}
		}
		else if(ad_voltage_buf>=c_voltage_3V51)	
		{
			if(!f_voltage_buf2)
			{
				a_voltage_level=1;	
				f_voltage_buf0=1;
				f_voltage_buf1=1;
			}
		}
		else 
		{
			if(!f_voltage_buf3)
			{
				a_voltage_level=0;			
				f_voltage_buf0=1;
				f_voltage_buf1=1;
				f_voltage_buf2=1;
				f_voltage_buf3=1;
			}
		}
		ad_voltage_buf=0;		
	}

	SetVoltageLevel(a_voltage_level);
}	

void Switch()
{
    //防止切换channel时进入sleep
    if(a_last_channel != SWICH)
    {
        a_1min=0;
        a_last_channel = SWICH;
    }

    if(!SWICH)
    {
        lcd_data[1]&=0xF0;
        lcd_data[1]|=0x04;
    }
    else
    {
        lcd_data[1]&=0xF0;
        lcd_data[1]|=0x08;
    }
}



void main()
{
	initail();
	while(1)
	{
		if(f_halt)
		{
			f_halt=0;
			_lcden=0;
			_adcen=0;
			_adpgaen=0;
			_vbgen=0;
			LED=0;
			BL=0;
			f_1min=0;	
			
			SEG3C=0;
			COM0=0;
			COM1=0;
			COM2=0;
			COM3=0;
			SEG0=0;
			SEG1=0;
			SEG2=0;
			SEG3=0;
			_slcdc0=0;
			_slcdc1=0;
			_slcdc3=0;
			_slcdc4=0;
			_wdtc=0xa8;
			
			_halt();
			
			f_halt_buf=1;
			_wdtc=0x53;	
			_lcden=1;		
		}

		CMT_TX();
		_clrwdt();
		if(f_10ms)
		{
		    BL=1;
			f_10ms=0;
            Switch();
			KEY();	
			Voltage();
			if(f_k2 || f_k3 || f_k1)
			{
			    f_k1=0;
				f_k2=0;
                f_k3=0;
				f_txen=1;
				a_1min=0;
				if(f_halt_buf)	
				{
					f_halt_buf=0;
					a_set_count=c_count8;
				}	
				else a_set_count=c_count2;
			}
		}
	}
}
