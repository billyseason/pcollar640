/*
  a_tx[4]:�����͵�4���ֽ����ݴ洢��������FIFO_WRITE(4)�еĲ�����Ӧ

��Ŀ��640ѵ����ң�������
������ HT-V3,оƬ��HT66F0185 
���ܣ�LCD��ʾ������������433M��������,���

�޸���ʷ��
--2018/06/13
  1������AD��⹦�ܡ�
  2�����ӳ���⹦�ܣ�����ϵ�����ʾ�������ܡ��ù����д���һ�����ơ�
  3����ʼ���������Ӳ��ֵ���ر�����  
*/

#include "HT66F0185.h"
#include "head.h"
#include "cmt2300a_defs.h"

// eeprom
typedef struct
{
	unsigned char bit0:1;
	unsigned char bit1:1;
	unsigned char bit2:1;
	unsigned char bit3:1;
	unsigned char bit4:1;
	unsigned char bit5:1;
	unsigned char bit6:1;
	unsigned char bit7:1;
}iar_bits;
DEFINE_SFR(iar_bits,iar1,0x02);
#define iar1_3	iar1.bit3
#define iar1_2	iar1.bit2
#define iar1_1	iar1.bit1
#define iar1_0	iar1.bit0

// common
volatile unsigned char a_lcd_count,lcd_data[2],a_100ms,a_count,a_10ms,a_500ms,a_1min;
unsigned char a_k1_high,a_k1_low,a_k2_high,a_k2_low,a_k3_high,a_k3_low,a_k4_high,a_k4_low,a_k5_high,a_k5_low;
unsigned char a_up_high,a_up_low,a_down_high,a_down_low;
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

// charge
volatile unsigned char a_charge_status;

#define c_charge_idle     0
#define c_charge_ongoing  1
#define c_charge_full     2

// channel
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
            if(a_500ms>=24)
            {
                a_500ms=0;
                if(f_dc_connect)
                {
                    if(a_charge_status==1)
                    {
                        a_voltage_level++;
                        if(a_voltage_level>3)
                        {
                            a_voltage_level=0;
                        }
                    }
                    else
                    {
                        a_voltage_level=3;
                    }
				}
				else
				{
    				f_voltage_500ms=1;
                }
				a_1min++;
				if(a_1min>=120)		//120
				{
					a_1min=0;
					f_1min=1;
					f_halt=1;	
				}
			}
		}
		
		a_lcd_count++;
		if(a_lcd_count>=8)	a_lcd_count=0;
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

//	tmp=((unsigned char)~0x07)&SPI_READ(CMT2300A_CUS_CMT10);		//RFPDK 1.46�Ժ󣬿ɺ���
//	SPI_WRITE(CMT2300A_CUS_CMT10, tmp|0x02);
//	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep	

	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO1_SEL_DOUT|CMT2300A_GPIO2_SEL_DOUT|CMT2300A_GPIO3_SEL_INT2);	//INT1 > GPIO1   ,INT2 > GPIO3
	tmp=CMT2300A_INT_SEL_TX_FIFO_NMTY;
	tmp|=CMT2300A_INT_POLAR_SEL_1;
	SPI_WRITE(CMT2300A_CUS_INT1_CTL, tmp);
	
//	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO3_SEL_INT2);	//INT2 > GPIO3
//	tmp=SPI_READ(CMT2300A_CUS_INT2_CTL);
	tmp=CMT2300A_INT_SEL_TX_DONE;						//0��Ч
	SPI_WRITE(CMT2300A_CUS_INT2_CTL, tmp);				//CMT2300A_INT1_SEL_TX_DONE
	SPI_WRITE(CMT2300A_CUS_INT_EN,CMT2300A_MASK_TX_DONE_EN);	//CMT2300A_MASK_TX_DONE_EN
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
	GCC_DELAY(60000);
	//����CMT�Ƿ����
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
        if(!f_tx)             //����ʹ�ñ�־λ���п���
		{
            a_tx_count++;
            if(a_tx_count>=a_set_count)
            {
                f_txen=0;
                a_tx_count=0;
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


// eeprom
unsigned char read_byte(unsigned char addr)
{  
	_eea=addr;
 	_mp1=0x40; 
 	_bp=0x01;
 	iar1_1=1;
	iar1_0=1;
  	while(iar1_0);
  	_mp1=0;
  	_iar1=0;
  	return _eed;
}

void  write_byte(unsigned char addr,unsigned char data_ee)
{    
  	_eea=addr;
  	_eed=data_ee; 
  	_mp1=0x40;
  	_bp=0x01;
  	_emi=0;
	iar1_3=1;
	iar1_2=1;
  	_emi=1;
  	while(iar1_2); 
  	_iar1=0;
  	_mp1=0;
}


void write_eeprom(unsigned char data_ee,unsigned char addr,unsigned char num)
{
	unsigned char i;
	for(i=0;i<num;i++)
	{
		write_byte(data_ee+i,addr+i);
		GCC_DELAY(10000);	
	}	
}



void SetNumber(unsigned char num)
{
    unsigned char decade=0;

    if(num>9)
    {decade=1;}
    //decade=num/10;
    lcd_data[0]=c_num[num%10];
    lcd_data[0]&=0x7F;
    if(decade)
    {
        lcd_data[0]|=0x80;
    }

    write_byte(0x00, num);
}



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


void ChargeDetect()
{
    if((!CHRGIN)&&(!f_dc_connect))     //�����
    {
        BL=1;
        a_charge_status=c_charge_ongoing;
        f_dc_connect=1;
    }

    if((CHRGIN)&&(f_dc_connect))
    {
        CHRGOUTC=0;
        CHRGOUTPU=1;
        CHRGOUT=1;
        if(!CHRGIN)  //����δ����
        {
            a_charge_status=c_charge_full;
            f_dc_connect=1;
        }
        else        //�����Ѱ��£�δ���
        {
            a_charge_status=c_charge_idle;
            f_dc_connect=0;
            f_dc_plugout=1;
        }

        CHRGOUTC=1;
        CHRGOUTPU=0;
    }

    if(f_dc_connect)
    {
        lcd_data[0]|=0xFF;
        switch(a_charge_status)
        {
            case c_charge_ongoing:
                lcd_data[0]&=0x59;  //�����lcd��ʾC
                break;
            case c_charge_full:     
                lcd_data[0]&=0x71;  //�������lcd��ʾF
                break;
            default:                
                lcd_data[0]&=0x79;  //���״̬�쳣����ʾE
                break;
        }
        
        lcd_data[1]&=0xF0;  //�ر�lcd��ͷ��ʾ
        SetVoltageLevel(a_voltage_level);
    }

    if(f_dc_plugout)
    {
        f_dc_plugout=0;
        SetNumber(a_count);
    }

}


void Switch()
{
    //��ֹ�л�channelʱ����sleep
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


void KEY()
{
    Switch();

    K1C=1;
    K1UP=1;
    if(!K1)
    {
        a_k1_high=0;
        if(!f_k1_buf)
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
        if(f_k1_buf)
        {
            a_k1_high++;
            if(a_k1_high>=5)
            {
                a_k1_high=0;
                f_k1_buf=0;
            }   
        }
    }

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
		if(!f_k3_buf)
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
		if(f_k3_buf)
		{
			a_k3_high++;
			if(a_k3_high>=5)
			{
				a_k3_high=0;
				f_k3_buf=0;
            }
		}
	}
    
    if(K1&&K2&&K3)
    {
        K3C=0;
        K3=0;
        if(!K1)
        {
            K3=1;
            if(K1)
            {
                a_up_high=0;
                if(!f_up_buf)
                {
                    a_up_low++;
                    if(a_up_low>=5)
                    {
                        a_up_low=0;
                        f_up=1;
                        f_up_buf=1;
                    }
                }
            }
        }
        else
        {
            a_up_low=0;
            if(f_up_buf)
            {
                a_up_high++;
                if(a_up_high>=5)
                {
                    a_up_high=0;
                    f_up_buf=0;
                }
            }
        }

        K3=0;
        if(!K2)
        {
            K3=1;
            if(K2)
            {
                a_down_high=0;
                if(!f_down_buf)
                {
                    a_down_low++;
                    if(a_down_low>=5)
                    {
                        a_down_low=0;
                        f_down=1;
                        f_down_buf=1;
                    }
                }
            }
        }
        else
        {
            a_down_low=0;
            if(f_down_buf)
            {
                a_down_high++;
                if(a_down_high>=5)
                {
                    a_down_high=0;
                    f_down_buf=0;
                }
            }
        }
    	K3C=1;
    	K3UP=1;
        K3WU=1;
    }

}


// for key press test
void KeyCountForTest()
{
    //��������LCD��ʾ�����ڵ���
    if(f_down)
    {
        if(a_count>0)
        {
            a_count--;
        }
    }

    if(f_up)
    {
        if(a_count<16)
        {
            a_count++;
        }
    }

    SetNumber(a_count);
}

void KeyPressTest()
{
    unsigned char number;

    if(f_k1)
    {
        number=1;
    }

    if(f_k2)
    {
        number=2;
    }

    if(f_k3)
    {
        number=3;
    }

    if(f_up)
    {
        number=4;
    }

    if(f_down)
    {
        number=5;
    }

    SetNumber(number);
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
	
	BL=1;
	CMT_init();             //2119B��ʼ�����ã������ϵ�ʱ��ʼ��һ�Σ����������ٳ�ʼ��
	a_tx[0]=0x55;
	a_tx[1]=0xaa;
	a_tx[2]=0xaa;
	a_tx[3]=0x55;
    a_count=read_byte(0x00);
    if(a_count>16) a_count=0;
	lcd_data[0]=c_num[a_count];    
	lcd_data[1]=0;
	a_set_count=3;
    a_tx_count=0;
    a_last_channel=1;
    a_charge_status=c_charge_idle;
    f_dc_connect=0;
    
	_idlen=0;
	_lvden=0;

    // k1 k2 k3 wakeup
    K1WU=1;
	K2WU=1;
    K3WU=1;

    //channel switch
    SWICHC=1;
    SWICHUP=1;
    SWICHWU=1;

    //charge detect in/out
    CHRGINC=1;
    CHRGINWU=1;
    CHRGOUTC=0;
    CHRGOUTPU=1;
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
            ChargeDetect();
            if(f_dc_connect)
            {
                a_1min=0;
            }
            else
            {
                KEY();
                Voltage();
                if(f_k1||f_k2||f_k3||f_up||f_down)
                {
                    KeyCountForTest();
                    //KeyPressTest();
                    f_k1=0;
                    f_k2=0;
                    f_k3=0;
                    f_up=0;
                    f_down=0;
                    f_txen=1;
                    a_1min=0;
                    if(f_halt_buf)
                    {
                        f_halt_buf=0;
                        a_set_count=c_count8;
                    }
                    else 
                    {
                        a_set_count=c_count2;
                    }
                }
            }

		}
	}
}



