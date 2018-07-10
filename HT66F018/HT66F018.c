#include "HT66F018.h"
#include "head.h"
#include "cmt2300a_defs.h"


volatile unsigned char a_count,a_10ms,a_100ms,a_1s,a_3min;
//unsigned char a_k1_high,a_k1_low,a_k2_high,a_k2_low,a_k3_high,a_k3_low,a_k4_high,a_k4_low,a_k5_high,a_k5_low;
unsigned char a_onoff_high, a_onoff_low;
volatile flag_byte f_flag;
volatile unsigned char a_data,a_count,a_rx[4],a_read[0x20],a_20s,f_20s;
volatile unsigned int a_bz,a_motor,a_stim_level;
volatile unsigned char a_charge_status;

volatile unsigned char loop_count,time_elec_high;

#define c_charge_idle     0
#define c_charge_ongoing  1
#define c_charge_full     2

//Vref=2.08V,AD=Hight 8bit
#define c_voltage_3V51 	215		//3.51
#define c_voltage_3V6	221		//3.6
#define c_voltage_3V9   239     //3.9

volatile unsigned char ad_data,a_voltage_count,a_voltage_level;
volatile unsigned int  ad_voltage_buf;

volatile unsigned char a_charge_full_wait;

void TestViaLed()
{
	while(1)
	{
		RED_LED=1;
		GREEN_LED=0;
		GCC_DELAY(263690);
		RED_LED=0;
		GREEN_LED=1;
		GCC_DELAY(263690);
		_clrwdt();
	}
}

//125us
void __attribute((interrupt(0x10)))	TM1_int()
{
	if(_t1af)
	{
		_t1af=0;
        if(f_bz)
        {
            GREEN_LED=1;
            RED_LED=0;
            if(a_bz--)
            {
                BZ^=1;
            }
            else
            {
                f_bz=0;
                GREEN_LED=0;
                RED_LED=0;
            }
        }        
		a_10ms++;
		if(a_10ms>=80)
		{
			a_10ms=0;            
			f_10ms=1;
			a_1s++;
			if(a_1s>=200)
			{
				a_1s=0;
				if(a_voltage_level==0)
				{
					f_voltage_500ms=1;
				}

				a_3min++;
				if(a_3min>=20)		//180
				{
					a_3min=0;
					f_halt=1;
				}
			}
		}
	}
}

void delay_ms(unsigned char x)
{
    unsigned char i;
    unsigned char j;
    
    for(i=0;i<x;i++)
    {
       //j=166;   //V1版本
       j=171;     //V1版本修正后
       _clrwdt();
       //j=80;
       //j=72;   //71.48 V2版本
       while(j--);
    }
}


void FIFO_READ(unsigned char len)
{	
	unsigned char i,j,data;	
	CSB=1;
	FCSB=0;
	SCLK=0;
	SDIOC=1;
	for(j=0;j<len;j++)
	{
		FCSB=0;
		for(i=0;i<8;i++)
		{
			data=0;
			for(i=0;i<8;i++)
			{
				data<<=1;
				SCLK=1;
				if(SDIO)		data|=1;
				_nop();
				SCLK=0;
			}
		}
		a_rx[j]=data;
		FCSB=1;
		GCC_DELAY(10);
	}
	SCLK=0;
	SDIOC=0;
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
		else 				SDIO=0;
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
		_nop();
		SCLK=1;
		address<<=1;
	}
	SDIOC=1;
	SCLK=0;
	GCC_DELAY(10);
	data=0;
	for(i=0;i<8;i++)
	{
		data<<=1;
		SCLK=1;
		if(SDIO)		data|=1;
		_nop();
		SCLK=0;
	}
	GCC_DELAY(10);
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
	
	//Enable LOCKING
	tmp=SPI_READ(CMT2300A_CUS_EN_CTL);		//adress 0x60
	tmp|=CMT2300A_MASK_LOCKING_EN;			//LOCKING_EN=1;
	SPI_WRITE(CMT2300A_CUS_EN_CTL, tmp);
	
	//Diable LFOSC 
	tmp=SPI_READ(CMT2300A_CUS_SYS2);
    tmp &= (unsigned char)~CMT2300A_MASK_LFOSC_RECAL_EN;
    tmp &= (unsigned char)~CMT2300A_MASK_LFOSC_CAL1_EN;
    tmp &= (unsigned char)~CMT2300A_MASK_LFOSC_CAL2_EN;
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

	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO1_SEL_DOUT|CMT2300A_GPIO2_SEL_INT1|CMT2300A_GPIO3_SEL_INT2);	//INT1 > GPIO1   ,INT2 > GPIO3
	tmp=CMT2300A_INT_SEL_RSSI_VLD;
	tmp|=CMT2300A_INT_POLAR_SEL_1;
	SPI_WRITE(CMT2300A_CUS_INT1_CTL, tmp);
	tmp=SPI_READ(CMT2300A_CUS_INT1_CTL);
	
//	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO3_SEL_INT2);	//INT2 > GPIO3
//	tmp=SPI_READ(CMT2300A_CUS_INT2_CTL);
	tmp=CMT2300A_INT_SEL_PKT_DONE;						//
	SPI_WRITE(CMT2300A_CUS_INT2_CTL, tmp);				//CMT2300A_INT_SEL_PKT_DONE
	SPI_WRITE(CMT2300A_CUS_INT_EN,CMT2300A_MASK_PKT_DONE_EN);	//CMT2300A_MASK_TX_DONE_EN
	
	
	//enable read fifo
	tmp=SPI_READ(CMT2300A_CUS_FIFO_CTL);
	tmp &= ~CMT2300A_MASK_SPI_FIFO_RD_WR_SEL; 
	tmp &= ~CMT2300A_MASK_FIFO_RX_TX_SEL;
   	SPI_WRITE(CMT2300A_CUS_FIFO_CTL,tmp);
   	//clr rx fifo
   	SPI_WRITE(CMT2300A_CUS_FIFO_CLR,CMT2300A_MASK_FIFO_CLR_RX);
	//go_tx
   	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_RX);
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
	GCC_DELAY(60000);
	//测试CMT是否存在
	back = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, 0xAA);
    tmp = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, back);
    if(tmp!=0xAA) while(1);
    
    SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
    SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_RX);
}

void CMT_RX()
{
//	unsigned char tmp;
	if(!RF_INT)
	{
		FIFO_READ(4);
		SPI_WRITE(CMT2300A_CUS_INT_CLR1,CMT2300A_MASK_PKT_DONE_CLR); //clr PKT_DONE int flag
		SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
	//	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_RX);
	//	GCC_DELAY(200);
	//	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
		
		if((a_rx[0]==0x55)&&(a_rx[1]==0xaa)&&(a_rx[2]==0xaa))
		{
            switch(a_rx[3]&0x0f)
            {
                case 0x01:
                    f_vibra=1;
                    a_motor=200;
                    break;
                case 0x02:
                    f_stim=1;
                    break;
                case 0x03:
                    f_bz=1;
                    a_bz=200;
                    break;
                default:
                    break;
            }

            a_stim_level=(a_rx[3]>>4)&0x0f;
		}
		a_rx[0]=0;
		a_rx[1]=0;
		a_rx[2]=0;
		a_rx[3]=0;
		a_3min=0;
	}
}

void KEY()
{
    ON_OFFC=1;
    ON_OFFUP=1;
    if(!ON_OFF)
    {
        a_onoff_high=0;
        if(!f_onoff_buf)
        {
            a_onoff_low++;
            if(a_onoff_low>=5)
            {
                a_onoff_low=0;
                f_onoff=1;
                f_onoff_buf=1; 
            }
        }
    }
    else 
    {
        a_onoff_low=0;
        if(f_onoff_buf)
        {
            a_onoff_high++;
            if(a_onoff_high>=5)
            {
                a_onoff_high=0;
                f_onoff_buf=0;
            }   
        }
    }
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
		while(_eocb);
		sum+=_adrh;
		if(_adrh>ad_max) 		ad_max=_adrh;
		else if(_adrh<ad_min) 	ad_min=_adrh;
	}
	sum-=ad_max;
	sum-=ad_min;
	sum>>=4;
//	_adoff=1;
	return sum;
}


void SetVoltageLevel(unsigned char level)
{
	if((level<1)&&(!f_bz))
	{
        GREEN_LED=0;
        RED_LED^=1;
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

		if(ad_voltage_buf>=c_voltage_3V51)	
		{
			if(!f_voltage_buf0)
			{
				a_voltage_level=1;
				f_voltage_buf0=1;
			}
		}
		else 
		{
			if(!f_voltage_buf0)
			{
				a_voltage_level=0;			
				f_voltage_buf0=1;
			}
		}
		ad_voltage_buf=0;
	}

	SetVoltageLevel(a_voltage_level);
}


void ChargeDetect()
{
    a_charge_full_wait=200;

    if(!CHRG&&!f_dc_connect)     //充电中
    {
		GCC_DELAY(10);
		if(!CHRG)
		{
            f_dc_connect=1;
            GCC_DELAY(10);
            if(!CHRG&&CHRG_PU)
            {
                GCC_DELAY(10);
                if(!CHRG&&CHRG_PU)
                {
        			GREEN_LED=0;
                    RED_LED=1;
                }
            }
		}
    }
#if 1
    if(CHRG&&CHRG_PU&&f_dc_connect)
    {
		GCC_DELAY(10);
		if(CHRG&&CHRG_PU)
		{
	        CHRG_PUC=1;
	        CHRG_PUUP=0;
#if 0
            while(CHRG)
            {
                RED_LED=1;
                GREEN_LED=0;
                GCC_DELAY(263690);
                RED_LED=0;
                GREEN_LED=1;
                GCC_DELAY(263690);
                _clrwdt();
            }
            
            while(1)
            {
    			GREEN_LED=1;
                RED_LED=0;
                GCC_DELAY(20);
                _clrwdt();
            }
#endif
        while(1)
        {
            GCC_DELAY(100);
            _clrwdt();
            GREEN_LED=1;
            RED_LED=1;
        }

#if 0
            while(a_charge_full_wait--)
            {
                if(!CHRG)  //冲满未拔下
                {
        			GCC_DELAY(10);
        			if(!CHRG)
        			{
            			GREEN_LED=1;
                        RED_LED=0;
                        break;
        			}
                }
        		else  //充满已拔下，未充电
        		{
        			GCC_DELAY(10);
        			if(CHRG&&a_charge_full_wait==0)
        			{
            			GREEN_LED=0;
                        RED_LED=0;
                        f_dc_connect=0;
        			}
        		}
                _clrwdt();
                GCC_DELAY(200);
            }
#endif
            CHRG_PUC=0;
            CHRG_PU=1;
		}
    }
    #endif
}

void DoAction()
{
    int j;

    if(f_vibra)
    {
        if(a_motor--)
        {
            MOTOR_EN=1;
            GREEN_LED=0;
            RED_LED=1;
        }
        else
        {
            f_vibra=0;
            MOTOR_EN=0;
            GREEN_LED=0;
            RED_LED=0;
        }
    }

    if(f_stim)
    {
        RED_LED=1;
        GREEN_LED=0;
        switch(a_stim_level)
        {
            case 1:
                loop_count = 4;    //added on 2018/05/17
				for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)   //added on 2017/10/15
					{
						HV_EN = 1;    //高电平
						//GCC_DELAY(3);
						GCC_DELAY(4);
						HV_EN = 0;    //低电平					
						GCC_DELAY(182);

						delay_ms(4);
					}
				}
              break;
			case 2:
			    loop_count = 8;    //added on 2018/05/17
				for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(6);
						
						HV_EN = 0;
						GCC_DELAY(180);
					
						delay_ms(4);
					}
				}
			  break;
			case 3:
			    loop_count = 12;    //added on 2018/05/17
				for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(9);
						
						HV_EN = 0;
						GCC_DELAY(177);
					
						delay_ms(4);
					}
				}
			  break;
		    case 4:
		        loop_count = 18;    //added on 2018/05/17
				for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(13);
						
						HV_EN = 0;
						GCC_DELAY(173);
					
						delay_ms(4);
					}
				}
			  break;
			case 5:
			    loop_count = 24;    //added on 2018/05/17
				for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(18);
						
						HV_EN = 0;
						GCC_DELAY(168);
					
						delay_ms(4);
					}
				}
			  break;
			case 6:
			    loop_count = 32;    //added on 2018/05/17
				for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(24);
						
						HV_EN = 0;
						GCC_DELAY(162);
					
						delay_ms(4);
					}
				}
			  break;  
			case 7:
			    loop_count = 40;    //added on 2018/05/17
			  	for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(31);
						
						HV_EN = 0;
						GCC_DELAY(155);
					
						delay_ms(4);
					}
				}
			  break;
			case 8:
			    loop_count = 50;    //added on 2018/05/17
			  	for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(38);
						GCC_DELAY(39);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(148);
						GCC_DELAY(147);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;
			case 9:
			  loop_count = 60;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(45);
						GCC_DELAY(48);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(141);
						GCC_DELAY(138);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;
			case 10:
			  loop_count = 70;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(52);
						GCC_DELAY(58);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(134);
						GCC_DELAY(128);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;
			case 11:
			  loop_count = 80;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(59);
						GCC_DELAY(68);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(127);
						GCC_DELAY(118);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;
			case 12:
			  loop_count = 90;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(66);
						GCC_DELAY(78);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(120);
					    GCC_DELAY(108);   //modified on 2018/05/17
					    
						delay_ms(4);
					}
				}
			  break;
			case 13:
			  loop_count = 100;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(73);
						GCC_DELAY(88);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(113);
						GCC_DELAY(98);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;
			case 14:
			  loop_count = 110;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(80);
						GCC_DELAY(98);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(106);
						GCC_DELAY(88);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;
			case 15:
			  loop_count = 120;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(87);
						GCC_DELAY(108);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(99);
						GCC_DELAY(78);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
				
				break;
			case 16:
			  loop_count = 130;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						//modified on 2018/05/17
						//GCC_DELAY(94);
						GCC_DELAY(118);
						//modified end
						
						HV_EN = 0;
						//GCC_DELAY(92);
						GCC_DELAY(68);   //modified on 2018/05/17
					
						delay_ms(4);
					}
				}
			  break;  
			default:
			  loop_count = 8;    //added on 2018/05/17
			  for(j=0;j<loop_count;j++)
				{
					if (time_elec_high <= 3)  //added on 2017/10/15
					{
						HV_EN = 1;
						//GCC_DELAY(4);   //第一个6us //modified on 2018/05/08
						GCC_DELAY(6);

						HV_EN = 0;
						GCC_DELAY(180);
					
						delay_ms(4);
					}
				}
			  break;               
		}
        f_stim=0;
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
	
	FCSBC=0;
	CSBC=0;
	SCLKC=0;
	SDIOC=0;
	SCLK=1;
	BZC=0;
	BZ=0;
	//

	//125us
	_tm1al=250;
	_tm1ah=0x0;
	//fsys/4
	_t1ck2=0;
	_t1ck1=0;
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
	
	CMT_init();
//	f_k2=1;
	RF_INTC=1;
	RF_INTUP=1;
	RF_INTWU=1;

	_papu=0xff;
	_pbpu=0xff;
	_pcpu=0xff;
	HV_ENC=0;
	HV_EN=0;
//	DCDC_ENC=0;
//	DCDC_EN=0;
	RSTC=1;
	IC_DATAC=1;
	ON_OFFC=1;
	IC_CLKC=1;
	
	CHRGC=1;
	CHRGWU=1;
	CHRG_PUC=0;
	CHRG_PU=1;
	
	RED_LEDC=0;
	RED_LED=0;

	GREEN_LEDC=0;
	GREEN_LED=0;
	
	MOTOR_ENC=0;
	MOTOR_EN=0;
	BAT_DETC=1;

    time_elec_high=0;
    loop_count=0;

	//TestViaLed();
}	
	
void main()
{
//	unsigned char i;
	initail();
	while(1)
	{
		_clrwdt();
		ChargeDetect();
        if(f_dc_connect)
        {            
            a_3min=0;
        }
        else
        {
    		CMT_RX();
    		if(f_10ms)
    		{
    			f_10ms=0;
                //Voltage();
                KEY();
                if(f_onoff)
                {
                    f_onoff=0;
                    GREEN_LED=1;
                    RED_LED=1;
                }
                DoAction();
                if(f_halt)
    			{
    				f_halt=0;
    				_we4=1;
    				_we3=0;
    				_we2=1;
    				_we1=0;
    				_we0=1;
    				_emi=0;
    				_idlen=0;
    				SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
    				//T1=2ms,Tsleep=713
    				SPI_WRITE(0x0f,0x92);
    				SPI_WRITE(0x10,0x53);
    				SPI_WRITE(0x11,0x3E);
    			//	SPI_WRITE(0x15,0x20);		//SLP Disable				//----------------------
    				SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep

    				//GREEN_LED=0;
    				_halt();
    				//GREEN_LED=1;

    				_we4=0;
    				_we3=1;
    				_we2=0;
    				_we1=1;
    				_we0=0;
    				_emi=1;
    				a_3min=0;
    				SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
    				//T1=3ms,Tsleep=168
    				SPI_WRITE(0x0f,0x40);
    				SPI_WRITE(0x10,0x51);
    				SPI_WRITE(0x11,0x4b);
    				SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep	
    			}
    		}            
        }
	}
}
