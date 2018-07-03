#include "HT66F018.h"
#include "head.h"
#include "cmt2300a_defs.h"


volatile unsigned char a_count,a_10ms,a_100ms,a_1s,a_3min;
unsigned char a_k1_high,a_k1_low,a_k2_high,a_k2_low,a_k3_high,a_k3_low,a_k4_high,a_k4_low,a_k5_high,a_k5_low;
volatile flag_byte f_flag;
volatile unsigned char a_data,a_count,a_rx[4],a_read[0x20],a_20s,f_20s;
volatile unsigned int a_bz;


//125us
void __attribute((interrupt(0x10)))	TM1_int()
{
	if(_t1af)
	{
		_t1af=0;
		if(f_bz)
		{
			if(a_bz--)	 BZ^=1;
			else         f_bz=0;
		}	 
		else BZ=0;
		a_10ms++;
		if(a_10ms>=80)
		{
			a_10ms=0;
			f_10ms=1;	
			a_1s++;
			if(a_1s>=100)
			{
				a_1s=0;
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
		
		if((a_rx[0]==0x55)&&(a_rx[1]==0xaa)&&(a_rx[2]==0xaa)&&(a_rx[3]==0x55))
		{
			f_bz=1;
			a_bz=50;	
		}
		a_rx[0]=0;
		a_rx[1]=0;
		a_rx[2]=0;
		a_rx[3]=0;
		a_3min=0;
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
	
	RF_INTC=1;	
	RF_INTUP=1;
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
	_pac0=1;
	_papu0=1;
	_pawu0=1;

	_papu=0xff;
	_pbpu=0xff;
	_pcpu=0xff;
	HV_ENC=0;
	HV_EN=0;
	DCDC_ENC=0;
	DCDC_EN=0;
	RSTC=1;
	IC_DATAC=1;
	ON_OFFC=1;
	IC_CLKC=1;
	CHRGC=1;
	RED_LEDC=0;
	RED_LED=0;
	CHRG_PULLUPC=1;
	GREEN_LEDC=0;
	GREEN_LED=0;
	MOTOR_ENC=0;
	MOTOR_EN=0;
	BAT_DETC=1;

}	
	
void main()
{
//	unsigned char i;
	initail();
	while(1)
	{
		_clrwdt();
		CMT_RX();
		if(f_10ms)
		{	
			f_10ms=0;			
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
						
				GREEN_LED=0;
				_halt();
				GREEN_LED=1;
				
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
