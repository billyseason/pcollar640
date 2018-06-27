#include "HT66F018.h"
#include "head.h"
#include "cmt2300a_defs.h"


volatile unsigned char a_100ms,a_count,a_10ms;
unsigned char a_k1_high,a_k1_low,a_k2_high,a_k2_low,a_k3_high,a_k3_low,a_k4_high,a_k4_low,a_k5_high,a_k5_low;
volatile flag_byte f_flag;
volatile unsigned char a_data,a_count,a_rx[4],a_read[0x20];
volatile unsigned int a_bz;


//1500us
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
		}
		a_100ms++;
//		if(a_100ms>=250)
//		{
//			a_100ms=0;
//			a_count++;
//			if(a_count>9)	a_count=0;	
//			lcd_data[0]=c_num[a_count];
//		}		
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

	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO1_SEL_DOUT|CMT2300A_GPIO3_SEL_INT2);	//INT1 > GPIO1   ,INT2 > GPIO3
//	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO1_SEL_INT1|CMT2300A_GPIO3_SEL_DOUT);
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
	
	tmp=SPI_READ(CMT2300A_CUS_SYS10);						//SLP13
	tmp|=13;
	SPI_WRITE(CMT2300A_CUS_SYS10,tmp); 
	
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
	GCC_DELAY(60000);
	//测试CMT是否存在
	back = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, 0xAA);
    tmp = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, back);
    if(tmp!=0xAA) while(1);    
  	
}

void CMT_RX()
{
	unsigned char tmp;   
//	tmp= SPI_READ(CMT2300A_CUS_INT_FLAG);
	if(!RF_INT)
	{
		FIFO_READ(4);
		SPI_WRITE(CMT2300A_CUS_INT_CLR2,CMT2300A_MASK_PKT_DONE_CLR); //clr TX_DONE int flag
		SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
		if((a_rx[0]==0x55)&&(a_rx[1]==0xaa)&&(a_rx[2]==0xaa)&&(a_rx[3]==0x55))
		{
			f_bz=1;
			a_bz=200;	
		}
		a_rx[0]=0;
		a_rx[1]=0;
		a_rx[2]=0;
		a_rx[3]=0;
		while(1)
		{
//			if(!f_bz)
			{
				_we4=1;
				_we3=0;
				_we2=1;
				_we1=0;
				_we0=1;
				_emi=0;
				_idlen=0;
				_clrwdt();
				GCC_HALT();	
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
	f_k2=1;
}	
	
void main()
{
//	unsigned char i;
	initail();
	while(1)
	{
		_clrwdt();
		if(f_10ms)
		{	
			f_10ms=0;
			CMT_RX();
		}	
	}
}