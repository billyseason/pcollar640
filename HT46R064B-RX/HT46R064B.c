#include "HT46R064B.h"
#include "cmt2300a_defs.h"

#define FCSB	_pa0
#define CSB		_pa1
#define SCLK	_pa3
#define SDIO	_pa2

#define FCSBC	_pac0
#define CSBC	_pac1
#define SCLKC	_pac3
#define SDIOC	_pac2
#define SDIOUP	_papu2

#define CMT_IO1		_pb0
#define CMT_IO1C	_pbc0

unsigned char a_data,a_count,a_rev[2],a_tx[2];

const unsigned char c_cmt_init[0x60]={
							 0x02,0x66,0xec,0x1c,0xf0,0x80,0x14,0x08,0x91,0x02,0x02,0x0d,	//CMT Bank
							 0xae,0xe0,0x70,0x00,0x00,0xf4,0x10,0xe2,0x42,0x20,0x00,0x81,	//System Bank
							 0x42,0x71,0xCE,0x1C,0x42,0x5B,0x1C,0x1C,						//Freq Bank
							 0x32,0x18,0x00,0x99,0xC1,0x9B,0x06,0x0a,0x9F,0x39,0x29,0x29,0xC0,0x51,0x2A,0x53,0x00,0x00,0xB4,0x00,0x00,0x01,0x00,0x00,	//Data Rate Bank
							 0x12,0x08,0x00,0xAA,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0xD4,0x2D,0x00,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0xFF,0x01,0x00,0x1F,0x10, //Baseband
							 0x50,0x26,0x03,0x00,0x42,0xB0,0x00,0x01,0x01,0x3F,	//Reserve Bank
							 0x7F,												//LBD Bank	
							 };

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
		data=0;
		for(i=0;i<8;i++)
		{
			data<<=1;
			SCLK=1;
			if(SDIO)		data|=1;	
			_nop();	
			SCLK=0;		
		}
		a_rev[j]=data;
		FCSB=1;	
		_delay(10);	
	}
	SDIOC=0;
	SCLK=0;	
	FCSB=1;	
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
		FCSB=1;	
		_delay(10);	
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
	_delay(5);
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
	_delay(40000);			//20ms
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
	
	tmp=SPI_READ(CMT2300A_CUS_MODE_STA);				//address 0x61
	tmp|=CMT2300A_MASK_CFG_RETAIN;						//Enable CFG_RETAIN 
	tmp&=(unsigned char)~CMT2300A_MASK_RSTN_IN_EN;		//Disable RSTN_IN 
	SPI_WRITE(CMT2300A_CUS_MODE_STA,tmp);
	
	tmp=SPI_READ(CMT2300A_CUS_EN_CTL);		//adress 0x60
	tmp|=CMT2300A_MASK_LOCKING_EN;			//LOCKING_EN=1;
	SPI_WRITE(CMT2300A_CUS_EN_CTL, tmp);				
	
	for(tmp=0;tmp<0x60;tmp++)
	{
		SPI_WRITE(tmp,c_cmt_init[tmp]);
		_clrwdt();	
	}

//	tmp=((unsigned char)~0x07)&SPI_READ(CMT2300A_CUS_CMT10);		//RFPDK 1.46以后，可忽略
//	SPI_WRITE(CMT2300A_CUS_CMT10, tmp|0x02);
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep	
	
	SPI_WRITE(CMT2300A_CUS_IO_SEL,CMT2300A_GPIO1_SEL_INT1|CMT2300A_GPIO2_SEL_INT2|CMT2300A_GPIO3_SEL_DOUT);	//INT1 > GPIO1;INT2 > GPIO2
	tmp=SPI_READ(CMT2300A_CUS_INT1_CTL);
	tmp|=CMT2300A_INT_SEL_PKT_DONE|CMT2300A_INT_POLAR_SEL_1;
	SPI_WRITE(CMT2300A_CUS_INT1_CTL, tmp);				//CMT2300A_INT1_SEL_PKT_DONE,GPIO1
	tmp=SPI_READ(CMT2300A_CUS_INT2_CTL);
	tmp|=CMT2300A_INT_SEL_RX_ACTIVE;
	SPI_WRITE(CMT2300A_CUS_INT1_CTL, tmp);				//CMT2300A_INT_SEL_RX_ACTIVE
	SPI_WRITE(CMT2300A_CUS_INT_EN,CMT2300A_MASK_PKT_DONE_EN);
	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);
	//测试CMT是否存在
	back = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, 0xAA);
    tmp = SPI_READ(CMT2300A_CUS_PKT17);
    SPI_WRITE(CMT2300A_CUS_PKT17, back);
    if(tmp!=0xAA) while(1);    
    
   	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_STBY);	//go_stdby
	SPI_WRITE(CMT2300A_CUS_INT_CLR2,CMT2300A_MASK_PKT_DONE_CLR);  //clr PKT_DONE int flag	
	//enable read fifo
	tmp=SPI_READ(CMT2300A_CUS_FIFO_CTL);
	tmp &= ~(unsigned char)CMT2300A_MASK_SPI_FIFO_RD_WR_SEL; 
   	tmp &= ~(unsigned char)CMT2300A_MASK_FIFO_RX_TX_SEL;
   	SPI_WRITE(CMT2300A_CUS_FIFO_CTL,tmp);
   	//clr rx fifo
   	SPI_WRITE(CMT2300A_CUS_FIFO_CLR,CMT2300A_MASK_FIFO_CLR_RX);
   	//go_rx
   	SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_RX);	
}

void CMT_RX()
{
	unsigned char i;	
	if(CMT_IO1)
	{
		FIFO_READ(2);
		SPI_WRITE(CMT2300A_CUS_INT_CLR2,CMT2300A_MASK_PKT_DONE_CLR); //clr PKT_DONE int flag
		SPI_WRITE(CMT2300A_CUS_MODE_CTL,CMT2300A_GO_SLEEP);	//go_sleep
		if(!(a_rev[0]^a_rev[1]))	_pb0^=1;
	}	
}

void initail()
{
	FCSBC=0;
	CSBC=0;
	SCLKC=0;
	SDIOC=0;
	FCSB=1;
	CSB=1;
	SCLK=0;
	SDIO=0;	
	SDIOUP=1;
//	CMT_init();
	a_tx[0]=0x55;
	a_tx[1]=0xaa;
}

void main()
{
	initail();
	while(1)
	{
		_clrwdt();
		FIFO_WRITE(2);
		_delay(1000);
	}
}