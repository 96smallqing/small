#include <stc15.h>
#include <intrins.h>
typedef unsigned char uchar;
typedef unsigned char uint;
//****************************************IO端口定义***************************************
sbit  MISO = P3^2;
sbit  MOSI = P3^5;
sbit  SCK  = P3^4;
sbit  CE   = P3^6;
sbit  CSN  = P3^7;
sbit  IRQ  = P3^3;

sbit led0 = P4^3;
sbit led1 = P4^2;
sbit led2 = P4^1;
sbit led3 = P4^0;
//********************* ************************NRF24L01*************************************
#define TX_ADR_WIDTH    5    // 5 uints TX address width
#define RX_ADR_WIDTH    5    // 5 uints RX address width
#define TX_PLOAD_WIDTH  32   // 20 uints TX payload
#define RX_PLOAD_WIDTH  32   // 20 uints TX payload
uint const TX_ADDRESS[TX_ADR_WIDTH]= {0x10,0x10,0x10,0x10,0x10}; //本地地址	0x10,0x10,0x10,0x10,0x10
uint const RX_ADDRESS[RX_ADR_WIDTH]= {0x01,0x01,0x01,0x01,0x01}; //接收地址      
//***************************************NRF24L01寄存器指令*******************************************************
#define READ_REG        0x00  //读配置寄存器,低5位为寄存器地址
#define WRITE_REG       0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读取接收数据指令  读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写待发数据指令    写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //冲洗发送 FIFO指令 清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //冲洗接收 FIFO指令 清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //定义重复装载数据指令  重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //保留              空操作,可以用来读状态寄存器
//*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式 配置寄存器地址
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置  接收地址允许
#define SETUP_AW        0x03  // 收发地址宽度设置(所有数据通道)
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置  RF通道
#define RF_SETUP        0x06  // 发射速率、功耗功能设置  RF寄存器
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测     载波检测寄存器         
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度 (1~32字节)
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//**************************************************************************************
void Delay(unsigned int s);
void inerDelay_us(unsigned char n);
void init_NRF24L01(void);
uint SPI_RW(uint uchar);
uchar SPI_Read(uchar reg);
void SetRX_Mode(void);
uint SPI_RW_Reg(uchar reg, uchar value);
uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars);
uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars);
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf);
void nRF24L01_TxPacket(unsigned char * tx_buf);
//*****************************************长延时*****************************************
void Delay(unsigned int s)
{
	unsigned int i;
	for(i=0; i<s; i++);
	for(i=0; i<s; i++);
}
//******************************************************************************************
uint  bdata sta;   		//状态标志
sbit RX_DR =sta^6;	   	//判断是否接收到数据	  若收到数据 则被置1
sbit TX_DS =sta^5;
sbit MAX_RT =sta^4;
/******************************************************************************************
/*延时函数
/******************************************************************************************/
void inerDelay_us(unsigned char n)
{
	for(;n>0;n--)
    _nop_();
}
//****************************************************************************************
/*NRF24L01初始化
//***************************************************************************************/
void init_NRF24L01(void)
{
	/*
		两个nrf24l01通信，需要满足3个条件相同：
		1.频道相同（设置频道寄存器RF_CH）
		2.地址相同（设置TX_ADDR和RX_ADDR_P0相同）
		3.每次发送接收的字节数相同（如果设置了通道的有效数据宽度为n，那么每次发送的字节数也必须为n，当然，n<=32）
	*/
    inerDelay_us(100);
    CE=0;    // chip enable  芯片使能
    CSN=1;   // Spi disable  SPI禁用
    SCK=0;   // Spi clock line init high   SPI时钟线
	IRQ=1;
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址     与接收代码的接收端地址相同
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址
	
	//有了以下这三个配置，发送方的流程就变成了发送-触发中断。这样就抛开了接收方，可以专心去调试发送
//	SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);                 // 失能通道0自动应答
//  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x00);            // 失能接收通道0
//	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);          // 失能自动重发
	
	//以上三句只是调试方法，最终的产品如果不加上应答和重发的话那么数据的稳定性是很难保证的，所以在基本的通讯建立之后就要把发送的配置改为以下这三个配置 
	//这样发送和接收就进入了一个标准状态，发送-等应答-（自动重发）-触发中断；接收-应答-触发中断，一切按部就班，程序里加上自己的应用部分就能实现很多功能了
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);               // 使能接收通道0自动应答
    SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);           // 使能接收通道0               
    SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a);          // 自动重发10次，间隔500us
	
	SPI_RW_Reg(WRITE_REG + SETUP_AW, 0x02); // Setup address width=5 bytes  安装地址宽度＝5字节	  
	SPI_RW_Reg(WRITE_REG + RF_CH, 0);        			//设置信道工作为2.4GHZ，收发必须一致
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); 	//设置接收数据长度，本次设置为32字节
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);     		//设置发射速率为2MHZ，发射功率为最大值0dB
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      			// IRQ收发完成中断响应，16位CRC，发射模式
}   //ps:波特率用设置么？IRQ用拉高么？不响应中断？
/****************************************************************************************************
/*函数：uint SPI_RW(uint uchar)
/*功能：NRF24L01的SPI写时序  NRF24L01最基本的SPI通信
/****************************************************************************************************/
uint SPI_RW(uint uuchar)
{
	uint bit_ctr;
    for(bit_ctr=0;bit_ctr<8;bit_ctr++) 	// output 8-bit
    {
		  MOSI = (uuchar & 0x80);         	// output 'uchar', MSB to MOSI  输出“uchar”，MSB到MOSI
		  uuchar = (uuchar << 1);           // shift next bit into MSB..    将下一位移到MSB
		  SCK = 1;                     		// Set SCK high..               设置SCK高。
		  uuchar |= MISO;           		// capture current MISO bit     获取当前 MISO bit
		  SCK = 0;                			// ..then set SCK low again     然后再次设置SCK低电平 
    }
    return(uuchar);               			// return read uchar            返回读取uchar
}
/****************************************************************************************************
/*函数：uchar SPI_Read(uchar reg)
/*功能：NRF24L01的SPI时序   SPI读寄存器一字节函数  
/****************************************************************************************************/
uchar SPI_Read(uchar reg)
{
    uchar reg_val;

	  CSN = 0;                // CSN low, initialize SPI communication... CSN低，初始化SPI通信…
	  SPI_RW(reg);            // Select register to read from..           写寄存器地址  选择寄存器读取
	  reg_val = SPI_RW(0);    // ..then read registervalue                然后读取注册值 写入读寄存器指令
	  CSN = 1;                // CSN high, terminate SPI communication    CSN高，终止SPI通信
	  return(reg_val);        // return register value                    返回寄存器值
}
/****************************************************************************************************/
/*功能：NRF24L01写寄存器函数	向寄存器reg写一个字节，同时返回状态字节
/****************************************************************************************************/
uint SPI_RW_Reg(uchar reg, uchar value)
{
		uint status;
		CSN = 0;                   // CSN low, init SPI transaction  CSN置低 进入SPI通信
		status = SPI_RW(reg);      // select register                选择寄存器
		SPI_RW(value);             // ..and write value to it..      并将其写入值
		CSN = 1;                   // CSN high again                 CSN再高
		return(status);            // return nRF24L01 status uchar   返回nRF24L01状态uchar
}
/****************************************************************************************************/
/*函数：uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
/*功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
/****************************************************************************************************/

uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uint status,uchar_ctr;	
	CSN = 0;                      // Set CSN low, init SPI tranaction  CSN置低 进入SPI通信
	status = SPI_RW(reg);         // Select register to write to and read status uchar  选择寄存器写入和读取状态uCHAR  写入要读取的寄存器地址	
	for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)       //读取数据
		pBuf[uchar_ctr] = SPI_RW(0);    //	
	CSN = 1;                           	
	return(status);                    // return nRF24L01 status uchar
}

/*********************************************************************************************************
/*函数：uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
/*功能: 写入bytes字节的数据  用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
/*********************************************************************************************************/
uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uint status,uchar_ctr;
	CSN = 0;            //SPI使能      
	status = SPI_RW(reg);   					//写入要写入寄存器的地址
	for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++) //写入数据
		SPI_RW(*pBuf++);
	CSN = 1;           //关闭SPI
	return(status);    //
}
/****************************************************************************************************/
/*函数：void SetRX_Mode(void)
/*功能：数据接收配置
/****************************************************************************************************/

void SetRX_Mode(void)
{
	CE=0;
	//SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // IRQ收发完成中断响应，16位CRC ，主接收
	CE = 1;
	inerDelay_us(130);						
}

/******************************************************************************************************/
/*函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
/*功能：数据读取后放如rx_buf接收缓冲区中
/******************************************************************************************************/

unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
    unsigned char revale=0;
	sta=SPI_Read(STATUS); 	// 读取状态寄存其来判断数据接收状况
	if(RX_DR)    			// 判断是否接收到数据
	{
	     CE = 0;    		//SPI使能
	     SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer	从RXXFIFO缓冲器读取接收有效载荷
	     revale =1;   		//读取数据完成标志
	}
	SPI_RW_Reg(WRITE_REG+STATUS,sta);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
	return revale;
}

/***********************************************************************************************************
/*函数：void nRF24L01_TxPacket(unsigned char * tx_buf)
/*功能：发送 tx_buf中数据
/**********************************************************************************************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	CE=0;   				//StandBy I模式
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	// 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);     			// 装载数据
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      							//IRQ收发完成中断响应，16位CRC，主发送
	CE=1;   				//置高CE，激发数据发送
	inerDelay_us(50);
}
/****************************************************************************************************/
/*函数：void SetTX_Mode(void)
/*功能：数据发送配置
/****************************************************************************************************/

void SetTX_Mode(void)
{
	CE=0;	
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     							//IRQ收发完成中断响应，16位CRC ，主接收	
	CE = 1;
	inerDelay_us(200);													//配置寄存器使芯片工作于发送模式后拉高CE端至少10us
}
 /********************************************/
/* 函数功能：检测24L01是否存在 */
/* 返回值； 1 存在 */
/* 2 不存在 */
/********************************************/
uchar NRF24L01_Check(void)
{
int NRF_CE=0;
uchar check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
uchar check_out_buf[5]={0x00};
//SendStr("check Init\r\n");


SPI_Write_Buf(WRITE_REG+TX_ADDR, check_in_buf, 5);

SPI_Read_Buf(READ_REG+TX_ADDR, check_out_buf, 5);

if((check_out_buf[0] == 0x11)&&\
(check_out_buf[1] == 0x22)&&\
(check_out_buf[2] == 0x33)&&\
(check_out_buf[3] == 0x44)&&\
(check_out_buf[4] == 0x55))return 1;
else return 2;
}
/***********************************************************************/
void ledstar(void){
		led1=0;
	  Delay(2100);  //约10ms	
		led1=1;


}
//************************************主函数************************************************************
void main()
{

	unsigned char a;  //模块是否存在标志位
	unsigned char TxBuf[20]={0x31,0x32};    //将要发送的数据放在数组中
//unsigned char TxBuf[20]={0x1,0x32,0x33};    //将要发送的数据放在数组中
	//led0 =0;
//	led3 =0;
	P0M1=0;P0M0=0;P1M1=0;P1M0=0;
	P2M1=0;P2M0=0;P3M1=0;P3M0=0;
	P4M1=0;P4M0=0;P5M1=0;P5M0=0;
	init_NRF24L01();							//NRF初始化
	a=NRF24L01_Check();
	nRF24L01_TxPacket(TxBuf);					//发送数据
	
	while(1)
	{
	
		/*
			&#8226;配置寄存器使芯片工作于发送模式后拉高CE端至少10us
			&#8226;读状态寄存器STATUS
			&#8226;判断是否是发送完成标志位置位
			&#8226;清标志
			&#8226;清数据缓冲
		*/
			if(a == 1){
				led0 = 0;
				led3 =0;
			}
			else if(a == 2){
			led1=0;
			led2=0;			
			}
			else{led0=0;led1=0;led2=0;led3=0;}
				
			init_NRF24L01();
			SetTX_Mode();						 	//NRF初始化
			nRF24L01_TxPacket(TxBuf);			 	//发送数据
			P0=SPI_Read(STATUS);				 	//读状态寄存器的值  如果数据成功发送，那么STATUS的值应该为0x2e
			SPI_RW_Reg(WRITE_REG+STATUS,0XFF);   	//清状态寄存器
			ledstar();//led1闪烁							  
	}
} 