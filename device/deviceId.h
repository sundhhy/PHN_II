#ifndef __DEVICEID_H_
#define __DEVICEID_H_

//�ַ����豸
#define DEVTYPE_CHAR				1


#	define DEVMAJOR_UART				1
#	define DEVMAJOR_GPIO				2
#	define DEVMAJOR_SPI					3
#	define DEVMAJOR_IIC					4
//��ʾ�豸
#define DEVTYPE_DISPLAY				2

#define DEVMAJOR_LCD				1


//�豸ID 
// | 31 .. 24 	| 23 .. 16 	|	15 .. 8		| 7 .. 0 	|
// | ����		| �豸����	|	���豸��	| ���豸��	|


#define DEVID_RESERVE_BITS		8
#define DEVID_TYPE_BITS			8
#define DEVID_MAJOR_BITS		8
#define DEVID_MINOR_BITS		8
#define DEVID_TYPE_MASK			0xff
#define DEVID_MAJOR_MASK		0xff
#define DEVID_MINOR_MASK		0xff

#define	SET_TYPE( type) 				((type & DEVID_TYPE_MASK) << (DEVID_MAJOR_BITS + DEVID_MINOR_BITS)) 
#define	SET_MAJOR( major) 				((major & DEVID_MAJOR_MASK) << (DEVID_MINOR_BITS))
#define	SET_MINOR( minor) 				(minor & DEVID_MINOR_MASK)

#define BUILD_DEVID( type, major, minor) ( SET_TYPE( type)  | SET_MAJOR( major)  | SET_MINOR( minor))

//#define DEVID_UART2  		BUILD_DEVID( DEVTYPE_CHAR, DEVMAJOR_UART, 1)
//#define DEVID_USARTGPULCD  	BUILD_DEVID( DEVTYPE_DISPLAY, DEVMAJOR_LCD, 1)
#define DEVID_UART1			0x00010100
#define DEVID_UART2			0x00010101
#define DEVID_UART3			0x00010102
#define DEVID_UART4			0x00010103
#define DEVID_SPI1			BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_SPI, 0)
#define DEVID_SPI2			BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_SPI, 1)
#define DEVID_SPI3			BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_SPI, 2)

#define DEVID_IIC1			BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_IIC, 0)
#define DEVID_IIC2			BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_IIC, 1)

//˳��Ҫ��dev_gpio.c�е�arr_Gpiocfg һ�£�
//ÿ��GPIO��16������
//Ӧ�ð��շ���������gpio���豸ID������Ϊ�˳����ܹ��򵥵Ĵ��������ǰ���˳�������壬������϶
//#define DEVID_GPIO_A10		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x0A)

//#define DEVID_GPIO_D0		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x40)
//#define DEVID_GPIO_D1		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x41)
//#define DEVID_GPIO_D2		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x42)
//#define DEVID_GPIO_D3		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x43)
//#define DEVID_GPIO_D4		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x44)
//#define DEVID_GPIO_D5		BUILD_DEVID(DEVTYPE_CHAR, DEVMAJOR_GPIO, 0x45)
#define DEVID_GPIO_D0			0x00010200
#define DEVID_GPIO_D1			0x00010201
#define DEVID_GPIO_D2			0x00010202
#define DEVID_GPIO_D3			0x00010203
#define DEVID_GPIO_D4			0x00010204
#define DEVID_GPIO_D5			0x00010205
#define DEVID_GPIO_A10			0x00010206
#define DEVID_GPIO_PWR			0x00010207

#define DEVID_USARTGPULCD	0x00020101







#endif