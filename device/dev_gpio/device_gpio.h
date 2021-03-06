#ifndef __INC_device_gpio_H__
#define __INC_device_gpio_H__


//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "stdint.h"
#include "GPIO/drive_gpio.h"
#include "lw_oopc.h"
#include "dev_char.h"
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

CLASS( devGpio)
{
	IMPLEMENTS( I_dev_Char);
	driveGpio		*dri;
	
	uint8_t		minor;
	uint8_t		none[3];
	
	
};


//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


devGpio *Get_DevGpio(int minor);


#endif
