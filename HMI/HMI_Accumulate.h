#ifndef _INC_Accumulate_HMI_HMI_H_
#define _INC_Accumulate_HMI_HMI_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "HMI.h"
#include "HMI_comm.h"
#include <stdint.h>


//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


 //------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
CLASS(Accm_HMI)
{
	EXTENDS(HMI);
	sheet		*p_sht_info;
	strategy_t *p_stt;
	
	uint8_t		cur_chn;
	uint8_t		none[3];
	
	
	
	
};




//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------
//extern strategy_t	g_AccDay_strategy, g_AccMonth_strategy;
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
Accm_HMI *Get_Accm_HMI(void);

#endif
