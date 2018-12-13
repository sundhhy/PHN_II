#ifndef _INC_barGhHMI_H_
#define _INC_barGhHMI_H_
#include "HMI.h"
#include "HMI_comm.h"
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define BARHMI_NUM_BARS			6		//����С��NUM_CHANNEL
 //------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
CLASS( HMI_bar)
{
	EXTENDS(HMI);
	IMPLEMENTS(mdl_observer);

//	sheet  			*p_bar_clean;		//��ͼ�Ĳ���
	sheet  			*arr_p_barshts[NUM_CHANNEL];
//	sheet  			*arr_p_sht_textPrcn[BARHMI_NUM_BARS];
//	sheet  			**pp_bar_unit;
//	uint8_t		focusRow;
//	uint8_t		focusCol;
//	char		flag;
	uint16_t		arr_bar_height[NUM_CHANNEL];		//180213 ��¼��ͼ�ĸ߶ȣ������Ż���ͼ��ˢ��
	
};
//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
HMI_bar *Get_barGhHMI(void);

#endif