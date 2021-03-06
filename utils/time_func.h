//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#ifndef __INC_time_func_H_
#define __INC_time_func_H_
#include "time.h"

//------------------------------------------------------------------------------
// check for correct compilation options
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

int TMF_Get_num_mday(int year, int mon);
int TMF_Check_tm(struct  tm	*p_t);
int TMF_Str_2_tm(char *s_time, struct  tm	*time);
uint32_t  Str_time_2_u32(char *s_time);
uint32_t  Time_2_u32(struct  tm	*tm_2_sec);
int 			Sec_2_tm(uint32_t seconds, struct  tm *time);


#endif
