//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#ifndef __INC_unit_test_H_
#define __INC_unit_test_H_
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


//------------------------------------------------------------------------------
// global variable declarations
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
void Unit_test(void);
void	UNT_Disp_text(char	*text, int	line, int	row);
void UNT_Clean_lcd(void);

void	UNT_Delay_ms(int ms);
//这个函数由各个单元测试程序提供
int Init_test(char	*test_buf, int size);
//这个函数由各个单元测试程序提供
void Run_test(void);		


#endif
