//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//
#include "HMI/HMIFactory.h"

#include "system.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sdhDef.h"
#include "utils/rtc_pcf8563.h"
#include "utils/hw_w25q.h"
#include "utils/FM25.h"
#include "utils/log.h"

#include "fs/easy_fs.h"
#include "utils/Storage.h"

#include "Modbus_app.h"
#include "channel_accumulated.h"
#include "cfg_test.h"
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define 	PHN_MAJOR_VER				4
#define 	PHN_MINOR_VER				0

#define 	PHN_DEF_SUPER_PSD_1			0x01			
#define 	PHN_DEF_SUPER_PSD_2			0x01
#define 	PHN_DEF_SUPER_PSD_3			0x01



//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

static UtlRtc *sys_rtc;	
//为了能够紧凑的定义一些静态变量，所以都定义在一起
//只有非4字节对齐的，需要放在这里定于
char 				*arr_p_vram[18];
//char				g_setting_chn = 0;
char				flush_flag = 0;
//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
system_t		phn_sys;
//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//要在存储功能初始化之后调用
void System_power_on(void)
{
	Storage			*stg = Get_storage();
	rcd_alm_pwr_t			stg_pwr = {0};
	int						num_pwr = 0;
	
	
	
	
	
//	STG_Set_file_position(STG_LOSE_PWR, STG_DRC_READ, 0);
//	while(stg_pwr.flag != 0xff)
//	{
//		if(stg->rd_stored_data(stg, STG_LOSE_PWR, \
//			&stg_pwr, sizeof(rcd_alm_pwr_t)) != sizeof(rcd_alm_pwr_t))
//			{
//				
//				//或者已经读完了
//				break;
//				
//				
//			}
//			if(stg_pwr.flag != 0xff)
//				num_pwr ++;
//		
//	}
//	
//	if(num_pwr == STG_MAX_NUM_LST_PWR)
//		num_pwr = 0;	
//	phn_sys.pwr_rcd_index = num_pwr;

	//获取当前的掉电信息的最后位置
	phn_sys.pwr_rcd_index = STG_Get_alm_pwr_num(STG_LOSE_PWR);
	if(phn_sys.pwr_rcd_index > STG_MAX_NUM_LST_PWR)
		phn_sys.pwr_rcd_index = 0;
	
	//读取最后一条掉电记录的掉电时间作为上一次掉电时间
	STG_Set_file_position(STG_LOSE_PWR, STG_DRC_WRITE, (phn_sys.pwr_rcd_index - 1) * sizeof(rcd_alm_pwr_t));
	stg->rd_stored_data(stg, STG_LOSE_PWR, &stg_pwr, sizeof(rcd_alm_pwr_t));
	phn_sys.last_poweroff_time = stg_pwr.disapper_time_s;
	
	//记录上电时间
	stg_pwr.flag = 1;
	stg_pwr.happen_time_s = SYS_time_sec();
	stg_pwr.disapper_time_s = 0xffffffff;
	STG_Set_file_position(STG_LOSE_PWR, STG_DRC_WRITE, phn_sys.pwr_rcd_index * sizeof(rcd_alm_pwr_t));
	
	stg->wr_stored_data(stg, STG_LOSE_PWR, &stg_pwr, sizeof(rcd_alm_pwr_t));
	
	//在记录中增加数量
	num_pwr = phn_sys.pwr_rcd_index + 1;
	num_pwr %= STG_MAX_NUM_LST_PWR;
	STG_Set_alm_pwr_num(STG_LOSE_PWR, num_pwr);
	
}

void System_power_off(void)
{
	short 				retry = 5;
	short				i = 0;
	uint32_t		dsp_time = 0;
	Storage			*stg = Get_storage();
	
//	return;		//todo:debug
	
	phn_sys.sys_flag |= SYSFLAG_URGENCY;
	//掉电信息无效时，就不要存储掉电时间了
	//当擦执行了擦除掉电信息操作的时候，会出现这种情况
	if(phn_sys.pwr_rcd_index != 0xff)
	{
	
		
	
		dsp_time = SYS_time_sec();
		STG_Set_file_position(STG_LOSE_PWR, STG_DRC_WRITE, \
		phn_sys.pwr_rcd_index * sizeof(rcd_alm_pwr_t) +(int)(&((rcd_alm_pwr_t *)0)->disapper_time_s));
		
		while(stg->wr_stored_data(stg, STG_LOSE_PWR, &dsp_time, sizeof(uint32_t)) != sizeof(uint32_t))
		{
			if(retry)
				retry --;
			else
				break;
			
		}
	}
	for(i = 0; i < NUM_CHANNEL; i++)
		MCH_Cancle_all_alarm(i);
//	stg->wr_stored_data(stg, STG_SYS_CONF, &phn_sys.sys_conf, sizeof(phn_sys.sys_conf));
	
	phn_sys.fs.fs_shutdown();
	phn_sys.sys_flag &= ~SYSFLAG_URGENCY;

}
#include "control/CtlTimer.h"
void SYS_Reset(int thoroughly, int num_chn)
{
	int chn_num;
	int	recode_file_size;
//	Controller		*p_control;

//	if(thoroughly)
//	{
//		p_control = SUPER_PTR(Get_ctl_time(), Controller);
//		if(p_control == NULL) while(1);
//		p_control->destory(p_control);
//		
//	}
	EFS_Reset(thoroughly);
	System_default(num_chn);
	
	recode_file_size = STG_Equally_recode_stg(num_chn);
	for(chn_num = 0; chn_num < num_chn; chn_num ++)
	{
		MdlChn_default_conf(chn_num);
		MdlChn_default_alarm(chn_num);
		MdlChn_set_rcd_file_size(chn_num, recode_file_size);
		MdlChn_Commit_conf(chn_num);
	}
	STG_Reset(num_chn);
	CNA_default(num_chn);
	
}

void System_default(int num_chn)
{
	system_conf_t 	*p_sc = &phn_sys.sys_conf;
	Storage			*stg = Get_storage();
	uint8_t		password[3];
	uint8_t		super_psd[3];
	
	//密码不要随意复位
	
//	Clone_psd(p_sc->super_psd, super_psd);
	Clone_psd(p_sc->password, password);
	memset(p_sc, 0, sizeof(system_conf_t));
//	Clone_psd(super_psd, p_sc->super_psd);
	Clone_psd(password, p_sc->password);
	
	
	p_sc->sys_flag = 0;
	p_sc->num_chn = num_chn;
	p_sc->cold_end_way = 0;
	p_sc->id = 1;
	p_sc->record_gap_s = 1;
	
	p_sc->baud_idx = 2;
	p_sc->baud_rate = arr_baud[2];
	p_sc->show_chn_status = 0;
	p_sc->storage_alarm = STG_DEFAULT_ALARM;
	
	
	while(stg->wr_stored_data(stg, STG_SYS_CONF, &phn_sys.sys_conf, sizeof(phn_sys.sys_conf)) != RET_OK)
	{
		
		delay_ms(10);
	}
	
//	if((p_sc->super_psd[0] == 0xff) && (p_sc->super_psd[1] == 0xff) && (p_sc->super_psd[2] == 0xff))
//	{
//		p_sc->super_psd[0] = PHN_DEF_SUPER_PSD_1;
//		p_sc->super_psd[1] = PHN_DEF_SUPER_PSD_2;
//		p_sc->super_psd[2] = PHN_DEF_SUPER_PSD_3;
//	}
	
//	if((p_sc->password[0] == 0xff) && (p_sc->password[1] == 0xff) && (p_sc->password[2] == 0xff))
	{
		p_sc->password[0] = 0;
		p_sc->password[1] = 0;
		p_sc->password[2] = 0;
	}
	

}

void System_init(void)
{
//	struct  tm stm;
	Model 		*m;
	Storage					*stg = Get_storage();
	char			chn_name[7];
	char			i;
//	int				retry = 5;

	phn_sys.major_ver = PHN_MAJOR_VER;
	phn_sys.minor_ver = PHN_MINOR_VER;
	
	
	sys_rtc = ( UtlRtc *)Pcf8563_new();
	if(sys_rtc == NULL) while(1);

	sys_rtc->init(sys_rtc, NULL);
	sys_rtc->get(sys_rtc, &phn_sys.sys_time);
	
	//md_time要系统时间初始化之后初始化
	m = Create_model("time");
	m->init(m, NULL);
	
	
	
	w25q_init();
	FM25_init();
	
#if UNT_ON	== 0
	EFS_init(NUM_FSH, 0);
#else
	EFS_init(NUM_FSH, 0);
#endif
	
#if UNT_ON	== 0
	stg->init(stg);
	
	
	
	/*  把所有的文件按照固定的顺序打开一遍，来保证每个仪表在第一次上电时，文件的存储位置是一致的*/
	//这里只处理FM25上的文件
	//通道的数据文件，会自行根据先后顺序排列
	stg->open_file(STG_SYS_CONF, STG_DEF_FILE_SIZE);
	
	//掉电信息以及素有通道的报警和累积信息都是在同一个文件里面的，因此打开一次就行
	stg->open_file(STG_CHN_ALARM(0), STG_DEF_FILE_SIZE);			

	stg->open_file(STG_LOG, STG_DEF_FILE_SIZE);
	
	System_power_on();
	
	LOG_Init();
	stg->rd_stored_data(stg, STG_SYS_CONF, &phn_sys.sys_conf, sizeof(phn_sys.sys_conf));
	if((phn_sys.sys_conf.num_chn > NUM_CHANNEL) || (phn_sys.sys_conf.num_chn == 0))
		System_default(NUM_CHANNEL);
	
	
	phn_sys.sys_conf.super_psd[0] = PHN_DEF_SUPER_PSD_1;
	phn_sys.sys_conf.super_psd[1] = PHN_DEF_SUPER_PSD_2;
	phn_sys.sys_conf.super_psd[2] = PHN_DEF_SUPER_PSD_3;
	
	
	CNA_Init();
	for(i = 0; i < phn_sys.sys_conf.num_chn; i++)
	{

		sprintf(chn_name,"chn_%d", i);
		m = Create_model(chn_name);
		m->init(m, &i);
		
	}
	
	
	MdlChn_Read_code_end_temperature();
//	System_power_off();
#endif	
	
}

//todo:坏味道的代码
extern void Ctime_Allco_time(uint16_t  all_time, uint8_t need);
int SYS_Commit(void)
{
	Storage					*stg = Get_storage();
	
	stg->wr_stored_data(stg, STG_SYS_CONF, &phn_sys.sys_conf, sizeof(phn_sys.sys_conf));
	Ctime_Allco_time(phn_sys.sys_conf.record_gap_s, NUM_CHANNEL);
	
	//Modbus的串口参数也可能被修改了，所有要重新初始化MODBUS的串口
	MBA_Init_uart(phn_sys.sys_conf.baud_rate);
	return RET_OK;
}
void System_time(struct  tm *stime)
{
	
	sys_rtc->get(sys_rtc, stime);
}


int  System_set_time(struct  tm *stime)
{
	int ret;
	ret = sys_rtc->set(sys_rtc, stime);
	
	if(ret == RET_OK)
		sys_rtc->get(sys_rtc, stime);
	
	return ret;
}



uint32_t  SYS_time_sec(void)
{
	
 	return Time_2_u32(&phn_sys.sys_time);
//	Model 		*m;
//	uint32_t	sec = 0;
//	m = Create_model("time");
//	m->getMdlData(m, TIME_U32, &sec);
	
//	return sec;
	
	
//	struct tm  t;
//	sys_rtc->get(sys_rtc, &t);
//	
//	return Time_2_u32(&t);
}










//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//
/// \name Private Functions
/// \{





