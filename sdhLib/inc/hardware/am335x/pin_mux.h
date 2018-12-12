/*
 * pin_mux.h
 *
 *  Created on: 2016-7-7
 *      Author: Administrator
 */

#ifndef PIN_MUX_H_
#define PIN_MUX_H_


#define conf_gpmc_ad0			(0x0800)
#define conf_gpmc_ad1			(0x0804)
#define conf_gpmc_ad2			(0x0808)
#define conf_gpmc_ad3			(0x080C)
#define conf_gpmc_ad4			(0x0810)
#define conf_gpmc_ad5			(0x0814)
#define conf_gpmc_ad6			(0x0818)
#define conf_gpmc_ad7			(0x081C)
#define conf_gpmc_ad8			(0x0820)
#define conf_gpmc_ad9			(0x0824)
#define conf_gpmc_ad10			(0x0828)
#define conf_gpmc_ad11			(0x082C)
#define conf_gpmc_ad12			(0x0830)
#define conf_gpmc_ad13			(0x0834)
#define conf_gpmc_ad14			(0x0838)
#define conf_gpmc_ad15			(0x083C)
#define conf_gpmc_a0			(0x0840)
#define conf_gpmc_a1			(0x0844)
#define conf_gpmc_a2			(0x0848)
#define conf_gpmc_a3			(0x084C)
#define conf_gpmc_a4			(0x0850)
#define conf_gpmc_a5			(0x0854)
#define conf_gpmc_a6			(0x0858)
#define conf_gpmc_a7			(0x085C)
#define conf_gpmc_a8			(0x0860)
#define conf_gpmc_a9			(0x0864)
#define conf_gpmc_a10			(0x0868)
#define conf_gpmc_a11			(0x086C)
#define conf_gpmc_wait0			(0x0870)
#define conf_gpmc_wpn			(0x0874)
#define conf_gpmc_be1n			(0x0878)
#define conf_gpmc_csn0			(0x087C)
#define conf_gpmc_csn1			(0x0880)
#define conf_gpmc_csn2			(0x0884)
#define conf_gpmc_csn3			(0x0888)
#define conf_gpmc_clk			(0x088C)
#define conf_gpmc_advn_ale		(0x0890)
#define conf_gpmc_oen_ren		(0x0894)
#define conf_gpmc_wen			(0x0898)
#define conf_gpmc_be0n_cle		(0x089C)
#define conf_lcd_data0			(0x08A0)
#define conf_lcd_data1			(0x08A4)
#define conf_lcd_data2			(0x08A8)
#define conf_lcd_data3			(0x08AC)
#define conf_lcd_data4			(0x08B0)
#define conf_lcd_data5			(0x08B4)
#define conf_lcd_data6			(0x08B8)
#define conf_lcd_data7			(0x08BC)
#define conf_lcd_data8			(0x08C0)
#define conf_lcd_data9			(0x08C4)
#define conf_lcd_data10			(0x08C8)
#define conf_lcd_data11			(0x08CC)
#define conf_lcd_data12			(0x08D0)
#define conf_lcd_data13			(0x08D4)
#define conf_lcd_data14			(0x08D8)
#define conf_lcd_data15			(0x08DC)
#define conf_lcd_vsync			(0x08E0)
#define conf_lcd_hsync			(0x08E4)
#define conf_lcd_pclk			(0x08E8)
#define conf_lcd_ac_bias_en		(0x08EC)
#define conf_mmc0_dat3			(0x08F0)
#define conf_mmc0_dat2			(0x08F4)
#define conf_mmc0_dat1			(0x08F8)
#define conf_mmc0_dat0			(0x08FC)
#define conf_mmc0_clk			(0x0900)
#define conf_mmc0_cmd			(0x0904)
#define conf_mii1_col			(0x0908)
#define conf_mii1_crs			(0x090C)
#define conf_mii1_rxerr			(0x0910)
#define conf_mii1_txen			(0x0914)
#define conf_mii1_rxdv			(0x0918)
#define conf_mii1_txd3			(0x091C)
#define conf_mii1_txd2			(0x0920)
#define conf_mii1_txd1			(0x0924)
#define conf_mii1_txd0			(0x0928)
#define conf_mii1_txclk			(0x092C)
#define conf_mii1_rxclk			(0x0930)
#define conf_mii1_rxd3			(0x0934)
#define conf_mii1_rxd2			(0x0938)
#define conf_mii1_rxd1			(0x093C)
#define conf_mii1_rxd0			(0x0940)
#define conf_rmii1_refclk		(0x0944)
#define conf_mdio_data			(0x0948)
#define conf_mdio_clk			(0x094C)
#define conf_spi0_sclk			(0x0950)
#define conf_spi0_d0			(0x0954)
#define conf_spi0_d1			(0x0958)
#define conf_spi0_cs0			(0x095C)
#define conf_spi0_cs1			(0x0960)
#define conf_ecap0_in_pwm0_out	(0x0964)
#define conf_uart0_ctsn			(0x0968)
#define conf_uart0_rtsn			(0x096C)
#define conf_uart0_rxd			(0x0970)
#define conf_uart0_txd			(0x0974)
#define conf_uart1_ctsn			(0x0978)
#define conf_uart1_rtsn			(0x097C)
#define conf_uart1_rxd			(0x0980)
#define conf_uart1_txd			(0x0984)
#define conf_i2c0_sda			(0x0988)
#define conf_i2c0_scl			(0x098C)
#define conf_mcasp0_aclkx		(0x0990)
#define conf_mcasp0_fsx			(0x0994)
#define conf_mcasp0_axr0		(0x0998)
#define conf_mcasp0_ahclkr		(0x099C)
#define conf_mcasp0_aclkr		(0x09A0)
#define conf_mcasp0_fsr			(0x09A4)
#define conf_mcasp0_axr1		(0x09A8)
#define conf_mcasp0_ahclkx		(0x09AC)
#define conf_xdma_event_intr0	(0x09B0)
#define conf_xdma_event_intr1	(0x09B4)
#define conf_nresetin_out		(0x09B8)
#define conf_porz				(0x09BC)
#define conf_nnmi				(0x09C0)
#define conf_osc0_in			(0x09C4)
#define conf_osc0_out			(0x09C8)
#define conf_rsvd1				(0x09CC)
#define conf_tms				(0x09D0)
#define conf_tdi				(0x09D4)
#define conf_tdo				(0x09D8)
#define conf_tck				(0x09DC)
#define conf_ntrst				(0x09E0)
#define conf_emu0				(0x09E4)
#define conf_emu1				(0x09E8)
#define conf_osc1_in			(0x09EC)
#define conf_osc1_out			(0x09F0)
#define conf_pmic_power_en		(0x09F4)
#define conf_rtc_porz			(0x09F8)
#define conf_rsvd2				(0x09FC)
#define conf_ext_wakeup			(0x0A00)
#define conf_enz_kaldo_1p8v		(0x0A04)
#define conf_usb0_dm			(0x0A08)
#define conf_usb0_dp			(0x0A0C)
#define conf_usb0_ce			(0x0A10)
#define conf_usb0_id			(0x0A14)
#define conf_usb0_vbus			(0x0A18)
#define conf_usb0_drvvbus		(0x0A1C)
#define conf_usb1_dm			(0x0A20)
#define conf_usb1_dp			(0x0A24)
#define conf_usb1_ce			(0x0A28)
#define conf_usb1_id			(0x0A2C)
#define conf_usb1_vbus			(0x0A30)
#define conf_usb1_drvvbus		(0x0A34)
#define conf_ddr_resetn			(0x0A38)
#define conf_ddr_csn0			(0x0A3C)
#define conf_ddr_cke			(0x0A40)
#define conf_ddr_ck				(0x0A44)
#define conf_ddr_nck			(0x0A48)
#define conf_ddr_casn			(0x0A4C)
#define conf_ddr_rasn			(0x0A50)
#define conf_ddr_wen			(0x0A54)
#define conf_ddr_ba0			(0x0A58)
#define conf_ddr_ba1			(0x0A5C)
#define conf_ddr_ba2			(0x0A60)
#define conf_ddr_a0				(0x0A64)
#define conf_ddr_a1				(0x0A68)
#define conf_ddr_a2				(0x0A6C)
#define conf_ddr_a3				(0x0A70)
#define conf_ddr_a4				(0x0A74)
#define conf_ddr_a5				(0x0A78)
#define conf_ddr_a6				(0x0A7C)
#define conf_ddr_a7				(0x0A80)
#define conf_ddr_a8				(0x0A84)
#define conf_ddr_a9				(0x0A88)
#define conf_ddr_a10			(0x0A8C)
#define conf_ddr_a11			(0x0A90)
#define conf_ddr_a12			(0x0A94)
#define conf_ddr_a13			(0x0A98)
#define conf_ddr_a14			(0x0A9C)
#define conf_ddr_a15			(0x0AA0)
#define conf_ddr_odt			(0x0AA4)
#define conf_ddr_d0				(0x0AA8)
#define conf_ddr_d1				(0x0AAC)
#define conf_ddr_d2				(0x0AB0)
#define conf_ddr_d3				(0x0AB4)
#define conf_ddr_d4				(0x0AB8)
#define conf_ddr_d5				(0x0ABC)
#define conf_ddr_d6				(0x0AC0)
#define conf_ddr_d7				(0x0AC4)
#define conf_ddr_d8				(0x0AC8)
#define conf_ddr_d9				(0x0ACC)
#define conf_ddr_d10			(0x0AD0)
#define conf_ddr_d11			(0x0AD4)
#define conf_ddr_d12			(0x0AD8)
#define conf_ddr_d13			(0x0ADC)
#define conf_ddr_d14			(0x0AE0)
#define conf_ddr_d15			(0x0AE4)
#define conf_ddr_dqm0			(0x0AE8)
#define conf_ddr_dqm1			(0x0AEC)
#define conf_ddr_dqs0			(0x0AF0)
#define conf_ddr_dqsn0			(0x0AF4)
#define conf_ddr_dqs1			(0x0AF8)
#define conf_ddr_dqsn1			(0x0AFC)
#define conf_ddr_vref			(0x0B00)
#define conf_ddr_vtp			(0x0B04)
#define conf_ddr_strben0		(0x0B08)
#define conf_ddr_strben1		(0x0B0C)
#define conf_ain7				(0x0B10)
#define conf_ain6				(0x0B14)
#define conf_ain5				(0x0B18)
#define conf_ain4				(0x0B1C)
#define conf_ain3				(0x0B20)
#define conf_ain2				(0x0B24)
#define conf_ain1				(0x0B28)
#define conf_ain0				(0x0B2C)
#define conf_vrefp				(0x0B30)
#define conf_vrefn				(0x0B34)
#define conf_avdd				(0x0B3C)
#define conf_avss				(0x0B3C)
#define conf_iforce				(0x0B40)
#define conf_vsense				(0x0B44)
#define conf_testout			(0x0B48)
#define cqdetect_status			(0x0E00)
#define ddr_io_ctrl				(0x0E04)
#define vtp_ctrl				(0x0E0C)
#define vref_ctrl				(0x0E14)
#define tpcc_evt_mux_0_3		(0x0F90)
#define tpcc_evt_mux_4_7		(0x0F94)
#define tpcc_evt_mux_8_11		(0x0F98)
#define tpcc_evt_mux_12_15		(0x0F9C)
#define tpcc_evt_mux_16_19		(0x0FA0)
#define tpcc_evt_mux_20_23		(0x0FA4)
#define tpcc_evt_mux_24_27		(0x0FA8)
#define tpcc_evt_mux_28_31		(0x0FAC)
#define tpcc_evt_mux_32_35		(0x0FB0)
#define tpcc_evt_mux_36_39		(0x0FB4)
#define tpcc_evt_mux_40_43		(0x0FB8)
#define tpcc_evt_mux_44_47		(0x0FBC)
#define tpcc_evt_mux_48_51		(0x0FC0)
#define tpcc_evt_mux_52_55		(0x0FC4)
#define tpcc_evt_mux_56_59		(0x0FC8)
#define tpcc_evt_mux_60_63		(0x0FCC)
#define timer_evt_capt			(0x0FD0)
#define ecap_evt_capt			(0x0FD4)
#define adc_evt_capt			(0x0FD8)
#define reset_iso				(0x1000)
#define ddr_cke_ctrl			(0x131C)
#define sma2 Section			(0x1320)
#define m3_txev_eoi				(0x1324)
#define ipc_msg_reg0			(0x1328)
#define ipc_msg_reg1			(0x132C)
#define ipc_msg_reg2			(0x1330)
#define ipc_msg_reg3			(0x1334)
#define ipc_msg_reg4			(0x1338)
#define ipc_msg_reg5			(0x133C)
#define ipc_msg_reg6			(0x1340)
#define ipc_msg_reg7			(0x1344)
#define ddr_cmd0_ioctrl			(0x1404)
#define ddr_cmd1_ioctrl			(0x1408)
#define ddr_cmd2_ioctrl			(0x140C)
#define ddr_data0_ioctrl		(0x1440)
#define ddr_data1_ioctrl		(0x1444)


#define SLEWCTRL	(0x1 << 6)
#define	RXACTIVE	(0x1 << 5)
#define	PULLUP_EN	(0x1 << 4) /* Pull UP Selection */
#define PULLUDEN	(0x0 << 3) /* Pull up enabled */
#define PULLUDDIS	(0x1 << 3) /* Pull up disabled */
#define MODE(val)	val


#define SOC_CONTROL_REGS    (0x44E10000)
#define SOC_AINTC_REGS      (0x48200000)
#define INTC_ILR(n)  (0x100 + ((n) * 0x04))
#define INTC_ILR_PRIORITY    (0x000001FC)
#define INTC_MIR_CLEAR(n) (0x88 + ((n) * 0x20))

int config_pin( int pin_off, int mode, int conf);
uint32_t rd_pin_conf(int pin_off );
#endif /* PIN_MUX_H_ */
