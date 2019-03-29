#include "stdio.h"
#include "PN102Series.h"
#include "rwip_config.h"


#include <string.h>             // for memcpy
#include "co_utils.h"           // common utility definition
#include "co_math.h"            // common math functions
#include "co_endian.h"          // endian definitions
#include "rf.h"                 // RF interface
#include "rf_spi.h"
#include "rwip.h"               // for RF API structure definition

#if defined(CFG_BLE)
#include "reg_blecore.h"        // ble core registers
#include "reg_ble_em_cs.h"      // control structure definitions
#include "reg_modem.h"
#endif


#define DBG_RF_CALIBRATION_TPCODE_TABLE_GENERATE_ENABLE     1

#define RF_CALIBRATION_TIMES            6


#define RF_CALIB_TBL_INDX_MIN           0
#define RF_CALIB_TBL_INDX_MAX           14

#define RF_CALIB_TBL_INDX_FIRST         7 //(RF_CALIB_TBL_INDX_MIN + RF_CALIB_TBL_INDX_MAX) / 2

#define RF_CALIB_VALUE_MIN              0
#define RF_CALIB_VALUE_MAX              15

#define RF_CALIB_VALUE_VALID_MIN        1
#define RF_CALIB_VALUE_VALID_MAX        14

#define RF_CALIB_VALUE_VALID_LOW        6
#define RF_CALIB_VALUE_VALID_HIGH       9


typedef struct 
{
    //uint8_t da_gain;
    uint8_t da_vref_lb;
    uint8_t da_vref_mb;
} rf_cal_tbl_t;

static uint16_t m_rf_calib_times = 0;

static uint8_t m_rf_cal_da_gain = 0;

const static rf_cal_tbl_t m_rf_cal_tbl[] = 
{
    { 0, 0 },    // 0
    { 1, 0 },    // 1
    { 2, 0 },    // 2
    { 3, 0 },    // 3
    { 4, 0 },    // 4
    { 5, 0 },    // 5
    { 6, 0 },    // 6
    { 7, 0 },    // 7
    { 7, 1 },    // 8
    { 7, 2 },    // 9
    { 7, 3 },    // 10
    { 7, 4 },    // 11
    { 7, 5 },    // 12
    { 7, 6 },    // 13
    { 7, 7 },    // 14
};


/**
 * 使用两点式校准 调节 GAUSS_SCALE，可调节范围 190KHz
 * 测试一颗芯片带外Deviation变化范围在108KHz~554KHz之间;
 * 与DAC输出幅度相关配置DA_VREF_MB、DA_VREF_LB、DA_GAIN、GAUSS_SCALE控制Deviation范围和步进
 */
static uint8_t gauss_scale_generate ( uint8_t da_gain, uint8_t da_vref_lb, uint8_t da_vref_mb );

/**
 * 使用 m_rf_cal_tbl 中的几组特征值 来进行两点式校准
 */
static uint8_t gauss_scale_generate_sample ( uint8_t index );

/* 使用二分法 (递归) 检查两点式校准的校准值 是否有效
 * 注意: 
 * 1. 每次都会执行两点式校准, 校准次数 @RF_CALIBRATION_TIMES
 **/
static uint8_t gauss_scale_search ( uint8_t index_low, uint8_t index_high, uint8_t range_min, uint8_t range_max );

/**
 * 执行 RF 校准 (两点式校准)
 **/
static uint8_t rf_calibration_run ( uint8_t times );

/**
 * 使用几组特征值 来进行两点式校准
 * 第一个参数 RF_CALIB_TBL_INDX_FIRST, 后续采用二分法依次检查
 **/
static uint8_t rf_calibration_tpcode_generate ( uint8_t index_low, uint8_t index_high, uint8_t range_min, uint8_t range_max );


uint8_t gauss_scale_generate ( uint8_t da_gain, uint8_t da_vref_lb, uint8_t da_vref_mb )
{
    uint8_t result = 0xFF;
    
    //clear [14:20]
    ANAC->TX_CTL &= ~ 0x1fc000;
    
    // set DA_GAIN, DA_VREF_LB, DA_VREF_MB
    ANAC->TX_CTL |= (da_gain << 14) | (da_vref_lb << 15) | (da_vref_mb << 18);
    
    result = rf_calibration_run ( RF_CALIBRATION_TIMES ); 
    m_rf_calib_times++;
    
    DBG_SYS ( DBG_RF_CALIB_TIMES,       m_rf_calib_times );
    DBG_SYS ( DBG_RF_CALIB_DA_GAIN,     da_gain );
    DBG_SYS ( DBG_RF_CALIB_DA_VREF_LB,  da_vref_lb );
    DBG_SYS ( DBG_RF_CALIB_DA_VREF_MB,  da_vref_mb );
    DBG_SYS ( DBG_RF_CALIB_DA_RESULT,   result );
    
    printf("(%d, %d, %d) = %d\n" , da_gain, da_vref_lb, da_vref_mb, result );
    
    return result;
}

uint8_t gauss_scale_generate_sample ( uint8_t index )
{
    //if ( index <= RF_CALIB_TBL_INDX_MAX )
    
    return gauss_scale_generate ( m_rf_cal_da_gain, 
                                  m_rf_cal_tbl[index].da_vref_lb, 
                                  m_rf_cal_tbl[index].da_vref_mb );
}

uint8_t gauss_scale_search ( uint8_t index_low, uint8_t index_high, uint8_t range_min, uint8_t range_max )
{
    uint8_t result = 0xFF;
    
    if ( index_low <= index_high )
    {
        uint8_t index_temp = (index_low + index_high) / 2;
        
        result = gauss_scale_generate_sample ( index_temp );
        
        if ( result < range_min )
        {
            result = gauss_scale_search ( index_temp, index_high, range_min, range_max );
        }
        else if ( result > range_max )
        {
            result = gauss_scale_search ( index_low, index_temp, range_min, range_max );
        }
        // else
        // result is valid, do nothing.
    }
    
    return result;
}

uint8_t rf_calibration_run ( uint8_t times )
{
    uint8_t i, j, tmp, rst;
    uint8_t result[(RF_CALIBRATION_TIMES - 1) * 8] = { 0 };
    
    for ( i = 0; i < times; i++ )
    {
        //配manul_code,当entpcal为0时使用manul
        set_ui32_reg(ANAC->TP_CTL, 0x00000008);
        TIMER_Delay(TIMER2, 100);
        
        //打开ENTXSYN
        set_ui32_reg(ANAC->TP_CTL, 0x01000008);
        TIMER_Delay(TIMER2, 100);
        
        //打开PA
        set_ui32_reg(ANAC->TP_CTL, 0x05000008);
        TIMER_Delay(TIMER2, 100);
        
        //使能两点式校正中断和EN信号
        set_ui32_reg(ANAC->TP_CTL, 0x05000308);
        TIMER_Delay(TIMER2, 100);
        
        //tpspitrig = 1
        set_ui32_reg(ANAC->TP_CTL, 0x05000388);
        TIMER_Delay(TIMER2, 100);
        
        //tpspitrig = 0 ,形成trig下降沿中断，触发校正
        set_ui32_reg(ANAC->TP_CTL, 0x05000308);
        
        //等待校正结束 此处至少需要100ms
        while(!(ANAC->TP_STS & 0x00000002));
        ANAC->TP_STS |= 1;
        
        if ( i >= 1 )
        {
            result[i - 1] = (ANAC->TP_CTL >> 16) & 0xf;
        }
        
        //printf("%d\n",result[i]);
    }
    
    // 将校准结果排序, 取中间值作为有效值
    for ( i = 0; i < times - 1; i++ )
    {
        for ( j = 0; j < times - i - 1; j++ )
        {
            if ( result[j] < result[j + 1] )
            {
                tmp = result[j];
                result[j] = result[j + 1];
                result[j + 1] = tmp;
            }
        }
    }
    
    // 取排序后的 中间值作为有效值
    rst = result[times / 2];
    
    //printf("Two Point final result is: %d\n", rst);
    
    return rst;
}

uint8_t rf_calibration_tpcode_generate ( uint8_t index_low, uint8_t index_high, uint8_t range_min, uint8_t range_max )
{
    uint8_t result = 0xFF;
    
    result = gauss_scale_generate_sample ( RF_CALIB_TBL_INDX_FIRST );
    
    if ( result > range_max )
    {
        result = gauss_scale_generate_sample ( RF_CALIB_TBL_INDX_MIN );
        
        if ( result < range_min )
        {
            result = gauss_scale_search ( RF_CALIB_TBL_INDX_MIN, RF_CALIB_TBL_INDX_FIRST, range_min, range_max );
        }
        else if ( result > range_max )
        {
            // invalid
            result = 0xFF;
        }
        // else
        // result is valid, do nothing.
    }
    else if ( result < range_min )
    {
        result = gauss_scale_generate_sample ( RF_CALIB_TBL_INDX_MAX );
        
        if ( result > range_max )
        {
            result = gauss_scale_search ( RF_CALIB_TBL_INDX_FIRST, RF_CALIB_TBL_INDX_MAX, range_min, range_max );
        }
        else if ( result < range_min )
        {
            // invalid
            result = 0xFF;
        }
        // else
        // result is valid, do nothing.
    }
    // else
    // result is valid, do nothing.
    
    return result;
}

#if DBG_RF_CALIBRATION_TPCODE_TABLE_GENERATE_ENABLE
void rf_calibration_tpcode_table_generate ( void )
{
    printf("TPCODE table drawing \n");
    
    uint8_t tpcode[2*8*8] = { 0 };
    uint8_t index         = 0;
    
    uint8_t da_gain;
    uint8_t da_vref_lb;
    uint8_t da_vref_mb;
    
    // enable
    set_ui32_reg(ANAC->MCU_RF, 0x000004f0);
    
    // da_gain, range: 0, 1
    for ( da_gain = 0; da_gain < 2; da_gain++ )
    {
        //  da_vref_lb, range: 0 to 7
        for ( da_vref_lb = 0; da_vref_lb < 8; da_vref_lb ++ )
        {
            // da_vref_mb, range: 0 to 7
            for ( da_vref_mb = 0; da_vref_mb < 8; da_vref_mb++ )
            {
                tpcode[index] = gauss_scale_generate ( da_gain, da_vref_lb, da_vref_mb );
                
                index++;
            }
        }
    }
    
    // disable
    ANAC->TP_CTL = ANAC->TP_CTL & 0xfffffE7f;
    set_ui32_reg(ANAC->MCU_RF, 0x00000100);
    set_ui32_reg(ANAC->AGC_CTL, 0x00000fff);
    
    printf("\n\n\n");
    for ( uint8_t i = 0; i < index; i++ )
    {
        printf ( "%2d\t", tpcode[i] );
        
        if ( (i & 0x7) == 0x7 )
        {
            printf("\n");
        }
    }
    printf("\n\n\n");
}
#endif // DBG_RF_CALIBRATION_TPCODE_TABLE_GENERATE_ENABLE

void rf_calibration_init ( void )
{
    m_rf_calib_times = 0;
    
    // enable
    set_ui32_reg(ANAC->MCU_RF, 0x000004f0);
    
    //judge (1,0,0)
    uint8_t result = gauss_scale_generate ( 1, 0, 0 );
    
    if ( result < RF_CALIB_VALUE_VALID_LOW )
    {
        m_rf_cal_da_gain = 1;
        
        result = rf_calibration_tpcode_generate ( RF_CALIB_VALUE_MIN, RF_CALIB_VALUE_MAX, RF_CALIB_VALUE_VALID_LOW, RF_CALIB_VALUE_VALID_HIGH );
    }
    else if ( result > RF_CALIB_VALUE_VALID_HIGH )
    {
        m_rf_cal_da_gain = 0;
        
        result = rf_calibration_tpcode_generate ( RF_CALIB_VALUE_MIN, RF_CALIB_VALUE_MAX, RF_CALIB_VALUE_VALID_LOW, RF_CALIB_VALUE_VALID_HIGH );
    }
    // else
	// result is valid, do nothing.
    
    if ( 0xFF != result )
    {
        // update result
        ANAC->TP_CTL = (ANAC->TP_CTL & 0xfffffff0) | result;
        printf ( "rf_calibration success\n" );
    }
    else
    {
        ANAC->TP_CTL = (ANAC->TP_CTL & 0xfffffff0) | 1;
        printf ( "rf_calibration failed\n" );
    }
    
    // disable
    ANAC->TP_CTL = ANAC->TP_CTL & 0xfffffE7f;
    set_ui32_reg(ANAC->MCU_RF, 0x00000100);
    set_ui32_reg(ANAC->AGC_CTL, 0x00000fff);
}

void rf_calibration_detect ( void )
{
    uint8_t result;
    
    uint8_t da_gain     = (ANAC->TX_CTL >> 14) & 0x1;
    uint8_t da_vref_lb  = (ANAC->TX_CTL >> 15) & 0x7;
    uint8_t da_vref_mb  = (ANAC->TX_CTL >> 18) & 0x7;
    
    uint8_t index       = da_gain*8 + da_vref_lb + da_vref_mb;
    
    m_rf_calib_times = 0;
    
    // enable
    set_ui32_reg(ANAC->MCU_RF, 0x000004f0);
    
    result = gauss_scale_generate ( da_gain, da_vref_lb, da_vref_mb ); 
    
    if ( result < RF_CALIB_VALUE_VALID_MIN )
    {
        result = rf_calibration_tpcode_generate ( index, RF_CALIB_TBL_INDX_MAX, RF_CALIB_VALUE_VALID_MIN, RF_CALIB_VALUE_VALID_MAX );
    }
    else if ( result > RF_CALIB_VALUE_VALID_MAX )
    {
        result = rf_calibration_tpcode_generate ( RF_CALIB_TBL_INDX_MIN, index, RF_CALIB_VALUE_VALID_MIN, RF_CALIB_VALUE_VALID_MAX );
    }
    
    if ( 0xFF != result )
    {
        // update result
        ANAC->TP_CTL = (ANAC->TP_CTL & 0xfffffff0) | result;
        printf ( "rf_calibration success\n" );
    }
    else
    {
        printf ( "rf_calibration failed\n" );
    }
    
    // disable
    ANAC->TP_CTL = ANAC->TP_CTL & 0xfffffE7f;
    set_ui32_reg(ANAC->MCU_RF, 0x00000100);
    set_ui32_reg(ANAC->AGC_CTL, 0x00000fff);
}

static uint8_t rf_rssi_convert (uint8_t rssi_reg)
{
#if PN102B
    return rssi_reg>>1;
#else
    return rssi_reg>>2;
#endif
}

uint32_t rf_reg_rd (uint16_t address)
{
    return 0;
}

void rf_reg_wr (uint16_t address, uint32_t data)
{

}

void rf_reset(void)
{

}

 void rf_force_agc_enable(bool en)
{
 
}

uint8_t rf_txpwr_dbm_get(uint8_t txpwr_idx, uint8_t modulation)
{
    return 0;
}

static void rf_sleep(void)
{
    ble_deepslcntl_set( ble_deepslcntl_get()
                        | BLE_DEEP_SLEEP_ON_BIT     // RW BLE Core sleep
                        | BLE_RADIO_SLEEP_EN_BIT    // Radio sleep  reset
                        | BLE_OSC_SLEEP_EN_BIT);    // Oscillator sleep
   // ble_deepslcntl_set(ble_deepslcntl_get() | BLE_DEEP_SLEEP_ON_BIT );
}

static void RADIOCNTL_Handler(void)
{

}

void PN102B_BLE_Init_New()
{
    set_ui32_reg(ANAC->MCU_RF, 0x00000100);             //0x24        0x00000400  two point calibration modes  0x00000100
    set_ui32_reg(ANAC->FSM_RF, 0x011f0164);             //0x28      0x011f0164
    set_ui32_reg(ANAC->PLL_CTL,0x00011c00);             //0x2c    df_sel  0x00001c00  fastlock en
    set_ui32_reg(ANAC->SD_CTL, 0x00000020);             //0x30
    set_ui32_reg(ANAC->GS_CTL, 0x08001f1f);             //0x34   0x08001f1f
    set_ui32_reg(ANAC->TP_CTL, 0x00000008);             //0x38
    set_ui32_reg(ANAC->TP_STS, 0x00000000);             //0x3c
    set_ui32_reg(ANAC->VCO_CTL,0x000010bf);         //0x40    vco_cal
    set_ui32_reg(ANAC->RX_CTL, 0x4000f7df);         //0x44   0x0000f7df
    set_ui32_reg(ANAC->TX_CTL, 0x0052380f);         //0x48   0x0052380f
    set_ui32_reg(ANAC->RF_PLL, 0x006ad7d0);         //0x4c  RF_PLL1    0x006fd7cc 
    set_ui32_reg(ANAC->RF_PLL2, 0x00003800);        //0x50  RF_PLL2    0x00083900 en_vga_out  0x00003800
    set_ui32_reg(ANAC->LDO_CTL, 0x4eba0208);            //0x54
    set_ui32_reg(ANAC->RCC_CTL, 0x0684251c);            //0x58    0x0684251c  PD RCCAL  
    set_ui32_reg(ANAC->FAR_CTL, 0x0001faaa);        //0x5C
    set_ui32_reg(ANAC->LDO2_CTL, 0x00124924);       //0x60
    set_ui32_reg(ANAC->PARAMP_CTL, 0x00000000);     //0x64    0x0022190a
    set_ui32_reg(ANAC->ANAC_3VCTL, 0x00035bfc);   //0x68
    set_ui32_reg(ANAC->AGC_CTL,0xefff);
}

void PN102_FPGA_BLE_INIT_NEW2()
{
    // For White List
    uint32_t mcu_mode;
    uint32_t agc_manu_en;
    uint32_t agc_sel;
    uint32_t agc_manu;
    uint32_t agc_gain_manu;

    uint32_t agc_th;
    uint32_t agc_set;
    uint32_t agc_gain;
    uint32_t agc_val;

    uint32_t gain_vld_time;
    uint32_t gain_step;
    uint32_t target_pdb;
    uint32_t pe_0db;
    uint32_t stable_min_pdb;
    uint32_t stable_max_pdb;
    uint32_t spi_agc_en;
    uint32_t spi_unused_bit;

    uint32_t sel_avg;
#if 1
    // convert to ram version
      read_ui32_reg(ANAC->MCU_RF, mcu_mode);
//    printf("read mcu mode :0x%08x\r\n",mcu_mode);
    mcu_mode = mcu_mode & (~(1 << 11));
    //printf("write mcu mode :0x%08x\r\n",mcu_mode);
   //set_ui32_reg(ANAC->MCU_RF, 0x000000fa);            //0x24
     set_ui32_reg(ANAC->MCU_RF, 0x0000000F|(1<<8));         //0x24
//   printf("ANAC->MCU_RF=0x%08x\r\n",ANAC->MCU_RF);
   set_ui32_reg(ANAC->FSM_RF, 0x011f0164);          //0x28

    // agc
    agc_manu_en = 1;
    agc_sel = 1;
    agc_manu = 0x5f9;
    agc_gain_manu = 74;
    agc_val = agc_gain_manu<<16 | agc_manu<<5 | agc_sel<<1 | agc_manu_en;
  set_ui32_reg(ANAC->AGC00, agc_val);//0x70

    agc_th = 13;
    agc_set = 0x00;//0b00_0_0000_00_00;
    agc_gain = 14;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC01, agc_val);//0x74

    agc_th = 17;
    agc_set = 0x04;//0b00_0_0000_01_00;
    agc_gain = 20;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC02, agc_val);//0x78

    agc_th = 23;
    agc_set = 0x200;//0b01_0_0000_00_00;
    agc_gain = 26;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC03, agc_val);//0x7c

    agc_th = 29;
    agc_set = 0x204;//0b01_0_0000_01_00;
    agc_gain = 32;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC04, agc_val);//0x7c

    agc_th = 35;
    agc_set = 0x600;//0b11_0_0000_00_00;
    agc_gain = 38;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC05, agc_val);//0x7c

    agc_th = 41;
    agc_set = 0x700;//0b11_1_0000_00_00;
    agc_gain = 44;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC06, agc_val);//0x7c

    agc_th = 47;
    agc_set = 0x704;//0b11_1_0000_01_00;
    agc_gain = 50;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC07, agc_val);//0x7c

    agc_th = 53;
    agc_set = 0x70c;//0b11_1_0000_11_00;
    agc_gain = 56;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC08, agc_val);//0x7c

    agc_th = 59;
    agc_set = 0x71c;//0b11_1_0001_11_00;
    agc_gain = 62;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC09, agc_val);//0x7c

    agc_th = 65;
    agc_set = 0x73c;//0b11_1_0011_11_00;
    agc_gain = 68;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC10, agc_val);//0x7c

    agc_th = 71;
    agc_set = 0x77c;//0b11_1_0111_11_00;
    agc_gain = 74;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC11, agc_val);//0x7c

    agc_th = 77;
    agc_set = 0x7fc;//0b11_1_f111_11_00;
    agc_gain = 80;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC12, agc_val);//0x7c

    //agc_th = 83;
    agc_th = 77;
    agc_set = 0x7fd;//0b11_1_f111_11_01;
    //agc_gain = 86;
    agc_gain = 80;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC13, agc_val);//0x7c

    //agc_th = 89;
    agc_th = 77;
    agc_set = 0x7fe;//0b11_1_f111_11_10;
    //agc_gain = 92;
    agc_gain = 80;
    agc_val = agc_gain<<24 | agc_set<<8 | agc_th;
    set_ui32_reg(ANAC->AGC14, agc_val);//0x7c

    gain_vld_time = 165;
    gain_vld_time = 180;
    gain_vld_time = 58;
    gain_step = 12;
    target_pdb =58;
    agc_val = target_pdb<<16 | gain_step<<8 | gain_vld_time;
    set_ui32_reg(ANAC->AGC15, agc_val);//0x7c

    pe_0db = 63;
    stable_min_pdb = 55;
    stable_max_pdb = 62;
    agc_val = stable_max_pdb<<16 | stable_min_pdb<<8 | pe_0db;
    set_ui32_reg(ANAC->AGC16, agc_val);//0x7c

    sel_avg = 1;
    set_ui32_reg(ANAC->AGC17, sel_avg);//0x7c
    
    set_ui32_reg(ANAC->AGC18,1);


    spi_agc_en = 0;
    spi_unused_bit = 0x7b;
    agc_val = spi_unused_bit<<8 |  spi_agc_en;
    set_ui32_reg(ANAC->AGC31, agc_val);//0x7c

#endif
    //set_ui32_reg(ANAC->AGC19,0);
    
#if 1
    set_ui32_reg(ANAC->MCU_RF, 0x00000100);             //0x24        0x00000400  two point calibration modes
    set_ui32_reg(ANAC->FSM_RF, 0x011f0164);             //0x28
    set_ui32_reg(ANAC->PLL_CTL,0x00011c00);             //0x2c    df_sel  0x00001c00  fastlock en
    set_ui32_reg(ANAC->SD_CTL, 0x00100020);             //0x30
    set_ui32_reg(ANAC->GS_CTL, 0x08001f1f);             //0x34   0x08001f1f
    set_ui32_reg(ANAC->TP_CTL, 0x00000008);             //0x38
    set_ui32_reg(ANAC->TP_STS, 0x00000000);             //0x3c
    set_ui32_reg(ANAC->VCO_CTL,0x000010bf);         //0x40    vco_cal
    set_ui32_reg(ANAC->RX_CTL, 0x0000f7df);         //0x44   0x0000f7df
    set_ui32_reg(ANAC->TX_CTL, 0x005fb80f);
    set_ui32_reg(ANAC->RF_PLL, 0x006ad7d0);         //0x4c  RF_PLL1    0x006fd7cc 
    set_ui32_reg(ANAC->RF_PLL2, 0x00003800);        //0x50  RF_PLL2    0x0008b900 en_vga_out
    set_ui32_reg(ANAC->RCC_CTL, 0x06842120);            //0x58      
    set_ui32_reg(ANAC->FAR_CTL, 0x0001faaa);        //0x5C      
#else
    set_ui32_reg(ANAC->MCU_RF, 0x00000400);             //0x24  0x000004f0
    set_ui32_reg(ANAC->FSM_RF, 0x011f0150);             //0x28  0x011f0164
    set_ui32_reg(ANAC->PLL_CTL, 0x00001c00);            //0x2c  0x00001c00  //0x00002a00
    set_ui32_reg(ANAC->SD_CTL, 0x00100020);             //0x30  0x00178020
    set_ui32_reg(ANAC->GS_CTL, 0x08001f1f);             //0x34  0x08001f1f   //0x08001fdf  GS_CTRL=11
    set_ui32_reg(ANAC->TP_CTL, 0x0500000f);             //0x38  0x05000008
    set_ui32_reg(ANAC->TP_STS, 0x00000000);             //0x3c  0x00000000 
    set_ui32_reg(ANAC->VCO_CTL,0x0003f087);         //0x40  0x0003f087      
    set_ui32_reg(ANAC->RX_CTL, 0x0000f7df);         //0x44  0x0000f7df
    set_ui32_reg(ANAC->TX_CTL, 0x0058740f );        //0x48  0x005ab40f    0x0058740f  //0x005fb80f
    set_ui32_reg(ANAC->RF_PLL, 0x006ad7d0);         //0x4c  0x006ad7d0
    set_ui32_reg(ANAC->RF_PLL2, 0x00222900);        //0x50  0x00003800   VGA OUT-> 0x00083900   0x0028a900  //0x00206900

    //set_ui32_reg(ANAC->LDO_CTL, 0x4eba0268);          //0x54  0x6eba0208
    set_ui32_reg(ANAC->LDO_CTL, 0x0eba0208);
    ANAC->ANAC_3VCTL    |=  1 ;

    set_ui32_reg(ANAC->RCC_CTL, 0x06842120);          //0x58    0x06842120
    TIMER_Delay(TIMER2,1000);
    set_ui32_reg(ANAC->RCC_CTL, 0x068421a0);      //RCCAL RESET
    TIMER_Delay(TIMER2,1000);
    set_ui32_reg(ANAC->RCC_CTL, 0x068421e0);      //RCCAL START      //0x068c21e0  12MHz;  0x069421e0  24MHz
    set_ui32_reg(ANAC->FAR_CTL, 0x0001ffff);        //0x5C  0x0001ffff

    //set_ui32_reg(ANAC->LDO2_CTL, 0x9c124924);     //0x60  0x00124924
    set_ui32_reg(ANAC->LDO2_CTL, 0x00124924);
    ANAC->ANAC_3VCTL    |=  1 ;

    set_ui32_reg(ANAC->PARAMP_CTL, 0x00000000);       //0x64  0x00000000
    set_ui32_reg(ANAC->ANAC_3VCTL, 0x00035bfc);       //0x68  0x00035bfc    
    ANAC->ANAC_3VCTL    |=  1 ;
#endif  
            
    if(default_use_ext_32k)
    {
        set_ui32_reg(ANAC->LDO_CTL, 0x4eba0208);            //0x54
        set_ui32_reg(ANAC->LDO2_CTL, 0x00124924);       //0x60
    }
    else
    {
        ANAC->LDO_CTL =  ((coarse_code << ANAC_LDO_CTL_CAPTRIMRCO_Pos) + 0x0eba0208);
        ANAC->LDO2_CTL = ((fine_code << ANAC_LDO_CTL2_LDOSEL_Pos) + 0x00124924);
    }

    if( 0 == ble_deep_sleep_stat_getf() )
    {
        if(default_use_ext_32k)
        {
            ANAC->RCC_CTL |= (1 <<11);
            ANAC->ANAC_3VCTL |= 0x01;
            while(ANAC->ANAC_3VCTL & 0x01);
            
            set_ui32_reg(ANAC->ANAC_3VCTL, 0x0003bbfc);   //0x68
            ANAC->ANAC_3VCTL |= 0x01;
            while(ANAC->ANAC_3VCTL & 0x01);
            
            while(!(CLK->STATUS & (1<<4)));
            ANAC->LDO_CTL &= (~(1 << 20));   //wait for xtal stable and shutoff rco;
        }
        else
        {
            set_ui32_reg(ANAC->ANAC_3VCTL, 0x00035bfc);   //0x68
            ANAC->LDO_CTL &= ~(1<<23);
        }
        //Wait configure valid
        ANAC->ANAC_3VCTL |= 0x01;
        while(ANAC->ANAC_3VCTL & 0x01);
    }
}


#if PN102B
void en_bb_debug(void)
{
    printf("en_bb_dbg\n");

    //bit7   ble_rx_irq
    GPIO_SetMode(P0,BIT5, GPIO_MODE_OUTPUT);    
    SYS->P0_MFP &= ~(SYS_MFP_ALT_Msk(5));
    SYS->P0_MFP |= SYS_MFP_MFP_Msk(5);
    
    //bit6   event_in_process
    GPIO_SetMode(P0,BIT3, GPIO_MODE_OUTPUT);    
    SYS->P0_MFP |= SYS_MFP_ALT_Msk(3);
    SYS->P0_MFP |= SYS_MFP_MFP_Msk(3);
    
    //bit5   sync_found_pulse
    GPIO_SetMode(P0,BIT2, GPIO_MODE_OUTPUT);    
    SYS->P0_MFP &= ~(SYS_MFP_ALT_Msk(2));
    SYS->P0_MFP |= SYS_MFP_MFP_Msk(2);
    
    //bit4   sync_window
    GPIO_SetMode(P1,BIT7, GPIO_MODE_OUTPUT);    
    SYS->P1_MFP |= SYS_MFP_ALT_Msk(7);
    SYS->P1_MFP &= ~(SYS_MFP_MFP_Msk(7));
    
    //bit3   rf_rx_data
    GPIO_SetMode(P1,BIT6,GPIO_MODE_OUTPUT); 
    SYS->P1_MFP |= SYS_MFP_ALT_Msk(6);
    SYS->P1_MFP &= ~(SYS_MFP_MFP_Msk(6));
    
    //bit2   radcntl_rxen
    GPIO_SetMode(P1,BIT1,GPIO_MODE_OUTPUT); 
    SYS->P1_MFP |= SYS_MFP_ALT_Msk(1);
    SYS->P1_MFP &= ~(SYS_MFP_MFP_Msk(1));
    
    //bit1  rf_tx_data
    GPIO_SetMode(P2,BIT5, GPIO_MODE_OUTPUT);    
    SYS->P2_MFP |= SYS_MFP_ALT_Msk(5);
    SYS->P2_MFP |= SYS_MFP_MFP_Msk(5);

    //bit0  radcntl_txend
    GPIO_SetMode(P2,BIT4, GPIO_MODE_OUTPUT);    
    SYS->P2_MFP |= SYS_MFP_ALT_Msk(4);
    SYS->P2_MFP |= SYS_MFP_MFP_Msk(4);

}
#endif

static void rf_mdm_init(void)
{
#if PN102B
    mdm_gfo_gfskdetect_set(0x00000500ul);
    //mdm_pe_powerthr_set(0x30);
#endif

    mdm_mdm_cntl_set(2);
    mdm_rx_startupdel_set(0x0001000aul);    //0xC
    mdm_tx_startupdel_set(0x00010006ul);    //0x10
}

void rf_init(struct rwip_rf_api *api)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_BLE_FREQ_TABLE_LEN];   //rf table size  40

    api->reg_rd = rf_reg_rd;
    api->reg_wr = rf_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->sleep = rf_sleep;
    api->reset = rf_reset;
    api->isr = RADIOCNTL_Handler;
    api->force_agc_enable = rf_force_agc_enable;
    api->rssi_convert = rf_rssi_convert;

    //Initialize the RF driver API structure
    //PN102_BLE_Init();
    //PN102B_BLE_Init_New();
    PN102_FPGA_BLE_INIT_NEW2();
    
#if defined(CFG_FPGA)
    // tx enable
    PN102_SPIWrite(0x11,0xf5);
    
    // channel select
    PN102_SPIWrite(0x31,(2402-2336));
#endif
    
    //RADIOPWRUP
#if (PN102B)
    api->txpwr_max = 0xf;       //0x0f

    ble_rtrip_delay_setf(0x0A); //6bit[0, 0x05      0x0A ok
    ble_rxpwrup_setf(0x74);     //8bit, affect  rx, recv nothing   >0x74
    ble_txpwrdn_setf(0x05);     //0x05
    ble_txpwrup_setf(0x7f);     //8bit, must : >0x64 <=0x82 for decryption need time.  149 + rtip_dly - txpwrup must >31 us; if <0x64, no pkt recv
                                // range (0x65,0x82)
#else
    api->txpwr_max = 0x0;       //0x0f

    ble_rtrip_delay_setf(0x0c); //6bit 0x05     0x0f
    ble_rxpwrup_setf(0x74);     //8bit, affect  rx, recv nothing   >0x74
    ble_txpwrdn_setf(0x05);     //0x05
    ble_txpwrup_setf(0x74);     //8bit, must : >0x64 <=0x82 for decryption need time.  149 + rtip_dly - txpwrup must >31 us; if <0x64, no pkt recv
                                // range (0x65,0x82)
#endif

//  ble_twext_setf(8);
//  ble_twosc_setf(40);
//  ble_twrm_setf(32);

    // fqt
    for( idx=0; idx < EM_BLE_FREQ_TABLE_LEN; idx++)
    {
        temp_freq_tbl[idx] = 0;
        switch (idx)
        {
            case 0:
                temp_freq_tbl[idx] = 37;        //0x25
                break;
            case 12:
                temp_freq_tbl[idx] = 38;        //0x26
                break;
            case 39:
                temp_freq_tbl[idx] = 39;        //0x27
                break;
            default:
                if(idx < 12)
                    temp_freq_tbl[idx] = idx - 1;
                else
                    temp_freq_tbl[idx] = idx - 2;
                break;
        }
        //printf("idx:%d,freq:%d\r\n",idx, temp_freq_tbl[idx]);
    }

    em_ble_burst_wr(&temp_freq_tbl[0], EM_BLE_FT_OFFSET, EM_BLE_FREQ_TABLE_LEN);
    
    ble_radiocntl0_set(0x0);
    ble_xrfsel_setf(0x03);

    rf_mdm_init();

    //printf("rf init\r\n");
}

void Internal_32k_init(void)
{
    if ( default_en_calib_32k )
    {
        Set_Calib_Intval(5000); 
    }
    
    if ( ( !default_use_ext_32k ) && default_sleep_en )
    {
        OPT_CALIB_32K_CLK();
    }
}


uint32_t u32pagc[6];
uint32_t anac_tx_ctl = 0;
#define _PAGC_0_  0x7f806020
#define _PAGC_BIT0_  0x7f886020
#define _PAGC_BIT1_  0x7f846020
#define _PAGC_BIT2_  0x7f826020
#define _PAGC_BIT3_  0x7f816020
#define _PAGC_BIT4_  0x7f80E020
#define TEST_PING_HIGH   P03 =1
#define TEST_PING_LOW    P03 =0
void rf_pa_ramp_init(void)
{
//    set_ui32_reg(BLEBB->DIAGCNTL, 0x0000009f);
//    GPIO_SetMode(P0,BIT4,GPIO_MODE_INPUT);
//    SYS->TESTCTL|=(0x04);
//    SYS->P0_MFP &= ~SYS_MFP_P04_Msk;
//    SYS->P0_MFP |= SYS_MFP_P04_BLE_DBG00;

//    GPIO_EnableInt(P0, 4, GPIO_INT_BOTH_EDGE);
//    NVIC_EnableIRQ(GPIO01_IRQn);
    
    set_ui32_reg(BLEBB->DIAGCNTL, 0x0000008c);
    GPIO_SetMode(P5,BIT4,GPIO_MODE_INPUT);
    SYS->TESTCTL|=(0x04);
    SYS->P5_MFP &= ~SYS_MFP_P54_Msk;
    SYS->P5_MFP |= SYS_MFP_P54_BLE_DBG07;

    GPIO_EnableInt(P5, 4, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(GPIO5_IRQn);
    
//  SYS->P0_MFP &= ~SYS_MFP_P03_Msk;
//  GPIO_SetMode(P0,BIT3,GPIO_MODE_OUTPUT);
   
    rf_power_map(default_tx_power);
}
void rf_power_map(uint8_t xdbm)
{
    anac_tx_ctl =  ANAC->TX_CTL|(1<<13)|0x0f ;// en tx_pa  set_pabias
    u32pagc[0] = _PAGC_0_;
    //DBG_RF_UART_TASK (("\n power  %d dbm \n",xdbm ));
    switch (xdbm)
    {
        case 0:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
            u32pagc[4] = u32pagc[3];  
            u32pagc[5] = u32pagc[4];          
         break;
        
        case 1:
           
            u32pagc[1] = u32pagc[0];
            u32pagc[2] = u32pagc[1];  
            u32pagc[3] = u32pagc[2];
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;  
            u32pagc[5] = u32pagc[4];          
         break;
        
        case 2:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
            u32pagc[3] = u32pagc[2];
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;  
            u32pagc[5] = u32pagc[4];          
         break;
        
        case 3:
            
            u32pagc[1] = u32pagc[0];
            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
            u32pagc[3] = u32pagc[2];
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;
            u32pagc[5] = u32pagc[4];          
        
        
         break;
        
        case 4:
            
            u32pagc[1] = u32pagc[0];
            u32pagc[2] = u32pagc[1];  
            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;
            u32pagc[5] = u32pagc[4];          
        
        
         break;
        
         case 5:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1];  
            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;  
            u32pagc[5] =  u32pagc[4];
            break;
         
         case 6:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;  
            u32pagc[5] =  u32pagc[4];
            break;

         case 7:
           
            u32pagc[1] = u32pagc[0];
            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
            u32pagc[3] = u32pagc[2];
            u32pagc[4] = u32pagc[3];
            u32pagc[5] = u32pagc[4]|_PAGC_BIT4_;          
            break;
         
         case 8:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1];  
            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
            u32pagc[4] = u32pagc[3];
            u32pagc[5] = u32pagc[4]|_PAGC_BIT4_;          
            break;
         
         case 9:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1];  
            u32pagc[3] = u32pagc[2];
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;
            u32pagc[5] = u32pagc[4]|_PAGC_BIT4_;          
            break;
         
        case 0x0a:
           
            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
            u32pagc[4] = u32pagc[3]|_PAGC_BIT3_;
            u32pagc[5] = u32pagc[4]|_PAGC_BIT4_;          
            break;
       
//        case 0XFE://-2
//            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
//            u32pagc[2] = u32pagc[1];  
//            u32pagc[3] = u32pagc[2]|_PAGC_BIT2_;
//            u32pagc[4] = u32pagc[3];
//            u32pagc[5] = u32pagc[4];          
//            break  ;

//        
//        case 0XFB://-5
//            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
//            u32pagc[2] = u32pagc[1]|_PAGC_BIT1_;  
//            u32pagc[3] = u32pagc[2];
//            u32pagc[4] = u32pagc[3];
//            u32pagc[5] = u32pagc[4];          
//            break  ;  
//        
//        
//          case 0xF5: //-11
//           
//            u32pagc[1] = u32pagc[0]|_PAGC_BIT0_;
//            u32pagc[2] = u32pagc[1];  
//            u32pagc[3] = u32pagc[2];
//            u32pagc[4] = u32pagc[3];
//            u32pagc[5] = u32pagc[4];          
//            break;      
        
        
        default:
              //DBG_RF_UART_TASK ( ( "\n paramer error \n" ) );

            break;
    
    }
    
     //DBG_RF_UART_TASK ( ( "\n ANAC->SD_CTL=0x%8x \n",ANAC->SD_CTL ) );
     //  for (i=0;i<6;i++)
     //DBG_RF_UART_TASK ( ( "\n u32pagc[%d]=0x%8x \n",i,u32pagc[i] ) );
}
void  rf_set_pa_ramp(void )
{

}
  
void DEALY_NOP(void)
{
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();  
    __NOP();  
    __NOP();
    __NOP();
}

void GPIO5_IRQHandler()
{
    //TEST_PING_HIGH;
    P5->INTSRC = BIT4; 
    if(P54==1)
    {
        ANAC->SD_CTL = u32pagc[0];
        DEALY_NOP(); 
        ANAC->SD_CTL = u32pagc[1]; 
        DEALY_NOP();
        ANAC->SD_CTL = u32pagc[2];
        DEALY_NOP();
        ANAC->SD_CTL = u32pagc[3];
        DEALY_NOP();        
        ANAC->SD_CTL = u32pagc[4]; 
        DEALY_NOP();   
        ANAC->SD_CTL = u32pagc[5];
    }
	else
    {
        ANAC->SD_CTL &= (0x00170020);
        DEALY_NOP();

        ANAC->SD_CTL &=(0x00160020);
        DEALY_NOP();

        ANAC->SD_CTL &= (0x00140020);
        DEALY_NOP();

        ANAC->SD_CTL &=(0x00100020);
        DEALY_NOP();

        ANAC->SD_CTL &= (0x00000020);
    }
   
    //TEST_PING_LOW; 
}

/********************************* end of file ************************************/
