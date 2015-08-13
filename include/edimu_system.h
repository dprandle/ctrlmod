#ifndef EDIMU_SYSTEM_H
#define EDIMU_SYSTEM_H

/*!
  Gyro Registers
*/
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38

/*!
  Accel/Magneto Registers
*/
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_XM			0x0F
#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_XM		0x1F
#define CTRL_REG1_XM		0x20
#define CTRL_REG2_XM		0x21
#define CTRL_REG3_XM		0x22
#define CTRL_REG4_XM		0x23
#define CTRL_REG5_XM		0x24
#define CTRL_REG6_XM		0x25
#define CTRL_REG7_XM		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

#include <edsystem.h>

class edi2c;

class edimu_system : public edsystem
{
  public:

	//! possible ranges of the gyroscope
	enum g_scale
	{
		G_SCALE_245DPS,			/*!< 245 degrees per second */
		G_SCALE_500DPS,			/*!< 500 dps */
		G_SCALE_2000DPS,		/*!< 2000 dps */
	};
	
	//! Possible FSR's of the accelerometer
	enum a_scale
	{
		A_SCALE_2G,				/*!< 2g */
		A_SCALE_4G,				/*!< 4g */
		A_SCALE_6G,				/*!< 6g */
		A_SCALE_8G,				/*!< 8g */
		A_SCALE_16G				/*!< 16g */
	};
	
	//! Possible FSR's of the magnetometer
	enum m_scale
	{
		M_SCALE_2GS,			/*!< 2Gs */
		M_SCALE_4GS,			/*!< 4Gs */
		M_SCALE_8GS,			/*!< 8Gs */
		M_SCALE_12GS,			/*!< 12Gs */
	};
	
	//! Possible data rate/bandwidth combos of the gyro
	enum g_odr
	{							// ODR (Hz) --- Cutoff
		G_ODR_95_BW_125  = 0x0,	/*!< 95         12.5 */
		G_ODR_95_BW_25   = 0x1,	/*!< 95          25 */
		G_ODR_190_BW_125 = 0x4,	/*!< 190        12.5 */
		G_ODR_190_BW_25  = 0x5,	/*!< 190         25 */
		G_ODR_190_BW_50  = 0x6,	/*!< 190         50 */
		G_ODR_190_BW_70  = 0x7,	/*!< 190         70 */
		G_ODR_380_BW_20  = 0x8,	/*!< 380         20 */
		G_ODR_380_BW_25  = 0x9,	/*!< 380         25 */
		G_ODR_380_BW_50  = 0xA,	/*!< 380         50 */
		G_ODR_380_BW_100 = 0xB,	/*!< 380         100 */
		G_ODR_760_BW_30  = 0xC,	/*!< 760         30 */
		G_ODR_760_BW_35  = 0xD,	/*!< 760         35 */
		G_ODR_760_BW_50  = 0xE,	/*!< 760         50 */
		G_ODR_760_BW_100 = 0xF,	/*!< 760         100 */
	};

    //! Possible output data rates of the accelerometer
	enum a_odr
	{
		A_POWER_DOWN,			/*!< Power-down mode (0x0) */
		A_ODR_3125,				/*!< 3.125 Hz	(0x1) */
		A_ODR_625,				/*!< 6.25 Hz (0x2) */
		A_ODR_125,				/*!< 12.5 Hz (0x3) */
		A_ODR_25,				/*!< 25 Hz (0x4) */
		A_ODR_50,				/*!< 50 Hz (0x5) */
		A_ODR_100,				/*!< 100 Hz (0x6) */
		A_ODR_200,				/*!< 200 Hz (0x7) */
		A_ODR_400,				/*!< 400 Hz (0x8) */
		A_ODR_800,				/*!< 800 Hz (9) */
		A_ODR_1600				/*!< 1600 Hz (0xA) */
	};

	//! Possible anti-aliasing filter rates of the accelerometer
	enum a_abw
	{
		A_ABW_773,				/*!< 773 Hz (0x0) */
		A_ABW_194,				/*!< 194 Hz (0x1) */
		A_ABW_362,				/*!< 362 Hz (0x2) */
		A_ABW_50,				/*!<  50 Hz (0x3) */
	};



	//! Possible output data rates of the magnetometer
	enum m_odr
	{
		M_ODR_3125,				/*!< 3.125 Hz (0x00) */
		M_ODR_625,				/*!< 6.25 Hz (0x01) */
		M_ODR_125,				/*!< 12.5 Hz (0x02) */
		M_ODR_25,				/*!< 25 Hz (0x03) */
		M_ODR_50,				/*!< 50 (0x04) */
		M_ODR_100,				/*!< 100 Hz (0x05) */
	};

	edimu_system();

	~edimu_system();
	
	a_abw accel_aa();
	
	a_scale accel_scale();

	a_odr accel_datarate();
	
	g_scale gyro_scale();

	g_odr gyro_datarate();
		
	m_scale mag_scale();

	m_odr mag_datarate();
	
    void init();

	bool process(edmessage * msg);

	void release();
	
	void set_accel_aa(a_abw antialiasing);
	
	void set_accel_scale(a_scale scale);

	void set_accel_datarate(a_odr datarate);

	void set_gyro_scale(g_scale scale);

	void set_gyro_datarate(g_odr datarate);
	
	void set_mag_scale(m_scale scale);

	void set_mag_datarate(m_odr datarate);

    void update();

    std::string typestr() {return TypeString();}

	static std::string TypeString() {return "edimu_system";}

  private:
	void _update_gres();
	void _update_mres();
	void _update_ares();
	
	edi2c * m_i2c;

	m_scale m_mag_scale;
	g_scale m_gyro_scale;
	a_scale m_accel_scale;

	float m_gres, m_mres, m_ares;
	
	m_odr m_mag_odr;
	g_odr m_gyro_odr;
	a_odr m_accel_odr;
	
	a_abw m_accel_abw;
};


#endif
