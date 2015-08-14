#include <edimu_system.h>
#include <edi2c.h>
#include <edutility.h>

edimu_system::edimu_system():
	m_i2c(NULL),
	m_temp(0),
	m_mag_scale(M_SCALE_2GS),
	m_gyro_scale(G_SCALE_245DPS),
	m_accel_scale(A_SCALE_2G),
	m_mag_odr(M_ODR_50),
	m_gyro_odr(G_ODR_95_BW_125),
	m_accel_odr(A_ODR_50),
	m_accel_abw(A_ABW_50)
{}

edimu_system::~edimu_system()
{}

void edimu_system::init()
{
	m_i2c = new edi2c(1);

	if (!m_i2c->start())
	{
		log_message("edimu_system::init Error initializing i2c");
		return;
	}

	m_i2c->set_target_address(LSM9DS0_G_ADDR);
	uint8_t gTest = m_i2c->readByte(WHO_AM_I_G);
	
	m_i2c->set_target_address(LSM9DS0_XM_ADDR);
	uint8_t xmTest = m_i2c->readByte(WHO_AM_I_XM);
	
	if (!_init_gyro())
		log_message("edimu_system::init Coult not initialize gyroscope");
	set_gyro_datarate(m_gyro_odr);
	set_gyro_scale(m_gyro_scale);
	
	if (!_init_accel())
		log_message("edimu_system::init Coult not initialize accelerometer");
	set_accel_datarate(m_accel_odr);
	set_accel_scale(m_accel_scale);
	
	if (!_init_mag())
		log_message("edimu_system::init Coult not initialize magnometer");
	set_mag_datarate(m_mag_odr);
	set_mag_scale(m_mag_scale);
	
	if((xmTest != WHO_AM_I_XM) || (gTest != WHO_AM_I_G))
		log_message("edimu_system::init IMU did not return correct who am i bytes");
}

void edimu_system::release()
{
	delete m_i2c;
}

edimu_system::a_abw edimu_system::accel_aa()
{
	return m_accel_abw;
}
	
edimu_system::a_scale edimu_system::accel_scale()
{
	return m_accel_scale;
}

edimu_system::a_odr edimu_system::accel_datarate()
{
	return m_accel_odr;
}

void edimu_system::calibrate()
{
	if (!m_i2c->running())
	{
		log_message("edimu_system::calibrate Can not run calibration because i2c thread not active");
		return;
	}
	
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	int32_t samples, ii;
  
	// Calibrate gyroscope
	m_i2c->set_target_address(LSM9DS0_G_ADDR);
	uint8_t c = m_i2c->readByte(CTRL_REG5_G);
	m_i2c->writeByte(CTRL_REG5_G, c | 0x40);
	delay(20);
	m_i2c->writeByte(FIFO_CTRL_REG_G, 0x20 | 0x1F);
	delay(1000);
  
	samples = (m_i2c->readByte(FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

	for(ii = 0; ii < samples ; ii++)
	{
		m_i2c->readBytes(OUT_X_L_G,  &data[0], 6);
		gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
		gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
		gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
	}  

	gyro_bias[0] /= samples; // average the data
	gyro_bias[1] /= samples; 
	gyro_bias[2] /= samples; 
  
	m_gbias[0] = (float)gyro_bias[0]*m_gres;  // Properly scale the data to get deg/s
	m_gbias[1] = (float)gyro_bias[1]*m_gres;
	m_gbias[2] = (float)gyro_bias[2]*m_gres;
  
	c = m_i2c->readByte(CTRL_REG5_G);
	m_i2c->writeByte(CTRL_REG5_G, c & ~0x40);
	delay(20);
	m_i2c->writeByte(FIFO_CTRL_REG_G, 0x00);
  

	// Calibrate accel
	m_i2c->set_target_address(LSM9DS0_XM_ADDR);
	c = m_i2c->readByte(CTRL_REG0_XM);
	m_i2c->writeByte(CTRL_REG0_XM, c | 0x40);
	delay(20);
	m_i2c->writeByte(FIFO_CTRL_REG, 0x20 | 0x1F);
	delay(1000);

	samples = (m_i2c->readByte(FIFO_SRC_REG) & 0x1F);

	for(ii = 0; ii < samples ; ii++)
	{
		m_i2c->readBytes(OUT_X_L_A, &data[0], 6);
		accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
		accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
		accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1./m_ares);
	}  

	accel_bias[0] /= samples;
	accel_bias[1] /= samples;
	accel_bias[2] /= samples; 
  
	m_abias[0] = (float)accel_bias[0]*m_ares;
	m_abias[1] = (float)accel_bias[1]*m_ares;
	m_abias[2] = (float)accel_bias[2]*m_ares;

	c = m_i2c->readByte(CTRL_REG0_XM);
	m_i2c->writeByte(CTRL_REG0_XM, c & ~0x40);
	delay(20);
	m_i2c->writeByte(FIFO_CTRL_REG, 0x00);
}
	
edimu_system::g_scale edimu_system::gyro_scale()
{
	return m_gyro_scale;
}

edimu_system::g_odr edimu_system::gyro_datarate()
{
	return m_gyro_odr;
}
		
edimu_system::m_scale edimu_system::mag_scale()
{
	return m_mag_scale;
}

edimu_system::m_odr edimu_system::mag_datarate()
{
	return m_mag_odr;
}

void edimu_system::set_accel_aa(a_abw antialiasing)
{
	m_accel_abw = antialiasing;

	if (m_i2c->running())
	{
		m_i2c->set_target_address(LSM9DS0_XM_ADDR);
		uint8_t temp = m_i2c->readByte(CTRL_REG2_XM);
		
		temp &= 0xFF^(0x3 << 6);
		temp |= (m_accel_abw << 6);

		m_i2c->writeByte(CTRL_REG2_XM, temp);
	}
}
	
void edimu_system::set_accel_scale(a_scale scale)
{
	m_accel_scale = scale;

	m_i2c->set_target_address(LSM9DS0_XM_ADDR);
	uint8_t temp = m_i2c->readByte(CTRL_REG2_XM);

	temp &= 0xFF^(0x3 << 3);
	temp |= m_accel_scale << 3;

	m_i2c->writeByte(CTRL_REG2_XM, temp);
	_update_ares();
}

void edimu_system::set_accel_datarate(a_odr datarate)
{
	m_accel_odr = datarate;

	if (m_i2c->running())
	{		
		m_i2c->set_target_address(LSM9DS0_XM_ADDR);
		uint8_t temp = m_i2c->readByte(CTRL_REG1_XM);

		temp &= 0xFF^(0xF << 4);
		temp |= (m_accel_odr << 4);

		m_i2c->writeByte(CTRL_REG1_XM, temp);
	}
}

void edimu_system::set_gyro_scale(g_scale scale)
{
	m_gyro_scale = scale;

	if (m_i2c->running())
	{		
		m_i2c->set_target_address(LSM9DS0_G_ADDR);
		uint8_t temp = m_i2c->readByte(CTRL_REG4_G);

		temp &= 0xFF^(0x3 << 4);
		temp |= m_gyro_scale << 4;

		m_i2c->writeByte(CTRL_REG4_G, temp);
		_update_gres();
	}
}

void edimu_system::set_gyro_datarate(g_odr datarate)
{
	m_gyro_odr = datarate;

	if (m_i2c->running())
	{		
		m_i2c->set_target_address(LSM9DS0_G_ADDR);
		uint8_t temp = m_i2c->readByte(CTRL_REG1_G);
	
		temp &= 0xFF^(0xF << 4);
		temp |= (m_gyro_odr << 4);

		m_i2c->writeByte(CTRL_REG1_G, temp);
	}
}
	
void edimu_system::set_mag_scale(m_scale scale)
{
	m_mag_scale = scale;

	if (m_i2c->running())
	{		
		m_i2c->set_target_address(LSM9DS0_XM_ADDR);
		uint8_t temp = m_i2c->readByte(CTRL_REG6_XM);
	
		temp &= 0xFF^(0x3 << 5);
		temp |= m_mag_scale << 5;

		m_i2c->writeByte(CTRL_REG6_XM, temp);
		_update_mres();
	}
}

void edimu_system::set_mag_datarate(m_odr datarate)
{
	m_mag_odr = datarate;

	if (m_i2c->running())
	{
		m_i2c->set_target_address(LSM9DS0_XM_ADDR);
		uint8_t temp = m_i2c->readByte(CTRL_REG5_XM);
	
		temp &= 0xFF^(0x7 << 2);
		temp |= (m_mag_odr << 2);

		m_i2c->writeByte(CTRL_REG5_XM, temp);
	}
}


bool edimu_system::process(edmessage * msg)
{
	return true;
}

void edimu_system::update()
{
	static bool waiting_for_reply = false;
	static uint8_t buf[6];
	static int32_t cnt = 0;
	
	if (!waiting_for_reply)
	{
		m_i2c->commandRead(OUT_X_L_G, 6);
		waiting_for_reply = true;
	}

	if (cnt != 6)
	{
		cnt += m_i2c->read(buf+cnt, 6-cnt);
		if (cnt == 6)
		{
			waiting_for_reply = false;
			m_gyro_raw.x = (buf[1] << 8) | buf[0];
			m_gyro_raw.y = (buf[3] << 8) | buf[2];
			m_gyro_raw.z = (buf[5] << 8) | buf[4];
		}
	}
}

void edimu_system::_update_gres()
{
	switch (m_gyro_scale)
	{
	  case (G_SCALE_245DPS):
		m_gres = 245.0 / 32768.0;
		break;
	  case (G_SCALE_500DPS):
		m_gres = 500.0 / 32768.0;
		break;
	  case (G_SCALE_2000DPS):
		m_gres = 2000.0 / 32768.0;
		break;
	}	
}

void edimu_system::_update_mres()
{
	m_mres = m_mag_scale == M_SCALE_2GS ? 2.0 / 32768.0 : 
	       (float) (m_mag_scale << 2) / 32768.0;	
}

void edimu_system::_update_ares()
{
	m_ares = m_accel_scale == A_SCALE_16G ? 16.0 / 32768.0 : 
		   (((float) m_accel_scale + 1.0) * 2.0) / 32768.0;
}

bool edimu_system::_init_mag()
{
	m_i2c->set_target_address(LSM9DS0_XM_ADDR);
	return (m_i2c->writeByte(CTRL_REG5_XM, 0x94) &&
			m_i2c->writeByte(CTRL_REG6_XM, 0x00) &&
			m_i2c->writeByte(CTRL_REG7_XM, 0x00) &&
			m_i2c->writeByte(CTRL_REG4_XM, 0x04) &&
			m_i2c->writeByte(INT_CTRL_REG_M, 0x09));
}

bool edimu_system::_init_accel()
{
	m_i2c->set_target_address(LSM9DS0_XM_ADDR);
	return ( m_i2c->writeByte(CTRL_REG0_XM, 0x00) &&
			 m_i2c->writeByte(CTRL_REG1_XM, 0x57) &&
			 m_i2c->writeByte(CTRL_REG2_XM, 0x00) &&
			 m_i2c->writeByte(CTRL_REG3_XM, 0x04));		
}

bool edimu_system::_init_gyro()
{
	m_i2c->set_target_address(LSM9DS0_G_ADDR);
	return (m_i2c->writeByte(CTRL_REG1_G, 0x0F) &&
			m_i2c->writeByte(CTRL_REG2_G, 0x00) &&
			m_i2c->writeByte(CTRL_REG3_G, 0x88) && 
			m_i2c->writeByte(CTRL_REG4_G, 0x00) &&
			m_i2c->writeByte(CTRL_REG5_G, 0x00));
}

void edimu_system::_createMessage()
{
	log_message("Received gyro reading: " + _calc_gyro().toString());
}


fvec3 edimu_system::_calc_gyro()
{
	return fvec3(m_gyro_raw.x, m_gyro_raw.y, m_gyro_raw.z) * m_gres;
}

fvec3 edimu_system::_calc_mag()
{
	return fvec3(m_magno_raw.x, m_magno_raw.y, m_magno_raw.z) * m_mres;
}

fvec3 edimu_system::_calc_accel()
{
	return fvec3(m_accel_raw.x, m_accel_raw.y, m_accel_raw.z) * m_ares;
}
