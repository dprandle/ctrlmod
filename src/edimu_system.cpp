#include <edimu_system.h>
#include <edi2c.h>

edimu_system::edimu_system():
	m_i2c(NULL)
{}

edimu_system::~edimu_system()
{}

void edimu_system::init()
{
	m_i2c = new edi2c(1);
	m_i2c->set_target_address(0x56);
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
}
	
void edimu_system::set_accel_scale(a_scale scale)
{
	m_accel_scale = scale;
	char temp = m_i2c->readByte(CTRL_REG2_XM);

	temp &= 0xFF^(0x3 << 3);
	temp |= m_accel_scale << 3;
	m_i2c->writeByte(CTRL_REG2_XM, temp);
	_update_ares();
}

void edimu_system::set_accel_datarate(a_odr datarate)
{
	m_accel_odr = datarate;
}

void edimu_system::set_gyro_scale(g_scale scale)
{
	m_gyro_scale = scale;
	char temp = m_i2c->readByte(CTRL_REG4_G);

	temp &= 0xFF^(0x3 << 4);
	temp |= m_gyro_scale << 4;
	m_i2c->writeByte(CTRL_REG4_G, temp);
	_update_gres();
}

void edimu_system::set_gyro_datarate(g_odr datarate)
{
	m_gyro_odr = datarate;
}
	
void edimu_system::set_mag_scale(m_scale scale)
{
	m_mag_scale = scale;
	uint8_t temp = m_i2c->readByte(CTRL_REG6_XM);
	
	temp &= 0xFF^(0x3 << 5);
	temp |= m_mag_scale << 5;
	m_i2c->writeByte(CTRL_REG6_XM, temp);
	_update_mres();
}

void edimu_system::set_mag_datarate(m_odr datarate)
{
	m_mag_odr = datarate;
}

bool edimu_system::process(edmessage * msg)
{
	return true;
}

void edimu_system::update()
{
		
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
