#include <edsocket.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>

edsocket::edsocket(int32_t socket_fd):
	edthreaded_fd()
{
	set_fd(socket_fd);
}
	
edsocket::~edsocket()
{}

int32_t edsocket::_raw_read(uint8_t * buffer, uint32_t max_size)
{
	int32_t cnt = recv(m_fd, buffer, max_size, MSG_DONTWAIT);
	if (cnt == 0)
	{
		int32_t err_no = errno;
		_setError(ConnectionClosed, err_no);
		stop();
	}
	return cnt;
}

int32_t edsocket::_raw_write(uint8_t * buffer, uint32_t max_size)
{
	return ::write(m_fd, buffer, max_size);
}
