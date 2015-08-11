#include <edsocket.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>

edsocket::edsocket(uint socket_fd):
	edthreaded_fd()
{
	set_fd(socket_fd);
}
	
edsocket::~edsocket()
{}

int edsocket::_raw_read(char * buffer, uint max_size)
{
	int cnt = recv(m_fd, buffer, max_size, MSG_DONTWAIT);
	if (cnt == 0)
	{
		int err_no = errno;
		_setError(ConnectionClosed, err_no);
		stop();
	}
	return cnt;
}

int edsocket::_raw_write(char * buffer, uint max_size)
{
	return ::write(m_fd, buffer, max_size);
}
