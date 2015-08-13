#ifndef EDTHREADED_SOCKET_H
#define EDTHREADED_SOCKET_H

#include <edthreaded_fd.h>

class edsocket : public edthreaded_fd
{
  public:

	edsocket(uint32_t socket_fd);
	
	~edsocket();

  private:

	int32_t _raw_read(uint8_t * buffer, uint32_t max_size);
	int32_t _raw_write(uint8_t * buffer, uint32_t max_size);

};

#endif
