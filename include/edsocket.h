#ifndef EDTHREADED_SOCKET_H
#define EDTHREADED_SOCKET_H

#include <edthreaded_fd.h>

class edsocket : public edthreaded_fd
{
  public:

	edsocket(uint socket_fd);
	
	~edsocket();

  private:

	int _raw_read(char * buffer, uint max_size);
	int _raw_write(char * buffer, uint max_size);

};

#endif
