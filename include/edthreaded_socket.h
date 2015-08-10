#ifndef EDTHREADED_SOCKET_H
#define EDTHREADED_SOCKET_H

#include <edglobal.h>
#include <pthread.h>


#define SCKT_SEND_BUF_SIZE 57600 // Enough room for 10 scans
#define SCKT_RECV_BUF_SIZE 1024 // Plenty for any incoming commands

#define SCKT_TMP_BUF_SIZE 5760

class edthreaded_socket
{
  public:

	enum ErrorVal
	{
		NoError,
        ConnectionClosed,
		DataOverwrite,
		InvalidRead,
		InvalidWrite,
		ThreadCreation
	};

	struct Error
	{
		Error():
			err_val(NoError),
			_errno(0)
		{}
		
		ErrorVal err_val;
		int _errno;
	};
	
	edthreaded_socket(uint socket_fd);
	
	~edthreaded_socket();

    uint read(char * buffer, uint max_size);

    uint write(char * buffer, uint size);

	Error error();

	bool running();

	bool start();

	int fd();
	
	void stop();
	
  private:
	void _exec();
	void _setError(ErrorVal err_val, int _errno);
	static void * thread_exec(void *);
	
	uint m_sckt_fd;
	uint m_recv_rawindex;
	uint m_recv_curindex;
	uint m_send_rawindex;
	uint m_send_curindex;
    char m_send_buffer[SCKT_SEND_BUF_SIZE];
    char m_recv_buffer[SCKT_RECV_BUF_SIZE];

	Error m_err;
	bool m_running;

	pthread_mutex_t m_send_lock;
	pthread_mutex_t m_recv_lock;
	pthread_mutex_t m_error_lock;
	pthread_mutex_t m_running_lock;
	
	pthread_t m_thread;
};

#endif
