#ifndef EDTHREADED_FD
#define EDTHREADED_FD

#include <edglobal.h>
#include <pthread.h>
#include <vector>

#define DEFAULT_FD_WRITE_BUFFER_SIZE 5120
#define DEFAULT_FD_READ_BUFFER_SIZE 5120
#define FD_TMP_BUFFER_SIZE 1024

class edthreaded_fd
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
	
	edthreaded_fd(
		uint readbuf_size = DEFAULT_FD_READ_BUFFER_SIZE,
		uint writebuf_size = DEFAULT_FD_WRITE_BUFFER_SIZE);
	
	virtual ~edthreaded_fd();

    virtual uint read(char * buffer, uint max_size);

    virtual uint write(char * buffer, uint size);

	virtual Error error();

	bool running();

	virtual bool start();

	int fd();

	void set_fd(int fd_);
	
	virtual void stop();
	
  protected:

	virtual int _raw_read(char * buffer, uint max_size) = 0;
	virtual int _raw_write(char * buffer, uint max_size) = 0;
	
	virtual void _exec();
	void _setError(ErrorVal err_val, int _errno);

	static void * thread_exec(void *);
	
	int m_fd;
	uint m_read_rawindex;
	uint m_read_curindex;
	uint m_write_rawindex;
	uint m_write_curindex;
	
	std::vector<char> m_write_buffer;
	std::vector<char> m_read_buffer;

	Error m_err;
	bool m_running;

	pthread_mutex_t m_send_lock;
	pthread_mutex_t m_recv_lock;
	pthread_mutex_t m_error_lock;
	pthread_mutex_t m_running_lock;
	
	pthread_t m_thread;
};

#endif
