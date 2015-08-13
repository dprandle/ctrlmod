#ifndef EDTHREADED_FD
#define EDTHREADED_FD

#include <edglobal.h>
#include <pthread.h>
#include <vector>
#include <edcallback.h>

#define DEFAULT_FD_WRITE_BUFFER_SIZE 5120
#define DEFAULT_FD_READ_BUFFER_SIZE 5120
#define FD_TMP_BUFFER_SIZE 1024

class edtimer;

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
		ThreadCreation,
		OpenFileDescriptor,
		Configuration,
		AlreadyRunning,
		FDAlreadyOpen,
		CommandNoResponse
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

	struct WriteVal
	{
		WriteVal(char byte_=0x00, int response_size_= 0):
			byte(byte_),
			response_size(response_size_)
		{}
		char byte;
		uint response_size;
	};
	
	edthreaded_fd(
		uint readbuf_size = DEFAULT_FD_READ_BUFFER_SIZE,
		uint writebuf_size = DEFAULT_FD_WRITE_BUFFER_SIZE);
	
	virtual ~edthreaded_fd();

    virtual uint read(char * buffer, uint max_size);

    virtual uint write(char * buffer, uint size, int response_size = 0);

	virtual Error error();

	bool running();

	virtual bool start();

	int fd();

	bool set_fd(int fd_);
	
	virtual void stop();
	
  protected:

	friend struct command_wait_callback;
	
	virtual int _raw_read(char * buffer, uint max_size) = 0;
	virtual int _raw_write(char * buffer, uint max_size) = 0;

	virtual void _do_read();
	virtual void _do_write();
	
	virtual void _exec();
	void _setError(ErrorVal err_val, int _errno);

	static void * thread_exec(void *);
	
	int m_fd;
	uint m_read_rawindex;
	uint m_read_curindex;
	uint m_write_rawindex;
	uint m_write_curindex;
	
	std::vector<WriteVal> m_write_buffer;
	std::vector<char> m_read_buffer;

	Error m_err;
	bool m_running;

	uint m_current_wait_for_byte_count;
	edtimer * m_wait_timer;
	
	pthread_mutex_t m_send_lock;
	pthread_mutex_t m_recv_lock;
	pthread_mutex_t m_error_lock;
	pthread_mutex_t m_running_lock;
	
	pthread_t m_thread;
};

struct command_wait_callback : public wait_ready_callback
{
	command_wait_callback(edthreaded_fd * _handle):
		handle(_handle)
	{}
	
	void exec()
	{
		wait_ready_callback::exec();
		handle->_setError(edthreaded_fd::CommandNoResponse, 0);
	}
	edthreaded_fd * handle;
};

#endif
