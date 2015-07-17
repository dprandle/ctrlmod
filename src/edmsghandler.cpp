#include <edmsghandler.h>
#include <edmessage.h>

edmessage_handler::edmessage_handler()
{
	
}

edmessage_handler::~edmessage_handler()
{
	
}

void edmessage_handler::process_all(edsystem * sys)
{
	edmessage * msg = next(sys);
	while (msg != NULL)
	{
		if (sys->process(msg))
		{
			pop(sys);
			msg = next(sys);
		}
		else
			msg = NULL;
	}
}

edmessage * edmessage_handler::next(edsystem * sys)
{
	listener_queue::iterator fiter = m_lmessages.find(sys);
	if (fiter != m_lmessages.end())
	{
		if (!fiter->second.empty())
			return fiter->second.front();
	}
	return NULL;
}

void edmessage_handler::pop(edsystem * sys)
{
	listener_queue::iterator fiter = m_lmessages.find(sys);
	if (fiter != m_lmessages.end())
	{
		edmessage * msg = fiter->second.front();
		fiter->second.pop_front();
		--msg->ref_count;
		if (msg->ref_count == 0)
			delete msg;
	}
}
