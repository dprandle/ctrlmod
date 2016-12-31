#include <edmessage_dispatch.h>
#include <edmessage.h>

edmessage_dispatch::edmessage_dispatch()
{
	
}

edmessage_dispatch::~edmessage_dispatch()
{
	
}

void edmessage_dispatch::process_all(edsystem * sys)
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

edmessage * edmessage_dispatch::next(edsystem * sys)
{
	listener_queue::iterator fiter = m_lmessages.find(sys);
	if (fiter != m_lmessages.end())
	{
		if (!fiter->second.empty())
			return fiter->second.front();
	}
	return NULL;
}

void edmessage_dispatch::pop(edsystem * sys)
{
	listener_queue::iterator fiter = m_lmessages.find(sys);
	if (fiter != m_lmessages.end())
	{
		edmessage * msg = fiter->second.front();
		fiter->second.pop_front();
		--msg->ref_count;
		if (msg->ref_count == 0)
        {
			delete msg;
        }
	}
}
