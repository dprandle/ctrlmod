#ifndef MSGANDLER_H
#define MSGANDLER_H

#include <edutility.h>
#include <edglobal.h>
#include <nsmath.h>
#include <map>
#include <set>
#include <deque>
#include <edutility.h>
#include <edsystem.h>

struct edmessage;

//! Class edmessage_handler
/*! 
  A system can register its interest in certain message types, and any time a message of that type is created
  it will be added to that system's message queue. This queue is FIFO, and messages will not be deleted until
  they have been removed from every system's message queue.

  Systems can process all messages in their queue by calling process_all(system*) where system* is a pointer
  to whatever system messages should be processed for (likely "this" pointer). Messages are processed by
  calling the respective system's process function over and over until all messages in the system's message
  que are gone. If process returns false at any point, no more messages will be processed and process_all
  will return.

  You can also process one message at a time by calling next to get the oldest message, and pop to remove
  that message.
 */

class edmessage_handler
{
public:

/*! 
  This maps message type names to sets of systems. Any system that registers with a message type will be
  added to the system set corresponding to that message type.
 */
	typedef std::map< std::string, std::set<edsystem*> > listener_map;

/*! 
  Listener queue holds a map of system pointers to deques of messages. This is FIFO setup - when a message
  is added to the queue it is appended to the back and when one is taken, it is taken from the front.
  This does not actually actually delete the message - the message is not deleted until it is no longer in
  any of the queues. A reference count is kept within the message itself.
 */
	typedef std::map<edsystem*, std::deque<edmessage*> > listener_queue;
	
    edmessage_handler();
    virtual ~edmessage_handler();

	template<class MessageType>
	void register_listener(edsystem * sys)
	{
		std::string id = MessageType::Type();
		m_listeners[id].insert(sys);
		std::ostringstream ss;
		ss << "Registered " << sys->typestr() << " as a listener of " << id;
		log_message(ss.str());
	}

	template<class MessageType>
	void unregister_listener(edsystem * sys)
	{
		std::string id = MessageType::Type;
		listener_map::iterator fiter = m_listeners.find(id);
		if (fiter != m_listeners.end())
		{
			listener_map::size_type ret = fiter->second.erase(sys);
			if (ret != 0)
			{
				std::ostringstream ss;
				ss << "Unregistered " << sys->typestr() << " as a listener of " << id;
				log_message(ss.str());
			}
		}
	}

	template<class MessageType>
	MessageType * push()
	{
		std::string id = MessageType::Type();
		
		listener_map::iterator fiter = m_listeners.find(id);
		if (fiter == m_listeners.end())
			return NULL;

		MessageType * msg = new MessageType();
		std::set<edsystem*>::iterator sys_iter = fiter->second.begin();
		while (sys_iter != fiter->second.end())
		{
			m_lmessages[*sys_iter].push_back(msg);
			++msg->ref_count; // increase reference count
			++sys_iter;
		}
		
		return msg;
	}

	edmessage * next(edsystem * sys);

	void pop(edsystem * sys);

	void process_all(edsystem * sys);
	
private:
	listener_map m_listeners;
	listener_queue m_lmessages;
};


#endif
