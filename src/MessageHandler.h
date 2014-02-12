/*
 * MessageHandler.h
 *
 *  Created on: 28 Nov 2013
 *      Author: m04701
 */

#ifndef MESSAGEHANDLER_H_
#define MESSAGEHANDLER_H_

#include <osg/Notify>
#include <osg/ref_ptr>

#include <iostream>
#include <string>

class MessageHandler: public osg::NotifyHandler {
public:
	MessageHandler();
	virtual ~MessageHandler();

	void notify(osg::NotifySeverity severity, const char *message);
private:
	osg::ref_ptr<osg::NotifyHandler> prev_handler;
	std::string ignore_msg;
};

#endif /* MESSAGEHANDLER_H_ */
