/*
 * MessageHandler.cpp
 *
 *  Created on: 28 Nov 2013
 *      Author: m04701
 */

#include "MessageHandler.h"

MessageHandler::MessageHandler() {
	prev_handler = osg::getNotifyHandler();
	osg::setNotifyHandler(this);
	ignore_msg = "Scaling image from";
}

MessageHandler::~MessageHandler() {
	osg::setNotifyHandler(prev_handler);
}

void MessageHandler::notify(osg::NotifySeverity severity, const char* message) {
	if (severity <= osg::WARN) {
		std::cerr << message;
	} else {
		//If message is ignore_msg then do not show it
		if (ignore_msg.compare(0, ignore_msg.length(), message, 0,
				ignore_msg.length()) != 0) {
			std::cout << message;
		}
	}
}
