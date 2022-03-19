/*
 * ground_mapper_node.cpp
 *
 *  Created on: 2017/10/06
 *      Author: alexandr <alexander@g.sp.m.is.nagoya-u.ac.jp>
 */

#include "mapping.h"
#include "ros/ros.h"
#include <signal.h>

void signalHandler(int signum)
{
   if (signum == SIGINT) {
	   printf("End program by kbd interrupt!\n");
	   ground_mapping::Mapping::instance->stop_program_ = 1;
   }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ground_mapping");
	ground_mapping::Mapping mapping;

	//install the signal handler to be able to stop with CTRL-C
	signal(SIGINT, &signalHandler);

	mapping.init(argc,argv);
	mapping.loop();

    return 0;
}
