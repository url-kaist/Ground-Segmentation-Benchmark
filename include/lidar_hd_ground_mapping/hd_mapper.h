//
// Created by jeewon on 22. 3. 19..
//

#ifndef GSEG_BENCHMARK_HD_MAPPER_H
#define GSEG_BENCHMARK_HD_MAPPER_H

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

void estimate_ground()
{
    ground_mapping::Mapping mapping;

    //install the signal handler to be able to stop with CTRL-C
    signal(SIGINT, &signalHandler);

    mapping.init();
    mapping.loop();

    return 0;
}

#endif //GSEG_BENCHMARK_HD_MAPPER_H
