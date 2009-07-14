#ifndef Header_ClientMain
#define Header_ClientMain

#include <string>
#include <cstdlib>
#include <iostream>

#include <libplayerc++/playerc++.h>
using namespace PlayerCc;

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
using namespace boost::interprocess;

#include "EpuckPlayerClient.h"
#include "RobotDevice.h"
#include "SHMConfig.h"
#include "Sleep.h"
#include "EpuckNavigator.h"


#endif
