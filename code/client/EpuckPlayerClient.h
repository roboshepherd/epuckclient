#ifndef HEADER_EpuckPlayerClient
#define HEADER_EpuckPlayerClient

#include <string>
#include <iostream>
#include <libplayerc++/playerc++.h>

#include "RobotDevice.h"
#include "SHMConfig.h"
#include "RobotTaskSelector.h"
#include "ShopTask.h"
#include "DataStructureShopTasks.h"

#include "EpuckNavigator.h"
#include "LiveGraphDataWriter.h"

#define DEFAULT_PORT 6600


static const char* RILROBOTLIST[] = {
  EPUCK1246,
  EPUCK1250,
  EPUCK1253,
  EPUCK1259,
  EPUCK1260,
  EPUCK1265,
  EPUCK1271,
  EPUCK1302
};




// test setup
static float TASKS_CENTERS[MAXSHOPTASK][XY] = {
    {398, 1753}, // task 1 start point
    {1578, 310},
    {2873, 1986}
};



using namespace PlayerCc;


//! A Player Client for Epuck Robot: "Client" referes to PlayerClient

class EpuckPlayerClient {

  public:

    char* mClientID;                    //!< Same as RobotDevice mID
    int mClientPort;                    //!< Player server port of this client

    RobotDevice mRobotDevice;           //!< Local copy of RobotDevice
    ShopTask mShopTask;                  //!< Tmp copy
    DataStructureShopTasks::tShopTaskVector mShopTasks;  //!< Local copy of shop tasks
    SHMConfig mSHM;                     //!< associated SHMConfig space
    BroadcastBuffer::tTaskBroadcast mTaskBroadcasts[MAXSHOPTASK]; //!< For checking out b/c tasks
    RobotTaskSelector mRobotTaskSelector; //!< associated AFM task allocation engine
    EpuckNavigator mNavigator;
    LiveGraphDataWriter mTaskDistWriter;
    LiveGraphDataWriter mTaskSzWriter;
    LiveGraphDataWriter mTaskProbWriter;
    LiveGraphDataWriter mNormPoseWriter;
    // task selection
    bool mTaskSelected;
    bool mDoingTask;
    bool mTaskDone;
    int mSelectedTask;

    //! Constructor.
    EpuckPlayerClient(char *id, char *port):\
      mClientID(id), mClientPort(atoi(port)),\
      mRobotDevice(), mShopTask(), mShopTasks(), mSHM(), mTaskBroadcasts(),\
      mRobotTaskSelector(), mNavigator(id),\
      mTaskDistWriter(), mTaskSzWriter(), mTaskProbWriter(), mNormPoseWriter(),\
      mTaskSelected(false), mDoingTask(false), mTaskDone(false),\
      mSelectedTask(-1){}

    //! Destructor.
    ~EpuckPlayerClient(){}

    //! Initialize player Client Structures, return PlayerError code
    void InitClient(PlayerClient* pc);

    //! Init local robot device, shop tasks
    void InitRobotDevice();
    void InitShopTasks(int taskcount);
    void SetupStaticTaskLocations(int taskcount, float taskcenters[][XY]);

    //! Trigger appropriate task based on current state
    void TriggerStateAction(PlayerClient *client,\
      Position2dProxy *p2d, IrProxy *irp);

    //! Checkout current pose and update local robot device
    void UpdateCurrentPose();

    //! Checkout task broadcast and update local shoptasks
    void UpdateShopTaskInfo();

    //! Select a Task based on AFM task-allocation engine
    int GetCurrentTask();

    //! LiveGraph logs
    void InitLogFiles();
    void LogDistToTasks();
    void LogSensitizations();
    void LogTaskProbabilities();
    void LogNormalizedPose();

    //! Check E-puck State by querying hardware
    RobotDevice::eState GetClientState(PlayerClient *client);

    //! Do Task
    CvPoint2D32f GetTaskCenter(int task);
    void DoTask(int task, PlayerClient *client, Position2dProxy *p2d, IrProxy *irp);

};



#endif
