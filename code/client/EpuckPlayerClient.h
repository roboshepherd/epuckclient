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

#include "RILSetup.h"
#include "RILObjects.h"

#define DEFAULT_PORT 6600

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

    LiveGraphDataWriter mExptConfWriter;
    LiveGraphDataWriter mTaskDistWriter;
    LiveGraphDataWriter mTaskSzWriter;
    LiveGraphDataWriter mTaskStimulusWriter;
    LiveGraphDataWriter mTaskProbWriter;
    LiveGraphDataWriter mNormPoseWriter;
    LiveGraphDataWriter mTaskUrgencyWriter;
    // task selection
    bool mTaskSelected;
    int mSelectedTask;

    //! Constructor.
    EpuckPlayerClient(char *id, char *port):\
      mClientID(id), mClientPort(atoi(port)),\
      mRobotDevice(), mShopTask(), mShopTasks(), mSHM(), mTaskBroadcasts(),\
      mRobotTaskSelector(), mNavigator(id), mExptConfWriter(),\
      mTaskDistWriter(), mTaskSzWriter(), mTaskStimulusWriter(), mTaskProbWriter(),\
      mNormPoseWriter(),mTaskUrgencyWriter(),\
      mTaskSelected(false),\
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
    void TriggerStateAction( PlayerClient *client, Position2dProxy *p2d,\
      IrProxy *irp, RobotDevice::eState state);

    //! Checkout current pose and update local robot device
    void UpdateCurrentPose();

    //! Checkout task broadcast and update local shoptasks
    void UpdateShopTaskInfo();

    //! Select a Task based on AFM task-allocation engine
    int GetCurrentTask();


    //! Check E-puck State by querying hardware
    RobotDevice::eState GetClientState(PlayerClient *client);

    //! Do Task
    CvPoint2D32f GetTaskCenter(int task);
    void DoTask(int task, PlayerClient *client, Position2dProxy *p2d, IrProxy *irp);

    //! Log config/data
    void InitLogFiles();
    void LogExptConfig(); // Robot id, player port, AFM learn/forget
    std::string GetDataHeader();
    void LogTaskRecords();
    void LogNormalizedPose();


protected:
    // sub fn
    void LogDistToTasks(std::string datahead);
    void LogSensitizations(std::string datahead);
    void LogTaskProbabilities(std::string datahead);
    void LogTaskStimulus(std::string datahead);
    void LogTaskUrgencies(std::string datahead);

};



#endif
