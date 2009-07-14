#include "EpuckPlayerClient.h"
#define  THISCLASS EpuckPlayerClient

#include <cstdlib>
#include "Sleep.h"
#include "Util.h"
#define STEP_TIME 3000 // 3 sec

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
using namespace boost::interprocess;

#include "SHMConfig.h"
#include "ShopTask.h"


void THISCLASS::InitClient(PlayerClient* client)
{
        printf(">>> Start up client \n");
        //printf("--> Set Data mode: pull\n");
        client->SetDataMode(PLAYER_DATAMODE_PULL);
        //printf("---> Set Replace rule mode\n");
        client->SetReplaceRule(TRUE, PLAYER_MSGTYPE_DATA, -1, -1);
        //printf("----> Read client\n");
        client->Read();;
        printf("<<< Client initialized\n");
}

void THISCLASS::InitRobotDevice()
{
  if(mClientID != NULL){
    mRobotDevice.SetDefaults(atoi(mClientID), INIT_LEARN_RATE, INIT_FORGET_RATE);
    mRobotDevice.InitTaskRecords(MAXSHOPTASK);
  } else {
    printf("InitRobotDevice(): Client ID not found \n");
  }
}

void THISCLASS::InitShopTasks(int taskcount)
{
  for(int i=0; i < taskcount ; i++){
        mShopTask.SetDefaults(i, INIT_MATERIAL_COUNT, INIT_URGENCY, DELTA_URGENCY);
        mShopTasks.push_back(mShopTask) ;
        printf("@ClientMain: Added %d shop task\n", i);
  }

  // Setup task locations from initial checkout
//  TaskBroadcastMessageType msg;
//  msg = mSHM.CheckoutTaskBroadcastMessage(mClientID);
//  if (msg.step != mRobotDevice.mBroadcastStep) {
//   mTaskBroadcasts = msg.tasks;
//   mRobotDevice.mBroadcastStep = msg.step;
//   // update/sync. task broadcast structure
//   for(unsigned int i=0; i< mTaskBroadcasts.size(); i++){
//      mShopTasks.at(i).mCenter.x =mTaskBroadcasts.at(i).center.x;
//      mShopTasks.at(i).mCenter.y =mTaskBroadcasts.at(i).center.y;
//   }
//
//  } else {
//    printf("InitShopTask:  no task broadcast since step %ld\n", mRobotDevice.mBroadcastStep);
//  }

}

void THISCLASS::SetupStaticTaskLocations(int taskcount, float taskcenters[][XY])
{
  for(int i=0; i < taskcount ; i++){
        mShopTasks.at(i).mCenter.x =taskcenters[i][0];
        mShopTasks.at(i).mCenter.y =taskcenters[i][1];
  }
}

void THISCLASS::UpdateCurrentPose()
{
  PoseMessageType msg;
  msg = mSHM.CheckoutPoseMessage(mClientID);
  if (msg.step != mRobotDevice.mPoseStep) {
    mRobotDevice.mPose = msg.pose;
    mRobotDevice.mPoseStep = msg.step;
    printf("@ClientMain: Robot %s PoseFound: %.0f %.0f %.2f\n", mClientID, mRobotDevice.mPose.center.x,\
   mRobotDevice.mPose.center.y, mRobotDevice.mPose.orient);
  } else {
    printf("UpdateCurrentPose: pose not updated since step %ld\n", mRobotDevice.mPoseStep);
  }
}

void THISCLASS::UpdateShopTaskInfo()
{
  printf("**Die pt1**\n");
  TaskBroadcastMessageType msg;
  msg = mSHM.CheckoutTaskBroadcastMessage(mClientID);
  printf("**Die pt2**\n");
  if (msg.step != mRobotDevice.mBroadcastStep) {
    //mTaskBroadcasts = msg.tasks;
    mRobotDevice.mBroadcastStep = msg.step;
    // update/sync. task broadcast structure
    for(unsigned int i=0; i< MAXSHOPTASK; i++){
     printf(">>>>>>>>>From Broadcast SHM got task %d urgency %.2f\n", i, msg.tasks[i].urgency);
     mTaskBroadcasts[i].urgency = msg.tasks[i].urgency;
     mShopTasks.at(i).mUrgency = mTaskBroadcasts[i].urgency;
    }
  } else {
    printf("UpdateCurrentPose: task broadcast not updated since step %ld\n",\
     mRobotDevice.mBroadcastStep);
  }

}

RobotDevice::eState THISCLASS::GetClientState(PlayerClient *client)
{
    RobotDevice::eState state = RobotDevice::UNAVAILABLE;

    std::list<playerc_device_info_t> devlist;

    client->RequestDeviceList();
    devlist = client->GetDeviceList();

    printf("GetClientState(): Found %d player devices \n", (int)devlist.size() );

    if(devlist.size()> 0)
        state = RobotDevice::AVAILABLE;

    return state;
}


int THISCLASS::GetCurrentTask()
{
  if(!(&mRobotDevice) || !(&mShopTasks)){
    printf("RobotDevice and/or ShopTasks not initialized yet \n");
    return -1;
  }
  if(!mTaskSelected){
    if(!mDoingTask){
      // update pose and shop tasks
      UpdateCurrentPose();
      UpdateShopTaskInfo();
      mSelectedTask = mRobotTaskSelector.SelectTask(&mRobotDevice, &mShopTasks);

      // convert into proper state
      int taskstate = 0;
      taskstate = (mSelectedTask + 1) * 100 ; // conversion to 100 scale
      mRobotDevice.mState = (RobotDevice::eState) taskstate; // As per API 2
      // log task selections

      // set flag
      mTaskSelected = true;
    }
  }

  return mSelectedTask;
}


void THISCLASS::TriggerStateAction( PlayerClient *client, Position2dProxy *p2d, IrProxy *irp)
{
  // trigger state action: update vitual device and do shm action
  RobotDevice::eState state = GetClientState(client);
  StateMessageType statemsg;
  statemsg.step = ++mRobotDevice.mStateStep; // tick state
  printf("========Client Step [%ld] >>> ", statemsg.step);
  int task = -1;
  switch(state){
    case RobotDevice::NOTSET: //at the beginning
        printf("State: %d ========\n", state );
        mRobotDevice.SetState(state);
        statemsg.state = mRobotDevice.mState;
        mSHM.CommitStateMessage(mClientID, statemsg);
        break;
    case RobotDevice::UNAVAILABLE:
        printf("State: %d ========\n", state );
        mRobotDevice.SetState(state);
        statemsg.state = mRobotDevice.mState;
        mSHM.CommitStateMessage(mClientID, statemsg);
        printf("Device %s still unavailable\n", mClientID);
        break;
    case RobotDevice::AVAILABLE:
        printf("Device %s now available\n", mClientID);
        task = GetCurrentTask();
        printf("Selected task %d now available\n", task);
        statemsg.state = mRobotDevice.mState;
        mSHM.CommitStateMessage(mClientID, statemsg);
        //if(task > 0){
            // navigate to task
        //    DoTask(task, client, p2d, irp);
        //} else if (task == 0){
        //    mRobotDevice.SetState(RobotDevice::RW);
            //mNavigator.RandomWalk(pc, p2d, irp);
        //}
        break;
    //            case RobotDevice::RW:
    //                task = mRobotTaskSelector.SelectTask(&mRobotDevice, &mShopTasks);
    //                mRobotDevice.SetTask(task);
    //                if(task > 0){
    //                     mRobotDevice.SetState(RobotDevice::TASK);
    //                } else if (task == 0){
    //                    mRobotDevice.SetState(RobotDevice::RW);
    //                }
    //                mSHM.CommitStateMessage(mClientID, statemsg);
    //                break;
    case RobotDevice::TASK:
        statemsg.state = mRobotDevice.mState;
        mSHM.CommitStateMessage(mClientID, statemsg);
        break;
    default:
        printf("TriggerStateAction(): Unknown state given \n");
  }
}

CvPoint2D32f THISCLASS::GetTaskCenter(int task)
{
  CvPoint2D32f center;
  for (unsigned int i = 0; i < mShopTasks.size(); i++) {
    if(i == (unsigned )task){
      center = mShopTasks.at(i).mCenter;
      break;
    }
  }
  return center;
}


void THISCLASS::DoTask(int task, PlayerClient *client,\
   Position2dProxy *p2d, IrProxy *irp)
{
  // get task center
  CvPoint2D32f center = GetTaskCenter(task);
  mNavigator.SetupTaskLoc(center, TASK_RADIUS, TASK_CONE_ANGLE );
  printf("\n ********** TASK LOOP: START ***************\n");
  mNavigator.GoToTaskLoc(client, p2d, irp);
  printf("\n ********** TASK LOOP: END ***************\n");
}

void THISCLASS::InitLogFiles()
{
  /* Create log file to save tasks' distance values*/
  char s1[DATA_ITEM_LEN];
  std::string objtype = "Robot";
  LiveGraphDataWriter::DataContextType ctx;
  ctx.name = "DistanceToTasks";
  ctx.sep = ";";
  ctx.desc = "Task distances (d) in global broadcast mode";
  ctx.label = "TimeStamp;StepCounter";
  // preapre data label
  for(unsigned  int i=0; i< mShopTasks.size(); i++){
    sprintf(s1, ";Task%d", mShopTasks.at(i).mID);
    ctx.label += s1;
  }

  mTaskDistWriter.InitDataFile(objtype, &ctx);

  /* Create log file to save all tasks' Sensitizations*/
  ctx.name = "TaskSensitizations";
  ctx.desc = "Task Sensitizations in global broadcast mode";

  mTaskSzWriter.InitDataFile(objtype, &ctx);

    /* Create log file to save all tasks' Probablities*/
  ctx.name = "TaskProbablities";
  ctx.desc = "Task Sensitizations in global broadcast mode";

  mTaskSzWriter.InitDataFile(objtype, &ctx);


  /* Create log file to save all pose normalized between 0~1 */
  ctx.name = "NormalizedPose";
  ctx.desc = "Robot normalized pose in global broadcast experiment";
  ctx.label = "TimeStamp;StepCounter;X;Y;Theta";
  mNormPoseWriter.InitDataFile(objtype, &ctx);

  /* Save Static Expt. Config */
  ctx.name = "ExptStaticConfig";
  ctx.desc = "Configuartaion for current expt.";
  ctx.label = "TimeStamp;StepCounter";
  mExptConfWriter.InitDataFile(objtype, &ctx);

}

void THISCLASS::LogExptConfig()
{
  char buff[128];
  sprintf(buff, "RobotID: %s \n Player server port: %d \n\n",\
   mClientID, mClientPort);
  std::string comment = mExptConfWriter.GetTimeStamp();
  comment.append(buff);

  comment.append("[AFM Parameters for Robots]");
  sprintf(buff, "Init Task Sensitization: %.2f\n", INIT_SENSITIZATION);
  comment.append(buff);
  sprintf(buff, "Task Learn Rate: %.2f\n", INIT_LEARN_RATE);
  comment.append(buff);
  sprintf(buff, "Task Forget Rate: %.2f\n", INIT_LEARN_RATE);
  comment.append(buff);

  comment.append("[AFM Parameters for Tasks]");
  sprintf(buff, "Task Initial Urgency: %.2f\n", INIT_URGENCY);
  comment.append(buff);
  sprintf(buff, "Task Delta Urgency: %.2f\n", DELTA_URGENCY);
  comment.append(buff);

  comment.append("[AFM Misc. Parameters ]");
  sprintf(buff, "Task Delta Distance: %.2f\n", DELTA_DISTANCE);
  comment.append(buff);

  comment.append("[Task Navigation Parameters]");
  sprintf(buff, "Total Tasks: %d\n", MAXSHOPTASK);
  comment.append(buff);
  // task centers
  for (int i=0; i < MAXSHOPTASK; i++) {
    sprintf(buff, "Task % center [%.0f, %0.f]", i,
      mShopTasks.at(i).mCenter.x, mShopTasks.at(i).mCenter.y);
      comment.append(buff);
  }
  sprintf(buff, "Task Radius: %.2f\n", TASK_RADIUS);
  comment.append(buff);
  sprintf(buff, "Task Cone Angle: %.2f\n", TASK_CONE_ANGLE);
  comment.append(buff);

  //add comment
  mExptConfWriter.AppendComment(comment);
}


void THISCLASS::LogDistToTasks()
{

}

void THISCLASS::LogSensitizations()
{
}

void THISCLASS::LogTaskProbabilities()
{
}

void THISCLASS::LogNormalizedPose()
{
}
