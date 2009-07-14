#ifndef HEADER_EpuckNavigator
#define HEADER_EpuckNavigator

#include <string>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cv.h>
#include <math.h>

#include "RobotDevice.h"
#include "SHMConfig.h"
#include "Sleep.h"
#include "Util.h"

using namespace PlayerCc;
class ShopTask;

#define TASK_RADIUS 100 //pixel
#define TASK_CONE_ANGLE 0.26 // 15deg


//! A class that helps Epuck to perform navigation between two locations
/// using player driver

class EpuckNavigator {

  public:
    char *mRobotID; //!< for POSE_SHM access
    SHMConfig mSHM; //!< Associated SHM
    RobotDevice::tPose mRobotPose; //!< current pose
    RobotDevice::tPose mTaskPose; //!< Target pose
    double mTaskRadius; //!< In Pixel, the raidius of task
    double mTaskConeAngle;

    //! Purpose of navigation
    typedef enum eNavFunc {
      NOTSET = -1,
      MINZ = 1, // minimize axis value
      FIXD = 50, // keep axis value const
      MAXZ = 100, // maximize axis value
    } eNavFuncType;

    eNavFuncType mXFunc;
    eNavFuncType mYFunc;


    typedef enum eQuad {
      Q0 = 0, //unknown
      Q1 = 1, // top left
      Q2 = 2, // top right
      Q3 = 3, // bottom right
      Q4 = 4, // bottom left
      Q12 = 12,
      Q23 = 23,
      Q34 = 34,
      Q41 = 41,
    } eQuadType;

    eQuadType mCurrentQuad; //!< In which quad robot is now situated on ?

    double mThetaDesired; //!< Derived angle from task and robot pose based on current Quad
    double mThetaLocal; //!< mRobotPose.theta - mThetaDesired

    long mStepCount;

    //! Constructor.
    EpuckNavigator(char *id): mRobotID(id), mSHM(), mRobotPose(), mTaskPose(),\
     mTaskRadius(TASK_RADIUS), mTaskConeAngle(TASK_CONE_ANGLE),\
     mXFunc(NOTSET), mYFunc(NOTSET), mCurrentQuad(Q0), mStepCount(0){}

    //! Destructor.
    ~EpuckNavigator(){}

    // Setup locations, update pose
    void SetupTaskLoc(CvPoint2D32f center, double radius, double coneangle);
    void UpdateCurrentPose();
    void UpdateTaskAngle();
    void UpdateNavFunc();

    // Navigation Routines
    void GoToTaskLoc(PlayerClient* pc, Position2dProxy* p2d, IrProxy* ir);
    void RandomWalk(PlayerClient* pc, Position2dProxy* p2d, IrProxy* ir, int cycle);

    // internal fn
    bool ArrivedAtTaskLoc();
    void Stop(PlayerClient* pc, Position2dProxy* p2d);
    void TurnReverse(PlayerClient* pc, Position2dProxy* p2d, double turnrate);
    void TurnLeft(PlayerClient* pc, Position2dProxy* p2d, double angle, double turnrate);
    void TurnRight(PlayerClient* pc, Position2dProxy* p2d, double angle, double turnrate);

    //! Go and return amount moved
    double GoAhead(PlayerClient* pc, Position2dProxy* p2d, IrProxy* ir, double speed);
    //! Turn upto angle at turnrate; which is + for left, - for right, return turned angle
    double Turn(PlayerClient* pc, Position2dProxy* p2d, double angle, double turnrate);
};

#endif
