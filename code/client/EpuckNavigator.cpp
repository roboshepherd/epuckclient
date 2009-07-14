#include "EpuckNavigator.h"
#define  THISCLASS EpuckNavigator

#define REVERSE_ANGLE1 2.90 //
#define REVERSE_ANGLE2 1.52
#define DELTA_ANGLE1 0.26
#define DELTA_ANGLE0 0.26

#define ANGLE30 0.52
#define ANGLE90 M_PI_2
#define ANGLE180 M_PI
#define ANGLE270 (3.0 * M_PI_2)
#define ANGLE360 ( 2.0 * M_PI)

// stop criteria
#define TARGET_DIST 0.30 // /step
#define TARGET_GOSTEP 20

//basic speed/turnrate
#define BASIC_VX 0.06 // velocity m/s
#define NIL 0.0 // default for non-holonomic robot
#define BASIC_VA 0.0 // fix rotation
#define FIX_VA (2.0* ANGLE30)

//obstacle sensing
#define VERY_CLOSE_DIST 0.02 // m -- works fine with light-reflective taps
//set reactions
#define AVOID_SPEED1 -2*BASIC_VX
#define AVOID_SPEED2 2*BASIC_VX
#define AVOID_ANGLE1 -1.52//-0.785 //rad
#define AVOID_ANGLE2 1.52//0.785 //rad
#define AVOID_LOOP_COUNT 6//**ADJUST THIS** total count of doing avoidance in the loop
#define AVOID_TURN_START 3 // In avoidance loop: how many count do turning
#define AFTER_AVOID_WAIT 30 // initail 30

#define MOD 10
// E-puck stuff
#define IR_COUNT 8

void THISCLASS::SetupTaskLoc(CvPoint2D32f center, double radius, double coneangle)
{
  mTaskPose.center = center;
  mTaskRadius = radius;
  mTaskConeAngle = coneangle;
}

void THISCLASS::UpdateCurrentPose()
{
  mRobotPose = mSHM.CheckoutPose(mRobotID);
  printf("@EpuckNavigator: Robot %s PoseFound: %.0f %.0f %.2f\n", mRobotID, mRobotPose.center.x,\
   mRobotPose.center.y, mRobotPose.orient);
}

void THISCLASS::UpdateTaskAngle()
{
  CvPoint2D32f *r = &mRobotPose.center;
  CvPoint2D32f *t = &mTaskPose.center;
  double dx, dy;
  dx = fabs(r->x - t->x);
  dy = fabs(r->y - t->y);
  printf("UpdateTaskAngle(): dy/dx  %.2f %.2f \n", dy, dx);
  mTaskPose.orient = atan2(dy, dx);
}

void THISCLASS::UpdateNavFunc()
{
  CvPoint2D32f *r = &mRobotPose.center;
  CvPoint2D32f *t = &mTaskPose.center;

  double tx1 = (t->x) - mTaskRadius;
  double ty1 = (t->y) - mTaskRadius;
  double tx2 = (t->x) + mTaskRadius;
  double ty2 = (t->y) + mTaskRadius;

  // reset
  mXFunc = NOTSET;
  mYFunc = NOTSET;
  mCurrentQuad = Q0;
  // for four vertical axes
  // xleft
  if ( (r->x < tx1) && (r->y < ty2 ) && (r->y > ty1) ) {
    mXFunc = MAXZ;
    mYFunc = FIXD;
    printf("NavFunc set @xleft \n");
    mCurrentQuad = Q41;
  }
  //xright
  else if ( (r->x > tx2) && (r->y < ty2 ) && (r->y > ty1) ) {
    mXFunc = MINZ;
    mYFunc = FIXD;
    printf("NavFunc set @xright \n");
    mCurrentQuad = Q23;
  }
  //yup
  else if ( (r->x < tx2) && (r->x > tx1 ) && (r->y < ty1) ) {
    mXFunc = FIXD;
    mYFunc = MAXZ;
    printf("NavFunc set @yup \n");
    mCurrentQuad = Q12;
  }
  //udown
   else if ( (r->x < tx2) && (r->x > tx1 ) && (r->y > ty2) ) {
    mXFunc = FIXD;
    mYFunc = MINZ;
    printf("NavFunc set @ydown \n");
    mCurrentQuad = Q34;
  }

  // @Q1 : topleft
  else if ( (r->x < tx1) && (r->y < ty1) ) {
    mXFunc = MAXZ;
    mYFunc = MAXZ;
    mCurrentQuad = Q1;
    printf("NavFunc set @Q1 \n");
  }

  else if ( (r->x > tx2) && (r->y < ty1) ) {
    mXFunc = MINZ;
    mYFunc = MAXZ;
    mCurrentQuad = Q2;
    printf("NavFunc set @Q2 \n");
  }

  else if ( (r->x > tx2) && (r->y > ty2) ) {
    mXFunc = MINZ;
    mYFunc = MINZ;
    mCurrentQuad = Q3;
    printf("NavFunc set @Q3 \n");
  }

  else if ( (r->x < tx1) && (r->y > ty2) ) {
    mXFunc = MAXZ;
    mYFunc = MINZ;
    mCurrentQuad = Q4;
    printf("NavFunc set @Q4 \n");
  }

}

void THISCLASS::RandomWalk(PlayerClient* pc, Position2dProxy* p2d, IrProxy* ir, int cycle)
{
}


 void THISCLASS::GoToTaskLoc(PlayerClient* pc, Position2dProxy* p2d, IrProxy* ir)
 {
  double thetadiff, localangle;

  int step = 0;
  while (!ArrivedAtTaskLoc()){
    step++;
    printf("\n");
    printf(">>>>>>>>>>>>>>>>GoToTaskLoc(): STEP_START **%d** \n", step);
    UpdateCurrentPose();
    // calculate current taskangle
    UpdateTaskAngle();
    // update objective function
    UpdateNavFunc();

    printf("GoToTaskLoc(): TaskPoseOrientation:%f Cone:%f\n", mTaskPose.orient, mTaskConeAngle);
    // Rotation check#1:  see if reverse turn required
    { // empty brace to prevent goto error
      switch (mCurrentQuad) {
        case Q12:
          mThetaDesired = ANGLE90;
          printf("GoToTaskLoc(): **Quad %d**, called ", (int)mCurrentQuad);
        if((mRobotPose.orient > (ANGLE270 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (ANGLE270 + DELTA_ANGLE0))) {
            printf("with TurnReverse \n");
            TurnReverse(pc, p2d, FIX_VA);
            goto translation;
        } else if ((mRobotPose.orient > (ANGLE270 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (ANGLE270 + DELTA_ANGLE0))) {
             goto translation;

        } else {
            printf("with rotation \n");
        }
        break;
        case Q23:
        mThetaDesired = ANGLE180;
        printf("GoToTaskLoc(): **Quad %d**,  called ", (int)mCurrentQuad);
        if((mRobotPose.orient > (ANGLE360 - DELTA_ANGLE0)) ||
           (mRobotPose.orient < (DELTA_ANGLE0))) {
              printf("with TurnReverse \n");
              TurnReverse(pc, p2d, FIX_VA);
              goto translation;
        } else if ((mRobotPose.orient > (ANGLE180 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (ANGLE180 + DELTA_ANGLE0))) {
             goto translation;

        } else {
           printf("with rotation \n");
        }

        break;
        case Q34:
        mThetaDesired = ANGLE270;
        printf("GoToTaskLoc(): **Quad %d**, called ", (int)mCurrentQuad);
        if((mRobotPose.orient > (ANGLE90 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (ANGLE90 + DELTA_ANGLE0))) {
              printf("with TurnReverse \n");
              TurnReverse(pc, p2d, FIX_VA);
              goto translation;
        } else if ((mRobotPose.orient > (ANGLE90 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (ANGLE90 + DELTA_ANGLE0))) {
             goto translation;
        } else {
           printf("with rotation \n");
        }
        break;
        case Q41:
        mThetaDesired = 0;
        printf("GoToTaskLoc(): **Quad %d**, called ", (int)mCurrentQuad);
        if((mRobotPose.orient > (ANGLE180 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (ANGLE180 + DELTA_ANGLE0))) {
              printf("with TurnReverse \n");
              TurnReverse(pc, p2d, FIX_VA);
              goto translation;
        } else if ((mRobotPose.orient > (ANGLE360 - DELTA_ANGLE0)) &&
           (mRobotPose.orient < (DELTA_ANGLE0))) {
             goto translation;

        } else {
          printf("with rotation \n");
        }
        break;
        // Now general checks
        case Q1:
        mThetaDesired = mTaskPose.orient;
        printf("GoToTaskLoc(): **Quad %d**, selected \n", (int)mCurrentQuad);
        break;
        case Q2:
        mThetaDesired = ANGLE180 - mTaskPose.orient;
        printf("GoToTaskLoc(): **Quad %d**, selected \n", (int)mCurrentQuad);
        break;
        case Q3:
        mThetaDesired = ANGLE180 + mTaskPose.orient;
        printf("GoToTaskLoc(): **Quad %d**, selected \n", (int)mCurrentQuad);
        break;
        case Q4:
        mThetaDesired = ANGLE360 - mTaskPose.orient;
        printf("GoToTaskLoc(): **Quad %d**, selected \n", (int)mCurrentQuad);
        break;
      } // end switch (mCurrentQuad)

      // Roatation Calculator //TODO Fix long reverse
      thetadiff =  mRobotPose.orient - mThetaDesired;
      localangle = TRUNC(fabs(thetadiff), 2);

      if(localangle < mTaskConeAngle){
          printf("GoToTaskLoc(): <No Turning> localangle:%.2f < TaskConeAngle:%.2f\n",\
          localangle, mTaskConeAngle);
          goto translation;
      } else { // localangle > mTaskConeAngle
          if (thetadiff < 0) { // Rigt Turn
            printf("GoToTaskLoc(): ThetaDesired: %.2f Localangle %.2f => TurnRight selected \n",\
            mThetaDesired, localangle);
            if (localangle >= ANGLE180) {
               printf("GoToTaskLoc(): Multi step turning needed \n");
                double localangle1 = TRUNC(ANGLE180, 2);
                double localangle2 = TRUNC(localangle - localangle1, 2);
                printf("GoToTaskLoc(): Local angle %.2f + %.2f \n", localangle1, localangle2);
                // first turn
                TurnReverse(pc, p2d, FIX_VA);
                // second turn
                if(localangle2 > mTaskConeAngle) TurnRight(pc, p2d, localangle2, FIX_VA);
            } else {
                TurnRight(pc, p2d, localangle, FIX_VA);
            }
          } else if (thetadiff > 0) { // Left Turn
              printf("GoToTaskLoc(): ThetaDesired: %.2f Localangle %.2f => TurnLeft selected \n",\
              mThetaDesired, localangle);
              if (localangle >= ANGLE180) {
                printf("GoToTaskLoc(): Multi step turning needed \n");
                double localangle1 = TRUNC(ANGLE180, 2);
                double localangle2 = TRUNC(localangle - localangle1, 2);
                printf("GoToTaskLoc(): Local angle %.2f + %.2f \n", localangle1, localangle2);
                // first turn
                TurnReverse(pc, p2d, FIX_VA);
                // second turn
                if(localangle2 > mTaskConeAngle) TurnLeft(pc, p2d, localangle2, FIX_VA);
              } else {
                  TurnLeft(pc, p2d, localangle, FIX_VA);
              }

          }

        }
        Sleep(100);
        goto translation;
  } // end top empty brace */

    // final action, to go front
translation:
{
    printf("NOW Doing Translation\n");
    if(!ArrivedAtTaskLoc()) {
      //p2d->SetSpeed(0.4,-0.26);
      double gone = GoAhead(pc, p2d, ir, BASIC_VX);
      Sleep(100);
    } else {
      Stop(pc, p2d);
    }
    continue;
}

  printf(">>>>>>>>>>>>>>>>GoToTaskLoc(): STEP_END **%d** \n", step);
  printf("\n");
  } // end while

}


bool THISCLASS::ArrivedAtTaskLoc()
{
  bool ret = false;
  UpdateCurrentPose();
  double dx = fabs(mRobotPose.center.x - mTaskPose.center.x);
  double dy = fabs(mRobotPose.center.y - mTaskPose.center.y);
  printf("ArrivedAtTaskLoc(): xdist: %f ydist: %f", dx, dy);

  double pxdist = sqrt(dx * dx + dy * dy);
  printf("********ArrivedAtTaskLoc(): pxdist: %.2f taskradi: %.2f\n", pxdist, mTaskRadius);
  //if((dx < mTaskRadius) || (dy < mTaskRadius)){
  if (pxdist < mTaskRadius) {
    ret = true;
     printf("ArrivedAtTaskLoc(): Arrived within radius %.2f !\n", mTaskRadius);
  }
  else
    ret = false;
  return ret;
}

void THISCLASS::Stop(PlayerClient* client, Position2dProxy* position2d)
{
  client->Read();
  printf("Last local pose (x,y,theta): %.2f %.2f %.2f \n",
    position2d->GetXPos(), position2d->GetYPos(), position2d->GetYaw());

  position2d->SetSpeed(0, 0);
  printf("Stopped robot motion\n");

  UpdateCurrentPose();
  printf("Last global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);
}

void THISCLASS::TurnReverse(PlayerClient* pc, Position2dProxy* p2d, double turnrate)
{
  UpdateCurrentPose();
  printf("Before <reverse> turning global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);

  Turn(pc, p2d, REVERSE_ANGLE1, turnrate);
  Turn(pc, p2d, REVERSE_ANGLE2, turnrate);

   UpdateCurrentPose();
   printf("After reverse turning global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);

}

void THISCLASS::TurnLeft(PlayerClient* pc, Position2dProxy* p2d, double angle, double turnrate)
{
  UpdateCurrentPose();
  printf("Before <left> turning global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);

  Turn(pc, p2d, angle, turnrate);

  UpdateCurrentPose();
  printf("After <left> turning global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);
}

void THISCLASS::TurnRight(PlayerClient* pc, Position2dProxy* p2d, double angle, double turnrate)
{
  UpdateCurrentPose();
  printf("Before <right> turning global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);

  Turn(pc, p2d, angle, -turnrate);

  UpdateCurrentPose();
  printf("After <right> turning global pose (x,y,theta): %.2f %.2f %.2f \n",
    mRobotPose.center.x, mRobotPose.center.y, mRobotPose.orient);
}


double THISCLASS::GoAhead(PlayerClient* client, Position2dProxy* position2d, IrProxy* ir,\
 double speed)
{
  int i = 0, j = 0, k=0;
  double newspeed =  0.0f, newturnrate = 0.0f, gone = 0.0f;
  double frontsum = 0.0f, backsum = 0.0f, dist = 0.0f;
  int avoid = 0, after_avoid = 0;

  position2d->ResetOdometry();
  /* Goes TARGET_GOSTEP forward with avoiding obstacle*/
  while(k < TARGET_GOSTEP) {
    client->Read();
    // Sense Obstacle: check ir only if robot is in no avoiding condition
    for(i=0; (i < IR_COUNT) && (!avoid); i++)
    {
        //calculate ir front and back sum
        if( (i <= 1)|| (i >= 6) ){
            frontsum += ir->GetRange(i);
        }
        if ((i==3)||(i==4)) {
            backsum += ir->GetRange(i);
        }
        // detect obstacle
        if(ir->GetRange(i) < VERY_CLOSE_DIST){
        printf(">>> Got obstacle-- near [IR%d]: %.2f\n", i, ir->GetRange(i) );
        avoid = AVOID_LOOP_COUNT;
        k = 0; // reset main while loop to aoid obstacle for some time
        }
    }

    //Set Reactive Course:  do avoidance
    if(avoid > 0){
        printf("---> Avoiding step: %d\n",avoid);
        if(avoid >= AVOID_TURN_START) // for first few actions
             //go oposite direction
             newspeed = (frontsum > backsum) ? AVOID_SPEED1 : AVOID_SPEED2;
        else { // turn a side and go in last set speed
            newturnrate = AVOID_ANGLE1;
        }
        avoid--;
        after_avoid = 0;

        if(avoid == 0)
          after_avoid = AFTER_AVOID_WAIT;

    } else { // default action: return to normal course
        newspeed = speed; // reset after turn complete
        newturnrate = BASIC_VA;
        frontsum = 0; //clear ir sum
        backsum = 0;
        avoid = 0; //make sure to free avoid counter

        //fix heading after avoid
        if(after_avoid >= AFTER_AVOID_WAIT){
          printf("*******After avoid turn to change direction n escape*****\n");
          Turn(client, position2d, (ANGLE90 + ANGLE30), -FIX_VA);
          after_avoid--;
        } //end if (after_avoid >= AFTER_AVOID_WAIT)


    } // end if(avoid > 0)

    //Command Robot: set velocity and turning rate
    client->Read();
    if(!(j%MOD))printf("  New speed: %.2f, turn:%.2f\n", newspeed, newturnrate);
    position2d->SetSpeed(newspeed, newturnrate);
    j++;

    client->Read();
    gone += position2d->GetXPos();
    position2d->ResetOdometry();
     k++;
  } //end while(position2d->GetXPos() < TARGET_DIST)

  printf("#### Last Normal Course#: %d ####\n", j);
  printf("    (x,y,theta), gone: (%.2f %.2f %.2f), %.2f\n",
                    position2d->GetXPos(), position2d->GetYPos(), position2d->GetYaw(), gone);
  position2d->SetSpeed(0,0);
  position2d->ResetOdometry();
  client->Read();

  return gone;
}

double THISCLASS::Turn(PlayerClient* client, Position2dProxy* position2d, double angle,\
 double turnrate)
{
    position2d->ResetOdometry();
    printf("Start turning for finalangle:%.2f rate:%.2f\n", angle, turnrate);
    client->Read();
    double turned = TRUNC(fabs(position2d->GetYaw()),2);
    int j = 0;
    position2d->SetSpeed(0,turnrate);
    while(turned < angle)
    {
      client->Read();
      turned = TRUNC(fabs(position2d->GetYaw()),2);
       //if(!(j%(MOD*5)))printf(" Turning on spot with (x,y,th, turned): %.2f %.2f %.2f : %.2f\n",
       //      position2d->GetXPos(), position2d->GetYPos(), position2d->GetYaw(), theta);
      j++;
    }
    printf(" Completed turn pose (x,y,th): turned: %.2f, %.2f, %.2f  : %.2f\n",
             position2d->GetXPos(), position2d->GetYPos(), position2d->GetYaw(), turned);
    printf("End turning \n");

    position2d->SetSpeed(0,0);
    Sleep(500);
    position2d->ResetOdometry();
    client->Read();
    //
}
