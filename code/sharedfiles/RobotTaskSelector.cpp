#include "RobotTaskSelector.h"
#define THISCLASS RobotTaskSelector

#include <math.h>
#include <cv.h>

#include "Random.h"


//#define INIT_LEARNING 1.0

int THISCLASS::SelectTask(RobotDevice* robot,\
 DataStructureShopTasks::tShopTaskVector* taskvec)
 {
    printf(" --- Selecting task for robot %d\n", robot->mID);
    CalculateProbabilities(robot, taskvec);

    ConvertProbabilitiesIntoRange(NEXTSTART);

   mSelectedTaskID = GetRandomSelection();

   return mSelectedTaskID;
 }



void THISCLASS::CalculateProbabilities(RobotDevice* robot,\
 DataStructureShopTasks::tShopTaskVector* taskvec)
 {
   // Find stimulus for each tasks: fn(distance, learn(ignore now), urgency)
   double stimuli;
   double dist;
   double deltadist = DELTA_DISTANCE;
   double learn; // search specific sensitization

   mStimulus.clear();

   // fill up stimulus
   DataStructureShopTasks::tShopTaskVector::iterator it = taskvec->begin();
   int taskid = 0;
   while(it != taskvec->end()){
    dist = CalculateDist(robot->mPose.center, it->mCenter);
    learn = robot->GetSensitization(taskid);
    stimuli = CalculateStimuli(learn, dist, deltadist, it->mUrgency);
    //printf("Robot %d task %d stimuli %f\n", robot->mID, taskid, stimuli);
    mStimulus.push_back(stimuli);

    // save to robot's task record for logging
    robot->mTaskRecords.at(taskid).mID = taskid;
    robot->mTaskRecords.at(taskid).mSensitization = learn;
    robot->mTaskRecords.at(taskid).mDist = dist;
    robot->mTaskRecords.at(taskid).mStimuli = stimuli;
    it++;
    taskid++;
   }

   // sum up all stimulus
   double sum = 0;
   tTaskStimuliVector::iterator itr = mStimulus.begin();
   while(itr != mStimulus.end()) {
     sum += (*itr);
     itr++;
   }

  printf("Robot %d stimulus sum %f\n", robot->mID, sum);

   // fill the probability vector
  mProbabilities.clear();
  double  prob = 0;
  itr = mStimulus.begin();
  taskid = 0;
  while(itr != mStimulus.end()) {
   prob = (*itr)/sum;
   //printf("Robot %d task %d prob %f\n", robot->mID, taskid, prob);
   mProbabilities.push_back(prob);

   // save into robot's task record
   robot->mTaskRecords.at(taskid).mProbability = prob;

   itr++;
   taskid++;
  }

 }

void THISCLASS::ConvertProbabilitiesIntoRange(double nextstart)
{
   int taskid = 0;
   Range r;
   // clear backlog
   mTaskRanges.clear();

   //set first item's range
   tTaskProbabilityVector::iterator itr = mProbabilities.begin();
   r.start = 0;
   r.end = (*itr);
   printf("Task %d prob range: %f to %f\n", taskid, r.start, r.end);
   mTaskRanges.push_back(r);
   taskid++;
   // next items but last one
   for(int i = 1; i < (mProbabilities.size() - 1); i++ ) {
     r.start = (r.end + nextstart);
     r.end = r.end + mProbabilities.at(i);
     mTaskRanges.push_back(r);
     printf("Task %d prob range: %f to %f\n", taskid, r.start, r.end);
   }
   taskid++;
   //set last item's range
   r.start = (r.end + nextstart);
   r.end = 1.0;
   mTaskRanges.push_back(r);
   printf("Task %d prob range: %f to %f\n", taskid, r.start, r.end);
}


int THISCLASS::GetRandomSelection()
{
  Random rng;
  rng.Initialize();
  //double randnum = rng.Uniform(); // within 0 and 1
  //mRandNum = randnum * PROB_SCALE; // scale
  mRandNum = rng.Uniform(0, PROB_SCALE);
  printf("DoRandomSelection(): randnum  : %ld\n", mRandNum);
  rng.Uninitialize();

  // find the task that have this randnum within it's range
  int sn, en;
  for ( int i = 0 ; i < mTaskRanges.size(); i++){
    sn =  mTaskRanges.at(i).start * PROB_SCALE;
    en = mTaskRanges.at(i).end * PROB_SCALE;
    printf("Probing range start:%d end:%d\n", sn, en);
    if ( (mRandNum >= sn) && (mRandNum <= en)){
      //mSelectedTaskID = i;
      printf("RobotTaskSelector --> Task selected: %d \n", i);
      return i;
    }
  }

}

double THISCLASS::CalculateStimuli(double learn, double dist, double deltadist,\
 double  urgency)
{
    // for now: use only disance and urgency statically
    double stimuli;
    stimuli =  tanh(learn * urgency / ( dist + deltadist));
    return stimuli;
}

double THISCLASS::CalculateDist(CvPoint2D32f p1, CvPoint2D32f p2)
{
  double x1 = p1.x;
  double y1 = p1.y;
  double x2 = p2.x;
  double y2 = p2.y;
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}
