#include "ClientMain.h"
#define DEFAULT_PORT 6600
#define STEP_TIME 1000 // 3 sec


int main (int argc, char* argv[]){

    EpuckPlayerClient epuck(argv[1], argv[2]);
    //epuck.mClientID = "2";
    //epuck.mClientID = argv[1];
    epuck.InitRobotDevice();
    epuck.InitShopTasks(MAXSHOPTASK);
    epuck.SetupStaticTaskLocations(MAXSHOPTASK, TASKS_CENTERS);

    //EpuckNavigator navigator(epuck.mClientID);

    PlayerClient* pc;
    Position2dProxy* p2d;
    IrProxy* irp;
    epuck.mClientID = argv[1];
    epuck.mClientPort = atoi(argv[2]);

    RobotDevice::eState state;

    //Find  STATE_SHM has this robot's StateBuffer object
    epuck.mSHM.FindObject(eBroadcastSHM, epuck.mClientID);
    //epuck.mSHM.FindObject(eStateSHM, epuck.mClientID);
    //epuck.mSHM.FindObject(eTaskSHM, epuck.mClientID);

//    printf("**Die pt1**\n");
//    TaskBroadcastMessageType msg;
//    msg = epuck.mSHM.CheckoutTaskBroadcastMessage(epuck.mClientID);
//    printf("**Die pt2**\n");


workloop:
    try
    {
        // create client
        PlayerClient client("localhost", epuck.mClientPort);
        epuck.InitClient(&client);
        // create devices
        Position2dProxy pose2d(&client);
        IrProxy ir(&client);

        pc = &client;
        p2d = &pose2d;
        irp = &ir;

        for(;;){
          // Get device state
          state =  epuck.GetClientState(pc);

//          //printf("Skipping state for now... \n");
//          //commit state to STATE_SHM
//          epuck.mSHM.CommitState(epuck.mClientID, state);
//
//          // Checkout pose data
//          epuck.UpdateCurrentPose();
//          task = epuck.GetCurrentTask();
//          //TODO
//          epuck.mNavigator.SetupTaskLoc(cvPoint2D32f(2544, 2253), 300, 0.26);
//
//          printf("[Current task: %d ]\n\n", task);

          // trigger state action
          epuck.TriggerStateAction(pc, p2d, irp);

//          printf("\n ********** TASK LOOP: START ***************\n");
//
//          epuck.mNavigator.GoToTaskLoc(pc, p2d, irp);
//
//          printf("\n ********** TASK LOOP: END ***************\n");

          Sleep(STEP_TIME);
        }


    } catch (PlayerError pe) {
        std::cerr << pe << std::endl;
        goto workloop;
    } catch (interprocess_exception ipce) {
        goto workloop;
    }


    return 0;
}

