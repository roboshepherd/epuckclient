#include "ClientMain.h"
#define DEFAULT_PORT 6600
#define STEP_TIME 1000 // 3 sec

int main (int argc, char* argv[]){

    EpuckPlayerClient epuck(argv[1], argv[2]);

    // init client stuff
    epuck.mClientID = argv[1];
    epuck.mClientPort = atoi(argv[2]);
    epuck.InitRobotDevice();
    epuck.InitShopTasks(MAXSHOPTASK);
    epuck.SetupStaticTaskLocations(MAXSHOPTASK, TASKS_CENTERS);
    epuck.InitLogFiles();
    epuck.LogExptConfig();

    // player client stuff
    PlayerClient* pc;
    Position2dProxy* p2d;
    IrProxy* irp;
    // device state
    RobotDevice::eState state;

    //Test SHM run by main loop
    epuck.mSHM.FindObject(eBroadcastSHM, epuck.mClientID);

workloop:
    try
    {
        // create client
        PlayerClient client("localhost", epuck.mClientPort);
        epuck.InitClient(&client);
        // create devices
        Position2dProxy pose2d(&client);
        IrProxy ir(&client);
        // player pointers
        pc = &client;
        p2d = &pose2d;
        irp = &ir;

        for(;;){
          // Sense device state
          state =  epuck.GetClientState(pc);
          // Act
          epuck.TriggerStateAction(pc, p2d, irp);

          // Wait a bit
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

