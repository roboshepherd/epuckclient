#include "DBusSignalClient.h"
#define THISCLASS DBusSignalClient

#include <iostream>
#include <pthread.h>
#include <signal.h>

using namespace std;


THISCLASS::DBusSignalClient(DBus::Connection &connection, const char *path, const char *name)
: DBus::InterfaceProxy(name), DBus::ObjectProxy(connection, path, name)
{
    connect_signal(DBusSignalClient, DeviceAvailable, DeviceAvailableCb);
    connect_signal(DBusSignalClient, DeviceUnavailableCb, DeviceUnavailableCb);
}

void THISCLASS::DeviceAvailableCb(const DBus::SignalMessage &sig)
{
  printf("Got DeviceAvailable signal: %s \n", sig);

  if(mTaskEngineThread->IsLive()) {

    // resume last thread
    if(mTaskEngineThread->IsPaused()) {
      mTaskEngineThread->Resume();
    } else // or do nothing, allowing existing thread to continue
      return;

  } else {
    mTaskEngineThread = new TaskEngineThread();
    TaskEngineThread->Run();
  }

  // update local RobotDevice
  // commit STATE_SHM (AVAILABLE/TASK)
  //

}

void THISCLASS::DeviceUnavailableCb(const DBus::SignalMessage &sig)
{
  printf("Got DeviceUnavailable signal: %s \n", sig);

  if(mTaskEngineThread->IsLive()) {
    // pause running thread
    mTaskEngineThread->Pause();

  }
  // update local RobotDevice
  // commit STATE_SHM (UNAVAILABLE)
  //

}
