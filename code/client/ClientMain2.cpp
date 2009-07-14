#include "ClientMain.h"
#include <iostream>
#include <pthread.h>
#include <signal.h>

#include "DBusSignalClient.h"

using namespace std;

static bool spin = true;

DBus::BusDispatcher dispatcher;

void cleanup(int sig)
{
	spin = false;

	dispatcher.leave();
}

int main()
{
	signal(SIGTERM, cleanup);
	signal(SIGINT, cleanup);

	DBus::_init_threading();

	DBus::default_dispatcher = &dispatcher;

	DBus::Connection conn = DBus::Connection::SessionBus();


  DBusSignalClient client(conn, SERVER_PATH, SERVER_NAME);

	dispatcher.enter();

	return 0;
}
