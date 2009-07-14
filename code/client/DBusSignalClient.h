#ifndef HEADER_DBusSignalClient
#define HEADER_DBusSignalClient

#include <dbus-c++/dbus.h>
#include "echo-client-glue.h"



class DBusSignalClient : public DBus::InterfaceProxy, public DBus::ObjectProxy
{
public:

	DBusSignalClient(DBus::Connection &connection, const char *path, const char *name);

private:

	void DeviceAvailableCb(const DBus::SignalMessage &sig);
	void DeviceUnavailableCb(const DBus::SignalMessage &sig);


};

#endif //HEADER_DBusSignalClient
