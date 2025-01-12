#include "AC_DroneShowManager.h"

bool AC_DroneShowManager::has_authorization_to_start() const
{
    return _params.authorization == DroneShowAuthorization_Granted_Live;
}
