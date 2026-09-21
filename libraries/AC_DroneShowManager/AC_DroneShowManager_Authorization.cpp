#include "AC_DroneShowManager.h"

DroneShowAuthorization AC_DroneShowManager::get_authorization_scope() const
{
    return static_cast<DroneShowAuthorization>(static_cast<int8_t>(_params.authorization));
}

bool AC_DroneShowManager::has_authorization() const
{
    return _params.authorization != DroneShowAuthorization_Revoked;
}

bool AC_DroneShowManager::has_authorization_to_start_motors() const
{
    // TODO(ntamas): add DroneShowAuthorization_Granted_Rehearsal later when we
    // add support for that
    return _params.authorization == DroneShowAuthorization_Granted_Live;
}
