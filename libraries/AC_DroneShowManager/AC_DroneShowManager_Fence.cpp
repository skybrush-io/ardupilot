#include "AC_DroneShowManager.h"

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

static const AP_Int8* find_int8_param(const char* name);

bool AC_DroneShowManager::configure_fences(DroneShow_FenceConfig& config)
{
    uint8_t fence_enabled_mask = 0;
    float value;
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
#endif

    if (config.max_altitude_dm > 0) {
        // Enable altitude fence
        value = config.max_altitude_dm / 10.0f; // [dm] --> [m]
        if (!AP_Param::set_and_save_by_name("FENCE_ALT_MAX", value)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "FENCE_ALT_MAX parameter cannot be set");
            return false;
        }

#if HAL_LOGGING_ENABLED
        if (logger != nullptr) {
            logger->Write_Parameter("FENCE_ALT_MAX", static_cast<float>(value));
        }
#endif

        fence_enabled_mask |= AC_FENCE_TYPE_ALT_MAX;
    }

    if (config.radius_dm > 0) {
        // Enable circular fence
        value = config.radius_dm / 10.0f; // [dm] --> [m]
        if (!AP_Param::set_and_save_by_name("FENCE_RADIUS", value)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "FENCE_RADIUS parameter cannot be set");
            return false;
        }

#if HAL_LOGGING_ENABLED
        if (logger != nullptr) {
            logger->Write_Parameter("FENCE_RADIUS", static_cast<float>(value));
        }
#endif

        fence_enabled_mask |= AC_FENCE_TYPE_CIRCLE;
    }

    if (config.num_points > 0) {
        // Enable polygon fence
        fence_enabled_mask |= AC_FENCE_TYPE_POLYGON;

        // Polygon fence needs a tentative show coordinate system
        if (!_tentative_show_coordinate_system.is_valid()) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Show coordinate system is not valid");
            return false;
        }

        // Construct the items for the polygon fence
        AC_PolyFenceItem *items = new AC_PolyFenceItem[config.num_points];
        if (items == nullptr) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to allocate memory for polygon fence items");
            return false;
        }

        sb_vector3_t vec;
        Location loc;

        for (uint8_t i = 0; i < config.num_points; i++) {
            vec.x = config.points[i].x_dm * 100.0f; // [dm] --> [mm]
            vec.y = config.points[i].y_dm * 100.0f; // [dm] --> [mm]
            vec.z = 0; // Altitude is not used in polygon fences

            // Use the tentative show coordinate system to convert the point
            // from the show coordinate system to the global GPS coordinate system.
            // Note that the show coordinate system is not necessarily valid
            // at this point because it is updated only at takeoff.
            _tentative_show_coordinate_system.convert_show_to_global_coordinate(vec, loc);

            items[i].type = AC_PolyFenceType::POLYGON_INCLUSION;
            items[i].loc.x = loc.lat;
            items[i].loc.y = loc.lng;
            items[i].vertex_count = config.num_points;
            items[i].radius = 0.0f;
        }

        AC_PolyFence_loader &poly_loader = AP::fence()->polyfence();
        if (!poly_loader.write_fence(items, config.num_points)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to write polygon fence");
            delete[] items;
            return false;
        }

        delete[] items;
    }

    // Set fence action
    switch (config.action) {
        case (uint8_t)AC_Fence::Action::REPORT_ONLY:
        case (uint8_t)AC_Fence::Action::RTL_AND_LAND:
        case (uint8_t)AC_Fence::Action::ALWAYS_LAND:
        case (uint8_t)AC_Fence::Action::SMART_RTL:
        case (uint8_t)AC_Fence::Action::SMART_RTL_OR_LAND:
        case (uint8_t)AC_Fence::Action::BRAKE:
            value = config.action;
            if (!AP_Param::set_and_save_by_name("FENCE_ACTION", value)) {
                gcs().send_text(MAV_SEVERITY_ERROR, "FENCE_ACTION parameter cannot be set");
                return false;
            }

#if HAL_LOGGING_ENABLED
            if (logger != nullptr) {
                logger->Write_Parameter("FENCE_ACTION", static_cast<float>(value));
            }
#endif
            break;

        default:
            gcs().send_text(MAV_SEVERITY_ERROR, "Unsupported fence action: %d", config.action);
            return false;
    }

    // FENCE_TYPE selects the fence types, and FENCE_ENABLE switches all of
    // them on or off. AC_Fence applies FENCE_TYPE only when FENCE_ENABLE
    // changes. We also update the set of active fences directly to cover the
    // case when FENCE_ENABLE stays the same. Fences that are not needed any
    // more are disabled first because AC_Fence ignores the fence types that
    // are missing from FENCE_TYPE.
    AC_Fence* fence = AP::fence();
    if (fence) {
        fence->enable(false, ~fence_enabled_mask, false);
    }

    value = fence_enabled_mask;
    if (!AP_Param::set_and_save_by_name("FENCE_TYPE", value)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "FENCE_TYPE parameter cannot be set");
        return false;
    }

#if HAL_LOGGING_ENABLED
    if (logger != nullptr) {
        logger->Write_Parameter("FENCE_TYPE", static_cast<float>(value));
    }
#endif

    value = fence_enabled_mask ? 1 : 0;
    if (!AP_Param::set_and_save_by_name("FENCE_ENABLE", value)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "FENCE_ENABLE parameter cannot be set");
        return false;
    }

#if HAL_LOGGING_ENABLED
    if (logger != nullptr) {
        logger->Write_Parameter("FENCE_ENABLE", static_cast<float>(value));
    }
#endif

    if (fence && fence_enabled_mask) {
        fence->enable(true, fence_enabled_mask, false);
    }

    // Recalculate the hash here instead of waiting for the fence to be
    // reloaded. The reload happens only when the EKF has an origin.
    _update_show_config_hash();

    return true;
}

void AC_DroneShowManager::_check_changes_in_geofence()
{
    const AC_Fence* fence = AP::fence();

    const uint32_t fence_loaded_at_msec = fence ? fence->polyfence().get_inclusion_polygon_update_ms() : 0;

    if (
        fence_loaded_at_msec != _show_config_hash.fence_loaded_at_msec ||
        _is_polygon_fence_enabled() != _show_config_hash.polygon_fence_enabled
    ) {
        _update_show_config_hash();
    }
}

void AC_DroneShowManager::_find_fence_params()
{
    _fence_params.enable = find_int8_param("FENCE_ENABLE");
    _fence_params.type = find_int8_param("FENCE_TYPE");
}

bool AC_DroneShowManager::_is_polygon_fence_enabled() const
{
    // FENCE_ENABLE switches all fences on or off, and FENCE_TYPE selects the
    // fence types that are active when the fences are on
    return (
        _fence_params.enable && _fence_params.type &&
        _fence_params.enable->get() != 0 &&
        (_fence_params.type->get() & AC_FENCE_TYPE_POLYGON)
    );
}

// Looks up an 8-bit integer parameter by its name; returns null if there is
// no such parameter or it has a different type
static const AP_Int8* find_int8_param(const char* name)
{
    enum ap_var_type ptype;
    const AP_Param* param = AP_Param::find(name, &ptype);
    return (param && ptype == AP_PARAM_INT8) ? static_cast<const AP_Int8*>(param) : nullptr;
}
