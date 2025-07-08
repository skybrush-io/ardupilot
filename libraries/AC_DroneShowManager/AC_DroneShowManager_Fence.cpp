#include "AC_DroneShowManager.h"

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

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

        // TODO(ntamas): coordinate conversion!
        sb_vector3_with_yaw_t vec;
        Location loc;

        for (uint8_t i = 0; i < config.num_points; i++) {
            vec.x = config.points[i].x_dm * 100.0f; // [dm] --> [mm]
            vec.y = config.points[i].y_dm * 100.0f; // [dm] --> [mm]
            vec.z = 0; // Altitude is not used in polygon fences
            vec.yaw = 0; // Yaw is not used in polygon fences

            // Use the tentative show coordinate system to convert the point
            // from the show coordinate system to the global GPS coordinate system.
            // Note that the show coordinate system is not necessarily valid
            // at this point because it is updated only at takeoff.
            _tentative_show_coordinate_system.convert_show_to_global_coordinate(vec, loc);

            memset(&items[i], 0, sizeof(AC_PolyFenceItem));
            items[i].type = AC_PolyFenceType::POLYGON_INCLUSION;
            items[i].loc.x = loc.lat;
            items[i].loc.y = loc.lng;
            items[i].vertex_count = config.num_points;
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
        case AC_FENCE_ACTION_REPORT_ONLY:
        case AC_FENCE_ACTION_RTL_AND_LAND:
        case AC_FENCE_ACTION_ALWAYS_LAND:
        case AC_FENCE_ACTION_SMART_RTL:
        case AC_FENCE_ACTION_SMART_RTL_OR_LAND:
        case AC_FENCE_ACTION_BRAKE:
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

    // Enable the fences
    value = fence_enabled_mask;
    if (!AP_Param::set_and_save_by_name("FENCE_ENABLE", value)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "FENCE_ENABLE parameter cannot be set");
        return false;
    }

#if HAL_LOGGING_ENABLED
    if (logger != nullptr) {
        logger->Write_Parameter("FENCE_ENABLE", static_cast<float>(value));
    }
#endif

    return true;
}
