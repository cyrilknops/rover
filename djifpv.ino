msp_analog_t analog = {0};
msp_battery_state_t battery_state = {0};
msp_name_t name = {0};
msp_raw_gps_t raw_gps = {0};
msp_comp_gps_t comp_gps = {0};
msp_attitude_t attitude = {0};
msp_status_t status = {0};
void send_msp_to_airunit()
{

    //MSP_FC_VERSION
    // fc_version.versionMajor = 4;
    // fc_version.versionMinor = 1;
    // fc_version.versionPatchLevel = 1;
    // msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
    //MSP_NAME
    if(batteryCellCount == 0 && vbat > 0)set_battery_cells_number();


    status.flightModeFlags = MSP_MODE_ARM;
    msp.send(MSP_STATUS, &status, sizeof(status));



    memcpy(name.craft_name, craftname, sizeof(craftname));
    msp.send(MSP_NAME, &name, sizeof(name));
    //MSP_ANALOG
    analog.rssi = map(rssiCH,1000,2000,20,1023);
    analog.vbat = vbat;
    msp.send(MSP_ANALOG, &analog, sizeof(analog));

    //MSP_BATTERY_STATE
    battery_state.batteryVoltage = vbat * 10;
    battery_state.batteryCellCount = batteryCellCount;
    //battery_state.batteryState = batteryState;
    battery_state.legacyBatteryVoltage = vbat;
    msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

    //MSP_RAW_GPS
    raw_gps.lat = fix.latitude()*10000000;
    raw_gps.lon = fix.longitude()*10000000;
    raw_gps.alt = fix.altitude();
    raw_gps.groundSpeed = (int16_t)(fix.speed_kph()*27.77777778);
    msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

    //MSP_COMP_GPS
    //comp_gps.distanceToHome = (int16_t)fix.location.DistanceKm( base )*1000;
    //comp_gps.directionToHome = fix.location.BearingToDegrees( base )*57.2957795;
    //msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));
    
    //MSP_OSD_CONFIG
    send_osd_config();
}

msp_osd_config_t msp_osd_config = {0};

void send_osd_config()
{
  
#ifdef IMPERIAL_UNITS
    msp_osd_config.units = 0;
#else
    msp_osd_config.units = 1;
#endif

    msp_osd_config.osd_item_count = 56;
    msp_osd_config.osd_stat_count = 24;
    msp_osd_config.osd_timer_count = 2;
    msp_osd_config.osd_warning_count = 16;              // 16
    msp_osd_config.osd_profile_count = 1;              // 1
    msp_osd_config.osdprofileindex = 1;                // 1
    msp_osd_config.overlay_radio_mode = 0;             //  0
    msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
    msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
    //msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
    msp_osd_config.osd_altitude_pos = osd_altitude_pos;
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
    msp_osd_config.osd_debug_pos = osd_debug_pos;
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
    msp_osd_config.osd_display_name_pos = osd_display_name_pos;

    msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}
