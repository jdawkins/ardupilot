/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_guided.pde - init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif


// guided_init - initialise guided controller
static bool guided_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
static void guided_run()
{
    float target_climb_rate = 0;
    // if not auto armed set throttle to zero and exit immediately
    /*if(!ap.auto_armed) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }*/
// This logic sets the appropriate controller setpoint values based on the type of guided control_guided
// being used, the low level control loop is called in the 100hz code 		
// body-frame rate controller is run directly from 100hz loop

	switch(attitude_control.get_mode()){
	
		case MAN_STABILIZE : // 0 sets roll pitch yawrate thrust
		{		
			attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(get_smoothing_gain());
			attitude_control.set_throttle_out(attitude_control.get_thrust(), true);
			break;
		}
		
		case MAN_ALT_HOLD: // 1 sets roll pitch yawspeed and climbrate
		{
			attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(get_smoothing_gain());
				
			// run altitude controller
			if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
				// if sonar is ok, use surface tracking
				target_climb_rate = get_throttle_surface_tracking(pos_control.get_target_climb_rate(), pos_control.get_alt_target(), G_Dt);
				pos_control.set_target_climb_rate(target_climb_rate);
			}

			// update altitude target and call position controller
			pos_control.set_alt_target_from_climb_rate(G_Dt);
			pos_control.update_z_controller();
			break;
		}
		
		case MAN_AUTO:  //2 sets roll pitch and yaw angle and climbrate
		{
			attitude_control.angle_ef_roll_pitch_yaw(true); 
					// run altitude controller
			if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
				// if sonar is ok, use surface tracking
				target_climb_rate = get_throttle_surface_tracking(pos_control.get_target_climb_rate(), pos_control.get_alt_target(), G_Dt);
				pos_control.set_target_climb_rate(target_climb_rate);
			}

			// update altitude target and call position controller
			pos_control.set_alt_target_from_climb_rate(G_Dt);
			pos_control.update_z_controller();
			break;
		}
		case MAN_ACRO:  //3 sets rollrate pitchrate yawrate climbrate
		{
			attitude_control.rate_bf_roll_pitch_yaw();
			
			if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
				// if sonar is ok, use surface tracking
				target_climb_rate = get_throttle_surface_tracking(pos_control.get_target_climb_rate(), pos_control.get_alt_target(), G_Dt);
				pos_control.set_target_climb_rate(target_climb_rate);
			}

			// update altitude target and call position controller
			pos_control.set_alt_target_from_climb_rate(G_Dt);
			pos_control.update_z_controller();
			break;		
		}
		case MAN_FREE: // 4 sets rollrate pitchrate yawrate thrust
		{
			attitude_control.rate_bf_roll_pitch_yaw();
			attitude_control.set_throttle_out(attitude_control.get_thrust(), true);
			break;
		}
		default: // Default set to stabilize mode behavior
		{
			attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(get_smoothing_gain());
			attitude_control.set_throttle_out(attitude_control.get_thrust(), true);
			break;
		}
	}

    
}
