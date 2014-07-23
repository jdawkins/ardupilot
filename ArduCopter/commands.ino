// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);

    // copter uses 0 home altitude
    Location loc = gps.location();

    ahrs.set_home(loc);

    inertial_nav.setup_home_position();

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(loc);
    scaleLongUp   = 1.0f/scaleLongDown;
}



