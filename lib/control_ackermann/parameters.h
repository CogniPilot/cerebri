#ifndef CEREBRI_CONTROL_ACKERMANN_H__
#define CEREBRI_CONTROL_ACKERMANN_H__

//gains
const double gain_heading = 0.1;
const double gain_cross_track = 0.1;
const double gain_along_track = 0.1;
const double max_turn_angle = 0.4;
const double max_velocity = 22;

// parameters
const double wheel_radius = 0.0365;
const double wheel_base = 0.2255;

#endif /* CEREBRI_CONTROL_ACKERMANN_H__ */
