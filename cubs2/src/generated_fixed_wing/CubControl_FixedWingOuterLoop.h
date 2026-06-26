#pragma once

#include <stddef.h>

typedef double real_t;

enum {
    CUBCONTROL_FIXEDWINGOUTERLOOP_Y_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_P_LEN = 290,
    CUBCONTROL_FIXEDWINGOUTERLOOP_STATE_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_DERIVATIVE_LEN = 0
};/* ---- Variable slot indices (name -> flat array position) ---- */

#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_x 165  /* input x */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_y 166  /* input y */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_z 167  /* input z */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_roll 168  /* input roll */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_pitch 169  /* input pitch */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_yaw 170  /* input yaw */


#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_aileron 171  /* discrete aileron */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_elevator 172  /* discrete elevator */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_throttle 173  /* discrete throttle */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_rudder 174  /* discrete rudder */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_stabilizer 175  /* discrete stabilizer */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_v 176  /* discrete des_v */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_gamma 177  /* discrete des_gamma */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_heading 178  /* discrete des_heading */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_a 179  /* discrete des_a */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_phi_cmd 180  /* discrete phi_cmd */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_chi_err 181  /* discrete chi_err */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_x_est 182  /* discrete x_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_y_est 183  /* discrete y_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_z_est 184  /* discrete z_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_roll_est 185  /* discrete roll_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_pitch_est 186  /* discrete pitch_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_yaw_est 187  /* discrete yaw_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vx_est 188  /* discrete vx_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vy_est 189  /* discrete vy_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vz_est 190  /* discrete vz_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_v_est 191  /* discrete v_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_gamma_est 192  /* discrete gamma_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vdot_est 193  /* discrete vdot_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_p_est 194  /* discrete p_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_q_est 195  /* discrete q_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_r_est 196  /* discrete r_est */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_x 197  /* discrete prev_x */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_y 198  /* discrete prev_y */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_z 199  /* discrete prev_z */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_roll 200  /* discrete prev_roll */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_pitch 201  /* discrete prev_pitch */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_yaw 202  /* discrete prev_yaw */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_speed 203  /* discrete prev_speed */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_time_s 204  /* discrete time_s */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_norm_es_dot_int 205  /* discrete err_norm_es_dot_int */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_dist_term_int 206  /* discrete err_dist_term_int */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_pitch_int 207  /* discrete err_pitch_int */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_r_int 208  /* discrete err_r_int */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_r_last 209  /* discrete err_r_last */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_phi_cmd_state 210  /* discrete phi_cmd_state */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_alpha 211  /* discrete alpha */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vx_new 212  /* discrete vx_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vy_new 213  /* discrete vy_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vz_new 214  /* discrete vz_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_speed_new 215  /* discrete speed_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_p_new 216  /* discrete p_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_q_new 217  /* discrete q_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_r_new 218  /* discrete r_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_gamma_new 219  /* discrete gamma_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_vdot_new 220  /* discrete vdot_new */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_next_wx 221  /* discrete next_wx */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_next_wy 222  /* discrete next_wy */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_next_wz 223  /* discrete next_wz */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_wx 224  /* discrete prev_wx */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_wy 225  /* discrete prev_wy */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_prev_wz 226  /* discrete prev_wz */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_x_err 227  /* discrete x_err */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_y_err 228  /* discrete y_err */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_z_err 229  /* discrete z_err */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_horz_dist_err 230  /* discrete horz_dist_err */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_path_vect_1 231  /* discrete path_vect[1] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_path_vect_2 232  /* discrete path_vect[2] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_path_vect_3 233  /* discrete path_vect[3] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_path_len 234  /* discrete path_len */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_path_angle 235  /* discrete path_angle */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_unit_along_path_1 236  /* discrete unit_along_path[1] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_unit_along_path_2 237  /* discrete unit_along_path[2] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_unit_normal_1 238  /* discrete unit_normal[1] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_unit_normal_2 239  /* discrete unit_normal[2] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_pose_vect_1 240  /* discrete pose_vect[1] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_pose_vect_2 241  /* discrete pose_vect[2] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_along_track_err_w0 242  /* discrete along_track_err_w0 */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_along_track_err_w1 243  /* discrete along_track_err_w1 */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_cross_track_err 244  /* discrete cross_track_err */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_lookahead_nom 245  /* discrete lookahead_nom */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_lookahead_eff 246  /* discrete lookahead_eff */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_switch_threshold 247  /* discrete switch_threshold */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_weight 248  /* discrete weight */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_drag 249  /* discrete drag */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_r_v_dot 250  /* discrete r_v_dot */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_norm_es_dot 251  /* discrete err_norm_es_dot */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_thrust_unsat 252  /* discrete thrust_unsat */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_ref_thrust 253  /* discrete ref_thrust */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_dist_term 254  /* discrete err_dist_term */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_pitch_unsat 255  /* discrete pitch_unsat */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_ref_pitch 256  /* discrete ref_pitch */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_pitch_ned 257  /* discrete pitch_ned */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_pitch 258  /* discrete err_pitch */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_q_turn 259  /* discrete q_turn */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_q 260  /* discrete err_q */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_nz_excess 261  /* discrete nz_excess */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_ele_ff_phi 262  /* discrete ele_ff_phi */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_chi 263  /* discrete chi */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_chi_dot_des 264  /* discrete chi_dot_des */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_phi_des 265  /* discrete phi_des */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_dphi_max 266  /* discrete dphi_max */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_yaw 267  /* discrete err_yaw */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_err_r_deriv 268  /* discrete err_r_deriv */

#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_airborne 269  /* discrete airborne */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_current_wp 270  /* discrete current_wp */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_started 271  /* discrete started */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_1 272  /* discrete c[1] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_2 273  /* discrete c[2] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_3 274  /* discrete c[3] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_4 275  /* discrete c[4] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_5 276  /* discrete c[5] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_6 277  /* discrete c[6] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_7 278  /* discrete c[7] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_8 279  /* discrete c[8] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_9 280  /* discrete c[9] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_10 281  /* discrete c[10] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_11 282  /* discrete c[11] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_12 283  /* discrete c[12] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_13 284  /* discrete c[13] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_14 285  /* discrete c[14] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_15 286  /* discrete c[15] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_16 287  /* discrete c[16] */
#define CUBCONTROL_FIXEDWINGOUTERLOOP_P_c_17 288  /* discrete c[17] */



typedef struct {
    real_t time;
    real_t y[1];
    real_t p[290];
} CubControl_FixedWingOuterLoop_t;

/* Initialise to start values and run the initialization (`initial()`) event. */
void CubControl_FixedWingOuterLoop_init(CubControl_FixedWingOuterLoop_t *m);

/* Continuous derivative RHS: writes dy/dt for the state block into out[]. */
void CubControl_FixedWingOuterLoop_derivative_rhs(const CubControl_FixedWingOuterLoop_t *m, real_t *out);

/* Advance one step of size dt: forward-Euler for continuous states, then one
   discrete/sample tick (snapshot pre(), evaluate when-equations). Call once per
   control period for a fixed-rate discrete controller. */
void CubControl_FixedWingOuterLoop_step(CubControl_FixedWingOuterLoop_t *m, real_t dt);