#pragma once

#include <stddef.h>

typedef double real_t;

enum {
    CUBCONTROL_FIXEDWINGOUTERLOOP_Y_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_P_LEN = 323,
    CUBCONTROL_FIXEDWINGOUTERLOOP_STATE_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_DERIVATIVE_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_EVENT_INDICATOR_LEN = 25,
    CUBCONTROL_FIXEDWINGOUTERLOOP_PERIODIC_EVENT_LEN = 1
};

/* Named Solve slots. Array symbols point at the first scalar slot and define
 * *_LEN for the contiguous scalar span; indexed symbols have length 1.
 */
#define MODEL_P_DT 0 /* p[0]: dt */
#define MODEL_P_G 1 /* p[1]: g */
#define MODEL_P_NWAYPOINTS 2 /* p[2]: nWaypoints */
#define MODEL_P_WAYPOINTS 3 /* p[3]: waypoints */
#define MODEL_P_WAYPOINTS_LEN 18
#define MODEL_P_WAYPOINTS_1 3 /* p[3]: waypoints[1] */
#define MODEL_P_WAYPOINTS_1_1 3 /* p[3]: waypoints[1,1] */
#define MODEL_P_WAYPOINTS_2 4 /* p[4]: waypoints[2] */
#define MODEL_P_WAYPOINTS_1_2 4 /* p[4]: waypoints[1,2] */
#define MODEL_P_WAYPOINTS_3 5 /* p[5]: waypoints[3] */
#define MODEL_P_WAYPOINTS_1_3 5 /* p[5]: waypoints[1,3] */
#define MODEL_P_WAYPOINTS_4 6 /* p[6]: waypoints[4] */
#define MODEL_P_WAYPOINTS_2_1 6 /* p[6]: waypoints[2,1] */
#define MODEL_P_WAYPOINTS_5 7 /* p[7]: waypoints[5] */
#define MODEL_P_WAYPOINTS_2_2 7 /* p[7]: waypoints[2,2] */
#define MODEL_P_WAYPOINTS_6 8 /* p[8]: waypoints[6] */
#define MODEL_P_WAYPOINTS_2_3 8 /* p[8]: waypoints[2,3] */
#define MODEL_P_WAYPOINTS_7 9 /* p[9]: waypoints[7] */
#define MODEL_P_WAYPOINTS_3_1 9 /* p[9]: waypoints[3,1] */
#define MODEL_P_WAYPOINTS_8 10 /* p[10]: waypoints[8] */
#define MODEL_P_WAYPOINTS_3_2 10 /* p[10]: waypoints[3,2] */
#define MODEL_P_WAYPOINTS_9 11 /* p[11]: waypoints[9] */
#define MODEL_P_WAYPOINTS_3_3 11 /* p[11]: waypoints[3,3] */
#define MODEL_P_WAYPOINTS_10 12 /* p[12]: waypoints[10] */
#define MODEL_P_WAYPOINTS_4_1 12 /* p[12]: waypoints[4,1] */
#define MODEL_P_WAYPOINTS_11 13 /* p[13]: waypoints[11] */
#define MODEL_P_WAYPOINTS_4_2 13 /* p[13]: waypoints[4,2] */
#define MODEL_P_WAYPOINTS_12 14 /* p[14]: waypoints[12] */
#define MODEL_P_WAYPOINTS_4_3 14 /* p[14]: waypoints[4,3] */
#define MODEL_P_WAYPOINTS_13 15 /* p[15]: waypoints[13] */
#define MODEL_P_WAYPOINTS_5_1 15 /* p[15]: waypoints[5,1] */
#define MODEL_P_WAYPOINTS_14 16 /* p[16]: waypoints[14] */
#define MODEL_P_WAYPOINTS_5_2 16 /* p[16]: waypoints[5,2] */
#define MODEL_P_WAYPOINTS_15 17 /* p[17]: waypoints[15] */
#define MODEL_P_WAYPOINTS_5_3 17 /* p[17]: waypoints[5,3] */
#define MODEL_P_WAYPOINTS_16 18 /* p[18]: waypoints[16] */
#define MODEL_P_WAYPOINTS_6_1 18 /* p[18]: waypoints[6,1] */
#define MODEL_P_WAYPOINTS_17 19 /* p[19]: waypoints[17] */
#define MODEL_P_WAYPOINTS_6_2 19 /* p[19]: waypoints[6,2] */
#define MODEL_P_WAYPOINTS_18 20 /* p[20]: waypoints[18] */
#define MODEL_P_WAYPOINTS_6_3 20 /* p[20]: waypoints[6,3] */
#define MODEL_P_FILTERCUTOFFHZ 21 /* p[21]: filterCutoffHz */
#define MODEL_P_PATHDISTANCEBUF 22 /* p[22]: pathDistanceBuf */
#define MODEL_P_WAYPOINTSWITCHINGDISTANCE 23 /* p[23]: waypointSwitchingDistance */
#define MODEL_P_LOOKAHEADTIME 24 /* p[24]: lookaheadTime */
#define MODEL_P_LOOKAHEADMIN 25 /* p[25]: lookaheadMin */
#define MODEL_P_LOOKAHEADMAX 26 /* p[26]: lookaheadMax */
#define MODEL_P_VCRUISE 27 /* p[27]: vCruise */
#define MODEL_P_TAKEOFFALTITUDE 28 /* p[28]: takeoffAltitude */
#define MODEL_P_TAKEOFFTHROTTLEMIN 29 /* p[29]: takeoffThrottleMin */
#define MODEL_P_TAKEOFFTHROTTLERATE 30 /* p[30]: takeoffThrottleRate */
#define MODEL_P_TAKEOFFSPEED 31 /* p[31]: takeoffSpeed */
#define MODEL_P_TAKEOFFELEVDOWN 32 /* p[32]: takeoffElevDown */
#define MODEL_P_TAKEOFFELEVUP 33 /* p[33]: takeoffElevUp */
#define MODEL_P_TAKEOFFELEVRATE 34 /* p[34]: takeoffElevRate */
#define MODEL_P_MASS 35 /* p[35]: mass */
#define MODEL_P_TRIMTHRUST 36 /* p[36]: trimThrust */
#define MODEL_P_TRIMELEV 37 /* p[37]: trimElev */
#define MODEL_P_TRIMRUD 38 /* p[38]: trimRud */
#define MODEL_P_TRIMAIL 39 /* p[39]: trimAil */
#define MODEL_P_THRMAX 40 /* p[40]: thrMax */
#define MODEL_P_K_THRUSTP 41 /* p[41]: K_thrustp */
#define MODEL_P_K_THRUSTI 42 /* p[42]: K_thrusti */
#define MODEL_P_K_PITCHP 43 /* p[43]: K_pitchp */
#define MODEL_P_K_PITCHI 44 /* p[44]: K_pitchi */
#define MODEL_P_K_ELEVP 45 /* p[45]: K_elevp */
#define MODEL_P_K_ELEVI 46 /* p[46]: K_elevi */
#define MODEL_P_K_Q 47 /* p[47]: K_q */
#define MODEL_P_K_PHI_ELEV 48 /* p[48]: K_phi_elev */
#define MODEL_P_K_DELTAP 49 /* p[49]: K_deltap */
#define MODEL_P_K_DELTAI 50 /* p[50]: K_deltai */
#define MODEL_P_K_DELTAD 51 /* p[51]: K_deltad */
#define MODEL_P_PITCHINTEGRALMAX 52 /* p[52]: pitchIntegralMax */
#define MODEL_P_NORMESDOTINTEGRALMAX 53 /* p[53]: normEsDotIntegralMax */
#define MODEL_P_DISTTERMINTEGRALMAX 54 /* p[54]: distTermIntegralMax */
#define MODEL_P_RINTEGRALMAX 55 /* p[55]: rIntegralMax */
#define MODEL_P_ROLLINTEGRALMAX 56 /* p[56]: rollIntegralMax */
#define MODEL_P_K_ROLLP 57 /* p[57]: K_rollp */
#define MODEL_P_K_ROLLI 58 /* p[58]: K_rolli */
#define MODEL_P_KCHI 59 /* p[59]: kChi */
#define MODEL_P_PHILIM 60 /* p[60]: phiLim */
#define MODEL_P_PHIDOTLIM 61 /* p[61]: phiDotLim */
#define MODEL_P_CHIDEADBAND 62 /* p[62]: chiDeadband */
#define MODEL_P_PHISTICKLIMIT 63 /* p[63]: phiStickLimit */
#define MODEL_P_PRE_DT 64 /* p[64]: __pre__.dt */
#define MODEL_P_PRE_ALPHA 65 /* p[65]: __pre__.alpha */
#define MODEL_P_PRE_WEIGHT 66 /* p[66]: __pre__.weight */
#define MODEL_P_PRE_PREV_X 67 /* p[67]: __pre__.prev_x */
#define MODEL_P_PRE_PREV_Y 68 /* p[68]: __pre__.prev_y */
#define MODEL_P_PRE_PREV_Z 69 /* p[69]: __pre__.prev_z */
#define MODEL_P_PRE_PREV_ROLL 70 /* p[70]: __pre__.prev_roll */
#define MODEL_P_PRE_PREV_PITCH 71 /* p[71]: __pre__.prev_pitch */
#define MODEL_P_PRE_PREV_YAW 72 /* p[72]: __pre__.prev_yaw */
#define MODEL_P_PRE_V_EST 73 /* p[73]: __pre__.v_est */
#define MODEL_P_PRE_PREV_SPEED 74 /* p[74]: __pre__.prev_speed */
#define MODEL_P_PRE_X_EST 75 /* p[75]: __pre__.x_est */
#define MODEL_P_PRE_Y_EST 76 /* p[76]: __pre__.y_est */
#define MODEL_P_PRE_Z_EST 77 /* p[77]: __pre__.z_est */
#define MODEL_P_PRE_ROLL_EST 78 /* p[78]: __pre__.roll_est */
#define MODEL_P_PRE_PITCH_EST 79 /* p[79]: __pre__.pitch_est */
#define MODEL_P_PRE_YAW_EST 80 /* p[80]: __pre__.yaw_est */
#define MODEL_P_PRE_VX_NEW 81 /* p[81]: __pre__.vx_new */
#define MODEL_P_PRE_VY_NEW 82 /* p[82]: __pre__.vy_new */
#define MODEL_P_PRE_VZ_NEW 83 /* p[83]: __pre__.vz_new */
#define MODEL_P_PRE_SPEED_NEW 84 /* p[84]: __pre__.speed_new */
#define MODEL_P_PRE_P_NEW 85 /* p[85]: __pre__.p_new */
#define MODEL_P_PRE_Q_NEW 86 /* p[86]: __pre__.q_new */
#define MODEL_P_PRE_R_NEW 87 /* p[87]: __pre__.r_new */
#define MODEL_P_PRE_GAMMA_NEW 88 /* p[88]: __pre__.gamma_new */
#define MODEL_P_PRE_VDOT_NEW 89 /* p[89]: __pre__.vdot_new */
#define MODEL_P_PRE_VX_EST 90 /* p[90]: __pre__.vx_est */
#define MODEL_P_PRE_VY_EST 91 /* p[91]: __pre__.vy_est */
#define MODEL_P_PRE_VZ_EST 92 /* p[92]: __pre__.vz_est */
#define MODEL_P_PRE_GAMMA_EST 93 /* p[93]: __pre__.gamma_est */
#define MODEL_P_PRE_VDOT_EST 94 /* p[94]: __pre__.vdot_est */
#define MODEL_P_PRE_P_EST 95 /* p[95]: __pre__.p_est */
#define MODEL_P_PRE_Q_EST 96 /* p[96]: __pre__.q_est */
#define MODEL_P_PRE_R_EST 97 /* p[97]: __pre__.r_est */
#define MODEL_P_PRE_AIRBORNE 98 /* p[98]: __pre__.airborne */
#define MODEL_P_PRE_TIME_S 99 /* p[99]: __pre__.time_s */
#define MODEL_P_PRE_TAKEOFF_TIME 100 /* p[100]: __pre__.takeoff_time */
#define MODEL_P_PRE_THROTTLE 101 /* p[101]: __pre__.throttle */
#define MODEL_P_PRE_ERR_NORM_ES_DOT_INT 102 /* p[102]: __pre__.err_norm_es_dot_int */
#define MODEL_P_PRE_CURRENT_WP 103 /* p[103]: __pre__.current_wp */
#define MODEL_P_PRE_REF_THRUST 104 /* p[104]: __pre__.ref_thrust */
#define MODEL_P_PRE_RUDDER 105 /* p[105]: __pre__.rudder */
#define MODEL_P_PRE_PHI_CMD_STATE 106 /* p[106]: __pre__.phi_cmd_state */
#define MODEL_P_PRE_CHI_ERR 107 /* p[107]: __pre__.chi_err */
#define MODEL_P_PRE_ERR_ROLL_INT 108 /* p[108]: __pre__.err_roll_int */
#define MODEL_P_PRE_AILERON 109 /* p[109]: __pre__.aileron */
#define MODEL_P_PRE_ELEVATOR 110 /* p[110]: __pre__.elevator */
#define MODEL_P_PRE_ERR_DIST_TERM_INT 111 /* p[111]: __pre__.err_dist_term_int */
#define MODEL_P_PRE_REF_PITCH 112 /* p[112]: __pre__.ref_pitch */
#define MODEL_P_PRE_ERR_PITCH_INT 113 /* p[113]: __pre__.err_pitch_int */
#define MODEL_P_PRE_DES_V 114 /* p[114]: __pre__.des_v */
#define MODEL_P_PRE_DES_GAMMA 115 /* p[115]: __pre__.des_gamma */
#define MODEL_P_PRE_DES_HEADING 116 /* p[116]: __pre__.des_heading */
#define MODEL_P_PRE_DES_A 117 /* p[117]: __pre__.des_a */
#define MODEL_P_PRE_PREV_WX 118 /* p[118]: __pre__.prev_wx */
#define MODEL_P_PRE_PREV_WY 119 /* p[119]: __pre__.prev_wy */
#define MODEL_P_PRE_PREV_WZ 120 /* p[120]: __pre__.prev_wz */
#define MODEL_P_PRE_NEXT_WX 121 /* p[121]: __pre__.next_wx */
#define MODEL_P_PRE_NEXT_WY 122 /* p[122]: __pre__.next_wy */
#define MODEL_P_PRE_NEXT_WZ 123 /* p[123]: __pre__.next_wz */
#define MODEL_P_PRE_X_ERR 124 /* p[124]: __pre__.x_err */
#define MODEL_P_PRE_Y_ERR 125 /* p[125]: __pre__.y_err */
#define MODEL_P_PRE_Z_ERR 126 /* p[126]: __pre__.z_err */
#define MODEL_P_PRE_HORZ_DIST_ERR 127 /* p[127]: __pre__.horz_dist_err */
#define MODEL_P_PRE_PATH_VECT 128 /* p[128]: __pre__.path_vect */
#define MODEL_P_PRE_PATH_VECT_LEN 3
#define MODEL_P_PRE_PATH_VECT_1 128 /* p[128]: __pre__.path_vect[1] */
#define MODEL_P_PRE_PATH_VECT_2 129 /* p[129]: __pre__.path_vect[2] */
#define MODEL_P_PRE_PATH_VECT_3 130 /* p[130]: __pre__.path_vect[3] */
#define MODEL_P_PRE_PATH_LEN 131 /* p[131]: __pre__.path_len */
#define MODEL_P_PRE_PATH_ANGLE 132 /* p[132]: __pre__.path_angle */
#define MODEL_P_PRE_UNIT_ALONG_PATH 133 /* p[133]: __pre__.unit_along_path */
#define MODEL_P_PRE_UNIT_ALONG_PATH_LEN 2
#define MODEL_P_PRE_UNIT_ALONG_PATH_1 133 /* p[133]: __pre__.unit_along_path[1] */
#define MODEL_P_PRE_UNIT_ALONG_PATH_2 134 /* p[134]: __pre__.unit_along_path[2] */
#define MODEL_P_PRE_UNIT_NORMAL 135 /* p[135]: __pre__.unit_normal */
#define MODEL_P_PRE_UNIT_NORMAL_LEN 2
#define MODEL_P_PRE_UNIT_NORMAL_1 135 /* p[135]: __pre__.unit_normal[1] */
#define MODEL_P_PRE_UNIT_NORMAL_2 136 /* p[136]: __pre__.unit_normal[2] */
#define MODEL_P_PRE_POSE_VECT 137 /* p[137]: __pre__.pose_vect */
#define MODEL_P_PRE_POSE_VECT_LEN 2
#define MODEL_P_PRE_POSE_VECT_1 137 /* p[137]: __pre__.pose_vect[1] */
#define MODEL_P_PRE_POSE_VECT_2 138 /* p[138]: __pre__.pose_vect[2] */
#define MODEL_P_PRE_ALONG_TRACK_ERR_W0 139 /* p[139]: __pre__.along_track_err_w0 */
#define MODEL_P_PRE_ALONG_TRACK_ERR_W1 140 /* p[140]: __pre__.along_track_err_w1 */
#define MODEL_P_PRE_CROSS_TRACK_ERR 141 /* p[141]: __pre__.cross_track_err */
#define MODEL_P_PRE_LOOKAHEAD_NOM 142 /* p[142]: __pre__.lookahead_nom */
#define MODEL_P_PRE_LOOKAHEAD_EFF 143 /* p[143]: __pre__.lookahead_eff */
#define MODEL_P_PRE_DRAG 144 /* p[144]: __pre__.drag */
#define MODEL_P_PRE_R_V_DOT 145 /* p[145]: __pre__.r_v_dot */
#define MODEL_P_PRE_ERR_NORM_ES_DOT 146 /* p[146]: __pre__.err_norm_es_dot */
#define MODEL_P_PRE_THRUST_UNSAT 147 /* p[147]: __pre__.thrust_unsat */
#define MODEL_P_PRE_ERR_DIST_TERM 148 /* p[148]: __pre__.err_dist_term */
#define MODEL_P_PRE_PITCH_UNSAT 149 /* p[149]: __pre__.pitch_unsat */
#define MODEL_P_PRE_ERR_PITCH 150 /* p[150]: __pre__.err_pitch */
#define MODEL_P_PRE_Q_TURN 151 /* p[151]: __pre__.q_turn */
#define MODEL_P_PRE_ERR_Q 152 /* p[152]: __pre__.err_q */
#define MODEL_P_PRE_NZ_EXCESS 153 /* p[153]: __pre__.nz_excess */
#define MODEL_P_PRE_ELE_FF_PHI 154 /* p[154]: __pre__.ele_ff_phi */
#define MODEL_P_PRE_CHI 155 /* p[155]: __pre__.chi */
#define MODEL_P_PRE_CHI_DOT_DES 156 /* p[156]: __pre__.chi_dot_des */
#define MODEL_P_PRE_PHI_DES 157 /* p[157]: __pre__.phi_des */
#define MODEL_P_PRE_DPHI_MAX 158 /* p[158]: __pre__.dphi_max */
#define MODEL_P_PRE_PHI_CMD 159 /* p[159]: __pre__.phi_cmd */
#define MODEL_P_PRE_ERR_YAW 160 /* p[160]: __pre__.err_yaw */
#define MODEL_P_PRE_ERR_ROLL 161 /* p[161]: __pre__.err_roll */
#define MODEL_P_PRE_SWITCH_THRESHOLD 162 /* p[162]: __pre__.switch_threshold */
#define MODEL_P_PRE_STABILIZER 163 /* p[163]: __pre__.stabilizer */
#define MODEL_P_PRE_C 164 /* p[164]: __pre__.c */
#define MODEL_P_PRE_C_LEN 25
#define MODEL_P_PRE_C_1 164 /* p[164]: __pre__.c[1] */
#define MODEL_P_PRE_C_2 165 /* p[165]: __pre__.c[2] */
#define MODEL_P_PRE_C_3 166 /* p[166]: __pre__.c[3] */
#define MODEL_P_PRE_C_4 167 /* p[167]: __pre__.c[4] */
#define MODEL_P_PRE_C_5 168 /* p[168]: __pre__.c[5] */
#define MODEL_P_PRE_C_6 169 /* p[169]: __pre__.c[6] */
#define MODEL_P_PRE_C_7 170 /* p[170]: __pre__.c[7] */
#define MODEL_P_PRE_C_8 171 /* p[171]: __pre__.c[8] */
#define MODEL_P_PRE_C_9 172 /* p[172]: __pre__.c[9] */
#define MODEL_P_PRE_C_10 173 /* p[173]: __pre__.c[10] */
#define MODEL_P_PRE_C_11 174 /* p[174]: __pre__.c[11] */
#define MODEL_P_PRE_C_12 175 /* p[175]: __pre__.c[12] */
#define MODEL_P_PRE_C_13 176 /* p[176]: __pre__.c[13] */
#define MODEL_P_PRE_C_14 177 /* p[177]: __pre__.c[14] */
#define MODEL_P_PRE_C_15 178 /* p[178]: __pre__.c[15] */
#define MODEL_P_PRE_C_16 179 /* p[179]: __pre__.c[16] */
#define MODEL_P_PRE_C_17 180 /* p[180]: __pre__.c[17] */
#define MODEL_P_PRE_C_18 181 /* p[181]: __pre__.c[18] */
#define MODEL_P_PRE_C_19 182 /* p[182]: __pre__.c[19] */
#define MODEL_P_PRE_C_20 183 /* p[183]: __pre__.c[20] */
#define MODEL_P_PRE_C_21 184 /* p[184]: __pre__.c[21] */
#define MODEL_P_PRE_C_22 185 /* p[185]: __pre__.c[22] */
#define MODEL_P_PRE_C_23 186 /* p[186]: __pre__.c[23] */
#define MODEL_P_PRE_C_24 187 /* p[187]: __pre__.c[24] */
#define MODEL_P_PRE_C_25 188 /* p[188]: __pre__.c[25] */
#define MODEL_P_X 189 /* p[189]: x */
#define MODEL_P_Y 190 /* p[190]: y */
#define MODEL_P_Z 191 /* p[191]: z */
#define MODEL_P_ROLL 192 /* p[192]: roll */
#define MODEL_P_PITCH 193 /* p[193]: pitch */
#define MODEL_P_YAW 194 /* p[194]: yaw */
#define MODEL_P_AILERON 195 /* p[195]: aileron */
#define MODEL_P_ELEVATOR 196 /* p[196]: elevator */
#define MODEL_P_THROTTLE 197 /* p[197]: throttle */
#define MODEL_P_RUDDER 198 /* p[198]: rudder */
#define MODEL_P_STABILIZER 199 /* p[199]: stabilizer */
#define MODEL_P_DES_V 200 /* p[200]: des_v */
#define MODEL_P_DES_GAMMA 201 /* p[201]: des_gamma */
#define MODEL_P_DES_HEADING 202 /* p[202]: des_heading */
#define MODEL_P_DES_A 203 /* p[203]: des_a */
#define MODEL_P_CURRENT_WP 204 /* p[204]: current_wp */
#define MODEL_P_AIRBORNE 205 /* p[205]: airborne */
#define MODEL_P_PHI_CMD 206 /* p[206]: phi_cmd */
#define MODEL_P_CHI_ERR 207 /* p[207]: chi_err */
#define MODEL_P_X_EST 208 /* p[208]: x_est */
#define MODEL_P_Y_EST 209 /* p[209]: y_est */
#define MODEL_P_Z_EST 210 /* p[210]: z_est */
#define MODEL_P_ROLL_EST 211 /* p[211]: roll_est */
#define MODEL_P_PITCH_EST 212 /* p[212]: pitch_est */
#define MODEL_P_YAW_EST 213 /* p[213]: yaw_est */
#define MODEL_P_VX_EST 214 /* p[214]: vx_est */
#define MODEL_P_VY_EST 215 /* p[215]: vy_est */
#define MODEL_P_VZ_EST 216 /* p[216]: vz_est */
#define MODEL_P_V_EST 217 /* p[217]: v_est */
#define MODEL_P_GAMMA_EST 218 /* p[218]: gamma_est */
#define MODEL_P_VDOT_EST 219 /* p[219]: vdot_est */
#define MODEL_P_P_EST 220 /* p[220]: p_est */
#define MODEL_P_Q_EST 221 /* p[221]: q_est */
#define MODEL_P_R_EST 222 /* p[222]: r_est */
#define MODEL_P_PREV_X 223 /* p[223]: prev_x */
#define MODEL_P_PREV_Y 224 /* p[224]: prev_y */
#define MODEL_P_PREV_Z 225 /* p[225]: prev_z */
#define MODEL_P_PREV_ROLL 226 /* p[226]: prev_roll */
#define MODEL_P_PREV_PITCH 227 /* p[227]: prev_pitch */
#define MODEL_P_PREV_YAW 228 /* p[228]: prev_yaw */
#define MODEL_P_PREV_SPEED 229 /* p[229]: prev_speed */
#define MODEL_P_TIME_S 230 /* p[230]: time_s */
#define MODEL_P_TAKEOFF_TIME 231 /* p[231]: takeoff_time */
#define MODEL_P_ERR_NORM_ES_DOT_INT 232 /* p[232]: err_norm_es_dot_int */
#define MODEL_P_ERR_DIST_TERM_INT 233 /* p[233]: err_dist_term_int */
#define MODEL_P_ERR_PITCH_INT 234 /* p[234]: err_pitch_int */
#define MODEL_P_ERR_R_INT 235 /* p[235]: err_r_int */
#define MODEL_P_ERR_ROLL_INT 236 /* p[236]: err_roll_int */
#define MODEL_P_ERR_R_LAST 237 /* p[237]: err_r_last */
#define MODEL_P_PHI_CMD_STATE 238 /* p[238]: phi_cmd_state */
#define MODEL_P_ALPHA 239 /* p[239]: alpha */
#define MODEL_P_VX_NEW 240 /* p[240]: vx_new */
#define MODEL_P_VY_NEW 241 /* p[241]: vy_new */
#define MODEL_P_VZ_NEW 242 /* p[242]: vz_new */
#define MODEL_P_SPEED_NEW 243 /* p[243]: speed_new */
#define MODEL_P_P_NEW 244 /* p[244]: p_new */
#define MODEL_P_Q_NEW 245 /* p[245]: q_new */
#define MODEL_P_R_NEW 246 /* p[246]: r_new */
#define MODEL_P_GAMMA_NEW 247 /* p[247]: gamma_new */
#define MODEL_P_VDOT_NEW 248 /* p[248]: vdot_new */
#define MODEL_P_X_ERR 249 /* p[249]: x_err */
#define MODEL_P_Y_ERR 250 /* p[250]: y_err */
#define MODEL_P_Z_ERR 251 /* p[251]: z_err */
#define MODEL_P_HORZ_DIST_ERR 252 /* p[252]: horz_dist_err */
#define MODEL_P_PATH_VECT 253 /* p[253]: path_vect */
#define MODEL_P_PATH_VECT_LEN 3
#define MODEL_P_PATH_VECT_1 253 /* p[253]: path_vect[1] */
#define MODEL_P_PATH_VECT_2 254 /* p[254]: path_vect[2] */
#define MODEL_P_PATH_VECT_3 255 /* p[255]: path_vect[3] */
#define MODEL_P_PATH_LEN 256 /* p[256]: path_len */
#define MODEL_P_PATH_ANGLE 257 /* p[257]: path_angle */
#define MODEL_P_UNIT_ALONG_PATH 258 /* p[258]: unit_along_path */
#define MODEL_P_UNIT_ALONG_PATH_LEN 2
#define MODEL_P_UNIT_ALONG_PATH_1 258 /* p[258]: unit_along_path[1] */
#define MODEL_P_UNIT_ALONG_PATH_2 259 /* p[259]: unit_along_path[2] */
#define MODEL_P_UNIT_NORMAL 260 /* p[260]: unit_normal */
#define MODEL_P_UNIT_NORMAL_LEN 2
#define MODEL_P_UNIT_NORMAL_1 260 /* p[260]: unit_normal[1] */
#define MODEL_P_UNIT_NORMAL_2 261 /* p[261]: unit_normal[2] */
#define MODEL_P_POSE_VECT 262 /* p[262]: pose_vect */
#define MODEL_P_POSE_VECT_LEN 2
#define MODEL_P_POSE_VECT_1 262 /* p[262]: pose_vect[1] */
#define MODEL_P_POSE_VECT_2 263 /* p[263]: pose_vect[2] */
#define MODEL_P_ALONG_TRACK_ERR_W0 264 /* p[264]: along_track_err_w0 */
#define MODEL_P_ALONG_TRACK_ERR_W1 265 /* p[265]: along_track_err_w1 */
#define MODEL_P_CROSS_TRACK_ERR 266 /* p[266]: cross_track_err */
#define MODEL_P_LOOKAHEAD_NOM 267 /* p[267]: lookahead_nom */
#define MODEL_P_LOOKAHEAD_EFF 268 /* p[268]: lookahead_eff */
#define MODEL_P_SWITCH_THRESHOLD 269 /* p[269]: switch_threshold */
#define MODEL_P_NEXT_WX 270 /* p[270]: next_wx */
#define MODEL_P_NEXT_WY 271 /* p[271]: next_wy */
#define MODEL_P_NEXT_WZ 272 /* p[272]: next_wz */
#define MODEL_P_PREV_WX 273 /* p[273]: prev_wx */
#define MODEL_P_PREV_WY 274 /* p[274]: prev_wy */
#define MODEL_P_PREV_WZ 275 /* p[275]: prev_wz */
#define MODEL_P_REF_THRUST 276 /* p[276]: ref_thrust */
#define MODEL_P_REF_PITCH 277 /* p[277]: ref_pitch */
#define MODEL_P_R_V_DOT 278 /* p[278]: r_v_dot */
#define MODEL_P_WEIGHT 279 /* p[279]: weight */
#define MODEL_P_DRAG 280 /* p[280]: drag */
#define MODEL_P_ERR_NORM_ES_DOT 281 /* p[281]: err_norm_es_dot */
#define MODEL_P_ERR_DIST_TERM 282 /* p[282]: err_dist_term */
#define MODEL_P_ERR_PITCH 283 /* p[283]: err_pitch */
#define MODEL_P_Q_TURN 284 /* p[284]: q_turn */
#define MODEL_P_ERR_Q 285 /* p[285]: err_q */
#define MODEL_P_THRUST_UNSAT 286 /* p[286]: thrust_unsat */
#define MODEL_P_PITCH_UNSAT 287 /* p[287]: pitch_unsat */
#define MODEL_P_CHI 288 /* p[288]: chi */
#define MODEL_P_CHI_DOT_DES 289 /* p[289]: chi_dot_des */
#define MODEL_P_PHI_DES 290 /* p[290]: phi_des */
#define MODEL_P_DPHI_MAX 291 /* p[291]: dphi_max */
#define MODEL_P_ERR_YAW 292 /* p[292]: err_yaw */
#define MODEL_P_ERR_ROLL 293 /* p[293]: err_roll */
#define MODEL_P_ERR_R_DERIV 294 /* p[294]: err_r_deriv */
#define MODEL_P_NZ_EXCESS 295 /* p[295]: nz_excess */
#define MODEL_P_ELE_FF_PHI 296 /* p[296]: ele_ff_phi */
#define MODEL_P_C 297 /* p[297]: c */
#define MODEL_P_C_LEN 25
#define MODEL_P_C_1 297 /* p[297]: c[1] */
#define MODEL_P_C_2 298 /* p[298]: c[2] */
#define MODEL_P_C_3 299 /* p[299]: c[3] */
#define MODEL_P_C_4 300 /* p[300]: c[4] */
#define MODEL_P_C_5 301 /* p[301]: c[5] */
#define MODEL_P_C_6 302 /* p[302]: c[6] */
#define MODEL_P_C_7 303 /* p[303]: c[7] */
#define MODEL_P_C_8 304 /* p[304]: c[8] */
#define MODEL_P_C_9 305 /* p[305]: c[9] */
#define MODEL_P_C_10 306 /* p[306]: c[10] */
#define MODEL_P_C_11 307 /* p[307]: c[11] */
#define MODEL_P_C_12 308 /* p[308]: c[12] */
#define MODEL_P_C_13 309 /* p[309]: c[13] */
#define MODEL_P_C_14 310 /* p[310]: c[14] */
#define MODEL_P_C_15 311 /* p[311]: c[15] */
#define MODEL_P_C_16 312 /* p[312]: c[16] */
#define MODEL_P_C_17 313 /* p[313]: c[17] */
#define MODEL_P_C_18 314 /* p[314]: c[18] */
#define MODEL_P_C_19 315 /* p[315]: c[19] */
#define MODEL_P_C_20 316 /* p[316]: c[20] */
#define MODEL_P_C_21 317 /* p[317]: c[21] */
#define MODEL_P_C_22 318 /* p[318]: c[22] */
#define MODEL_P_C_23 319 /* p[319]: c[23] */
#define MODEL_P_C_24 320 /* p[320]: c[24] */
#define MODEL_P_C_25 321 /* p[321]: c[25] */
#define MODEL_P_RUMOCA_INITIAL_EVENT 322 /* p[322]: __rumoca.initial_event */

typedef enum {
    CUBCONTROL_FIXEDWINGOUTERLOOP_SLOT_Y = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_SLOT_P = 1
} CubControl_FixedWingOuterLoop_slot_kind_t;

typedef struct {
    const char *name;
    CubControl_FixedWingOuterLoop_slot_kind_t kind;
    int index;
    int length;
} CubControl_FixedWingOuterLoop_symbol_t;

extern const CubControl_FixedWingOuterLoop_symbol_t CubControl_FixedWingOuterLoop_symbols[];
extern const size_t CubControl_FixedWingOuterLoop_symbol_count;

typedef struct {
    real_t time;
    real_t y[1];
    real_t p[323];
    real_t event_indicators[25];
    real_t event_indicators_prev[25];
    real_t next_periodic_event[1];
} CubControl_FixedWingOuterLoop_t;

static inline real_t CubControl_FixedWingOuterLoop_get_y(const CubControl_FixedWingOuterLoop_t *m, int index) {
    return (m && index >= 0 && index < CUBCONTROL_FIXEDWINGOUTERLOOP_Y_LEN) ? m->y[index] : 0.0;
}

static inline real_t CubControl_FixedWingOuterLoop_get_p(const CubControl_FixedWingOuterLoop_t *m, int index) {
    return (m && index >= 0 && index < CUBCONTROL_FIXEDWINGOUTERLOOP_P_LEN) ? m->p[index] : 0.0;
}

static inline void CubControl_FixedWingOuterLoop_set_y(CubControl_FixedWingOuterLoop_t *m, int index, real_t value) {
    if (m && index >= 0 && index < CUBCONTROL_FIXEDWINGOUTERLOOP_Y_LEN) {
        m->y[index] = value;
    }
}

static inline void CubControl_FixedWingOuterLoop_set_p(CubControl_FixedWingOuterLoop_t *m, int index, real_t value) {
    if (m && index >= 0 && index < CUBCONTROL_FIXEDWINGOUTERLOOP_P_LEN) {
        m->p[index] = value;
    }
}

void startup(CubControl_FixedWingOuterLoop_t *m);
void dostep(CubControl_FixedWingOuterLoop_t *m, real_t dt);
void recalibrate(CubControl_FixedWingOuterLoop_t *m);
void CubControl_FixedWingOuterLoop_sync_pre(CubControl_FixedWingOuterLoop_t *m);