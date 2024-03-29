/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of:
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
#define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
#define _CASADI_NAMESPACE_CONCAT(NS, ID) NS##ID
#define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
#define CASADI_PREFIX(ID) rdd2_##ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_fabs CASADI_PREFIX(fabs)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

casadi_real casadi_sq(casadi_real x)
{
    return x * x;
}

casadi_real casadi_fabs(casadi_real x)
{
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
    return x > 0 ? x : -x;
#else
    return fabs(x);
#endif
}

static const casadi_int casadi_s0[8] = { 4, 1, 0, 4, 0, 1, 2, 3 };
static const casadi_int casadi_s1[5] = { 1, 1, 0, 1, 0 };
static const casadi_int casadi_s2[7] = { 3, 1, 0, 3, 0, 1, 2 };

/* attitude_error:(q[4],yaw_r,pitch_r,roll_r)->(omega[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem)
{
    w[0] = 2.;
    w[1] = arg[0] ? arg[0][0] : 0;
    w[2] = 0.;
    w[3] = arg[1] ? arg[1][0] : 0;
    w[4] = cos(w[3]);
    w[5] = arg[2] ? arg[2][0] : 0;
    w[6] = cos(w[5]);
    w[7] = (w[4] * w[6]);
    w[8] = cos(w[3]);
    w[9] = arg[3] ? arg[3][0] : 0;
    w[10] = cos(w[9]);
    w[11] = (w[8] * w[10]);
    w[12] = sin(w[3]);
    w[13] = sin(w[5]);
    w[14] = (w[12] * w[13]);
    w[15] = sin(w[9]);
    w[16] = (w[14] * w[15]);
    w[11] = (w[11] + w[16]);
    w[16] = (w[7] + w[11]);
    w[17] = cos(w[5]);
    w[18] = cos(w[9]);
    w[19] = (w[17] * w[18]);
    w[16] = (w[16] + w[19]);
    w[2] = (w[2] < w[16]);
    w[3] = sin(w[3]);
    w[9] = sin(w[9]);
    w[16] = (w[3] * w[9]);
    w[4] = (w[4] * w[13]);
    w[13] = (w[4] * w[18]);
    w[16] = (w[16] + w[13]);
    w[5] = sin(w[5]);
    w[13] = (w[16] + w[5]);
    w[20] = 4.;
    w[21] = 5.0000000000000000e-01;
    w[22] = 1.;
    w[23] = (w[22] + w[7]);
    w[23] = (w[23] + w[11]);
    w[23] = (w[23] + w[19]);
    w[23] = sqrt(w[23]);
    w[23] = (w[21] * w[23]);
    w[24] = (w[20] * w[23]);
    w[13] = (w[13] / w[24]);
    w[13] = (w[2] ? w[13] : 0);
    w[24] = (!w[2]);
    w[25] = (w[11] < w[7]);
    w[26] = (w[19] < w[7]);
    w[25] = (w[25] && w[26]);
    w[4] = (w[4] * w[15]);
    w[3] = (w[3] * w[10]);
    w[4] = (w[4] - w[3]);
    w[12] = (w[12] * w[6]);
    w[6] = (w[4] + w[12]);
    w[3] = (w[22] + w[7]);
    w[3] = (w[3] - w[11]);
    w[3] = (w[3] - w[19]);
    w[3] = sqrt(w[3]);
    w[3] = (w[21] * w[3]);
    w[10] = (w[20] * w[3]);
    w[6] = (w[6] / w[10]);
    w[6] = (w[25] ? w[6] : 0);
    w[10] = (!w[25]);
    w[26] = (w[19] < w[11]);
    w[27] = (w[22] - w[7]);
    w[27] = (w[27] + w[11]);
    w[27] = (w[27] - w[19]);
    w[27] = sqrt(w[27]);
    w[27] = (w[21] * w[27]);
    w[28] = (w[26] ? w[27] : 0);
    w[29] = (!w[26]);
    w[14] = (w[14] * w[18]);
    w[8] = (w[8] * w[9]);
    w[14] = (w[14] - w[8]);
    w[17] = (w[17] * w[15]);
    w[15] = (w[14] + w[17]);
    w[7] = (w[22] - w[7]);
    w[7] = (w[7] - w[11]);
    w[7] = (w[7] + w[19]);
    w[7] = sqrt(w[7]);
    w[21] = (w[21] * w[7]);
    w[7] = (w[20] * w[21]);
    w[15] = (w[15] / w[7]);
    w[15] = (w[29] ? w[15] : 0);
    w[28] = (w[28] + w[15]);
    w[28] = (w[10] ? w[28] : 0);
    w[6] = (w[6] + w[28]);
    w[6] = (w[24] ? w[6] : 0);
    w[13] = (w[13] + w[6]);
    w[6] = (w[1] * w[13]);
    w[28] = arg[0] ? arg[0][2] : 0;
    w[15] = (w[2] ? w[23] : 0);
    w[7] = (w[17] - w[14]);
    w[19] = (w[20] * w[3]);
    w[7] = (w[7] / w[19]);
    w[7] = (w[25] ? w[7] : 0);
    w[19] = (w[16] + w[5]);
    w[11] = (w[20] * w[27]);
    w[19] = (w[19] / w[11]);
    w[19] = (w[26] ? w[19] : 0);
    w[11] = (w[12] - w[4]);
    w[8] = (w[20] * w[21]);
    w[11] = (w[11] / w[8]);
    w[11] = (w[29] ? w[11] : 0);
    w[19] = (w[19] + w[11]);
    w[19] = (w[10] ? w[19] : 0);
    w[7] = (w[7] + w[19]);
    w[7] = (w[24] ? w[7] : 0);
    w[15] = (w[15] + w[7]);
    w[7] = (w[28] * w[15]);
    w[19] = arg[0] ? arg[0][3] : 0;
    w[11] = (w[17] - w[14]);
    w[8] = (w[20] * w[23]);
    w[11] = (w[11] / w[8]);
    w[11] = (w[2] ? w[11] : 0);
    w[8] = (w[25] ? w[3] : 0);
    w[9] = (w[4] + w[12]);
    w[18] = (w[20] * w[27]);
    w[9] = (w[9] / w[18]);
    w[9] = (w[26] ? w[9] : 0);
    w[18] = (w[16] - w[5]);
    w[30] = (w[20] * w[21]);
    w[18] = (w[18] / w[30]);
    w[18] = (w[29] ? w[18] : 0);
    w[9] = (w[9] + w[18]);
    w[9] = (w[10] ? w[9] : 0);
    w[8] = (w[8] + w[9]);
    w[8] = (w[24] ? w[8] : 0);
    w[11] = (w[11] + w[8]);
    w[8] = (w[19] * w[11]);
    w[7] = (w[7] + w[8]);
    w[6] = (w[6] - w[7]);
    w[7] = arg[0] ? arg[0][1] : 0;
    w[12] = (w[12] - w[4]);
    w[23] = (w[20] * w[23]);
    w[12] = (w[12] / w[23]);
    w[2] = (w[2] ? w[12] : 0);
    w[16] = (w[16] - w[5]);
    w[3] = (w[20] * w[3]);
    w[16] = (w[16] / w[3]);
    w[25] = (w[25] ? w[16] : 0);
    w[14] = (w[14] + w[17]);
    w[20] = (w[20] * w[27]);
    w[14] = (w[14] / w[20]);
    w[26] = (w[26] ? w[14] : 0);
    w[29] = (w[29] ? w[21] : 0);
    w[26] = (w[26] + w[29]);
    w[10] = (w[10] ? w[26] : 0);
    w[25] = (w[25] + w[10]);
    w[24] = (w[24] ? w[25] : 0);
    w[2] = (w[2] + w[24]);
    w[24] = (w[7] * w[2]);
    w[6] = (w[6] + w[24]);
    w[24] = (w[28] * w[11]);
    w[25] = (w[19] * w[15]);
    w[24] = (w[24] - w[25]);
    w[25] = (w[7] * w[13]);
    w[24] = (w[24] - w[25]);
    w[25] = (w[1] * w[2]);
    w[24] = (w[24] + w[25]);
    w[25] = (w[6] * w[24]);
    w[10] = (w[1] * w[15]);
    w[26] = (w[7] * w[11]);
    w[10] = (w[10] + w[26]);
    w[26] = (w[28] * w[13]);
    w[10] = (w[10] + w[26]);
    w[26] = (w[19] * w[2]);
    w[10] = (w[10] + w[26]);
    w[1] = (w[1] * w[11]);
    w[7] = (w[7] * w[15]);
    w[1] = (w[1] - w[7]);
    w[19] = (w[19] * w[13]);
    w[1] = (w[1] + w[19]);
    w[28] = (w[28] * w[2]);
    w[1] = (w[1] - w[28]);
    w[28] = (w[10] * w[1]);
    w[2] = (w[25] + w[28]);
    w[2] = (w[0] * w[2]);
    w[25] = (w[25] - w[28]);
    w[25] = (w[0] * w[25]);
    w[2] = (w[2] - w[25]);
    w[25] = casadi_sq(w[10]);
    w[28] = casadi_sq(w[1]);
    w[19] = (w[25] + w[28]);
    w[13] = casadi_sq(w[6]);
    w[19] = (w[19] - w[13]);
    w[7] = casadi_sq(w[24]);
    w[19] = (w[19] - w[7]);
    w[15] = (w[25] + w[13]);
    w[15] = (w[15] - w[28]);
    w[15] = (w[15] - w[7]);
    w[19] = (w[19] + w[15]);
    w[25] = (w[25] + w[7]);
    w[25] = (w[25] - w[28]);
    w[25] = (w[25] - w[13]);
    w[19] = (w[19] + w[25]);
    w[19] = (w[19] - w[22]);
    w[19] = (w[19] / w[0]);
    w[19] = acos(w[19]);
    w[25] = casadi_fabs(w[19]);
    w[13] = 9.9999999999999995e-08;
    w[25] = (w[25] < w[13]);
    w[13] = -1.6666666666666666e-01;
    w[28] = casadi_sq(w[19]);
    w[13] = (w[13] * w[28]);
    w[22] = (w[22] + w[13]);
    w[13] = 8.3333333333333332e-03;
    w[28] = casadi_sq(w[19]);
    w[28] = casadi_sq(w[28]);
    w[13] = (w[13] * w[28]);
    w[22] = (w[22] + w[13]);
    w[22] = (w[25] ? w[22] : 0);
    w[25] = (!w[25]);
    w[13] = sin(w[19]);
    w[13] = (w[13] / w[19]);
    w[25] = (w[25] ? w[13] : 0);
    w[22] = (w[22] + w[25]);
    w[22] = (w[0] * w[22]);
    w[2] = (w[2] / w[22]);
    if (res[0] != 0)
        res[0][0] = w[2];
    w[2] = (w[1] * w[24]);
    w[25] = (w[10] * w[6]);
    w[13] = (w[2] + w[25]);
    w[13] = (w[0] * w[13]);
    w[2] = (w[2] - w[25]);
    w[2] = (w[0] * w[2]);
    w[13] = (w[13] - w[2]);
    w[13] = (w[13] / w[22]);
    if (res[0] != 0)
        res[0][1] = w[13];
    w[1] = (w[1] * w[6]);
    w[10] = (w[10] * w[24]);
    w[24] = (w[1] + w[10]);
    w[24] = (w[0] * w[24]);
    w[1] = (w[1] - w[10]);
    w[0] = (w[0] * w[1]);
    w[24] = (w[24] - w[0]);
    w[24] = (w[24] / w[22]);
    if (res[0] != 0)
        res[0][2] = w[24];
    return 0;
}

int attitude_error(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem)
{
    return casadi_f0(arg, res, iw, w, mem);
}

int attitude_error_alloc_mem(void)
{
    return 0;
}

int attitude_error_init_mem(int mem)
{
    return 0;
}

void attitude_error_free_mem(int mem)
{
}

int attitude_error_checkout(void)
{
    return 0;
}

void attitude_error_release(int mem)
{
}

void attitude_error_incref(void)
{
}

void attitude_error_decref(void)
{
}

casadi_int attitude_error_n_in(void) { return 4; }

casadi_int attitude_error_n_out(void) { return 1; }

casadi_real attitude_error_default_in(casadi_int i)
{
    switch (i) {
    default:
        return 0;
    }
}

const char* attitude_error_name_in(casadi_int i)
{
    switch (i) {
    case 0:
        return "q";
    case 1:
        return "yaw_r";
    case 2:
        return "pitch_r";
    case 3:
        return "roll_r";
    default:
        return 0;
    }
}

const char* attitude_error_name_out(casadi_int i)
{
    switch (i) {
    case 0:
        return "omega";
    default:
        return 0;
    }
}

const casadi_int* attitude_error_sparsity_in(casadi_int i)
{
    switch (i) {
    case 0:
        return casadi_s0;
    case 1:
        return casadi_s1;
    case 2:
        return casadi_s1;
    case 3:
        return casadi_s1;
    default:
        return 0;
    }
}

const casadi_int* attitude_error_sparsity_out(casadi_int i)
{
    switch (i) {
    case 0:
        return casadi_s2;
    default:
        return 0;
    }
}

int attitude_error_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w)
{
    if (sz_arg)
        *sz_arg = 4;
    if (sz_res)
        *sz_res = 1;
    if (sz_iw)
        *sz_iw = 0;
    if (sz_w)
        *sz_w = 31;
    return 0;
}

/* quaternion_to_euler:(q[4])->(e[3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem)
{
    w[0] = 2.;
    w[1] = arg[0] ? arg[0][1] : 0;
    w[2] = arg[0] ? arg[0][3] : 0;
    w[3] = (w[1] * w[2]);
    w[4] = arg[0] ? arg[0][0] : 0;
    w[5] = arg[0] ? arg[0][2] : 0;
    w[6] = (w[4] * w[5]);
    w[7] = (w[3] - w[6]);
    w[7] = (w[0] * w[7]);
    w[7] = (-w[7]);
    w[7] = asin(w[7]);
    w[8] = 1.5707963267948966e+00;
    w[9] = (w[7] - w[8]);
    w[9] = casadi_fabs(w[9]);
    w[10] = 1.0000000000000000e-03;
    w[9] = (w[9] < w[10]);
    w[11] = (w[5] * w[2]);
    w[12] = (w[4] * w[1]);
    w[13] = (w[11] - w[12]);
    w[13] = (w[0] * w[13]);
    w[3] = (w[3] + w[6]);
    w[3] = (w[0] * w[3]);
    w[6] = atan2(w[13], w[3]);
    w[6] = (w[9] ? w[6] : 0);
    w[14] = (!w[9]);
    w[8] = (w[7] + w[8]);
    w[8] = casadi_fabs(w[8]);
    w[8] = (w[8] < w[10]);
    w[13] = (-w[13]);
    w[3] = (-w[3]);
    w[13] = atan2(w[13], w[3]);
    w[13] = (w[8] ? w[13] : 0);
    w[3] = (!w[8]);
    w[10] = (w[1] * w[5]);
    w[15] = (w[4] * w[2]);
    w[10] = (w[10] + w[15]);
    w[10] = (w[0] * w[10]);
    w[4] = casadi_sq(w[4]);
    w[1] = casadi_sq(w[1]);
    w[15] = (w[4] + w[1]);
    w[5] = casadi_sq(w[5]);
    w[15] = (w[15] - w[5]);
    w[2] = casadi_sq(w[2]);
    w[15] = (w[15] - w[2]);
    w[10] = atan2(w[10], w[15]);
    w[10] = (w[3] ? w[10] : 0);
    w[13] = (w[13] + w[10]);
    w[13] = (w[14] ? w[13] : 0);
    w[6] = (w[6] + w[13]);
    if (res[0] != 0)
        res[0][0] = w[6];
    w[9] = (w[9] ? w[7] : 0);
    w[8] = (w[8] ? w[7] : 0);
    w[7] = (w[3] ? w[7] : 0);
    w[8] = (w[8] + w[7]);
    w[8] = (w[14] ? w[8] : 0);
    w[9] = (w[9] + w[8]);
    if (res[0] != 0)
        res[0][1] = w[9];
    w[11] = (w[11] + w[12]);
    w[0] = (w[0] * w[11]);
    w[4] = (w[4] + w[2]);
    w[4] = (w[4] - w[1]);
    w[4] = (w[4] - w[5]);
    w[0] = atan2(w[0], w[4]);
    w[3] = (w[3] ? w[0] : 0);
    w[14] = (w[14] ? w[3] : 0);
    if (res[0] != 0)
        res[0][2] = w[14];
    return 0;
}

int quaternion_to_euler(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem)
{
    return casadi_f1(arg, res, iw, w, mem);
}

int quaternion_to_euler_alloc_mem(void)
{
    return 0;
}

int quaternion_to_euler_init_mem(int mem)
{
    return 0;
}

void quaternion_to_euler_free_mem(int mem)
{
}

int quaternion_to_euler_checkout(void)
{
    return 0;
}

void quaternion_to_euler_release(int mem)
{
}

void quaternion_to_euler_incref(void)
{
}

void quaternion_to_euler_decref(void)
{
}

casadi_int quaternion_to_euler_n_in(void) { return 1; }

casadi_int quaternion_to_euler_n_out(void) { return 1; }

casadi_real quaternion_to_euler_default_in(casadi_int i)
{
    switch (i) {
    default:
        return 0;
    }
}

const char* quaternion_to_euler_name_in(casadi_int i)
{
    switch (i) {
    case 0:
        return "q";
    default:
        return 0;
    }
}

const char* quaternion_to_euler_name_out(casadi_int i)
{
    switch (i) {
    case 0:
        return "e";
    default:
        return 0;
    }
}

const casadi_int* quaternion_to_euler_sparsity_in(casadi_int i)
{
    switch (i) {
    case 0:
        return casadi_s0;
    default:
        return 0;
    }
}

const casadi_int* quaternion_to_euler_sparsity_out(casadi_int i)
{
    switch (i) {
    case 0:
        return casadi_s2;
    default:
        return 0;
    }
}

int quaternion_to_euler_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w)
{
    if (sz_arg)
        *sz_arg = 1;
    if (sz_res)
        *sz_res = 1;
    if (sz_iw)
        *sz_iw = 0;
    if (sz_w)
        *sz_w = 16;
    return 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
