/* This file was automatically generated by CasADi 3.6.5.
 *  It consists of:
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int butterworth_2_filter(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int butterworth_2_filter_alloc_mem(void);
int butterworth_2_filter_init_mem(int mem);
void butterworth_2_filter_free_mem(int mem);
int butterworth_2_filter_checkout(void);
void butterworth_2_filter_release(int mem);
void butterworth_2_filter_incref(void);
void butterworth_2_filter_decref(void);
casadi_int butterworth_2_filter_n_in(void);
casadi_int butterworth_2_filter_n_out(void);
casadi_real butterworth_2_filter_default_in(casadi_int i);
const char* butterworth_2_filter_name_in(casadi_int i);
const char* butterworth_2_filter_name_out(casadi_int i);
const casadi_int* butterworth_2_filter_sparsity_in(casadi_int i);
const casadi_int* butterworth_2_filter_sparsity_out(casadi_int i);
int butterworth_2_filter_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
int butterworth_2_filter_work_bytes(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
#define butterworth_2_filter_SZ_ARG 4
#define butterworth_2_filter_SZ_RES 2
#define butterworth_2_filter_SZ_IW 0
#define butterworth_2_filter_SZ_W 12
int eulerB321_to_quat(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int eulerB321_to_quat_alloc_mem(void);
int eulerB321_to_quat_init_mem(int mem);
void eulerB321_to_quat_free_mem(int mem);
int eulerB321_to_quat_checkout(void);
void eulerB321_to_quat_release(int mem);
void eulerB321_to_quat_incref(void);
void eulerB321_to_quat_decref(void);
casadi_int eulerB321_to_quat_n_in(void);
casadi_int eulerB321_to_quat_n_out(void);
casadi_real eulerB321_to_quat_default_in(casadi_int i);
const char* eulerB321_to_quat_name_in(casadi_int i);
const char* eulerB321_to_quat_name_out(casadi_int i);
const casadi_int* eulerB321_to_quat_sparsity_in(casadi_int i);
const casadi_int* eulerB321_to_quat_sparsity_out(casadi_int i);
int eulerB321_to_quat_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
int eulerB321_to_quat_work_bytes(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
#define eulerB321_to_quat_SZ_ARG 3
#define eulerB321_to_quat_SZ_RES 1
#define eulerB321_to_quat_SZ_IW 0
#define eulerB321_to_quat_SZ_W 28
int quat_to_eulerB321(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int quat_to_eulerB321_alloc_mem(void);
int quat_to_eulerB321_init_mem(int mem);
void quat_to_eulerB321_free_mem(int mem);
int quat_to_eulerB321_checkout(void);
void quat_to_eulerB321_release(int mem);
void quat_to_eulerB321_incref(void);
void quat_to_eulerB321_decref(void);
casadi_int quat_to_eulerB321_n_in(void);
casadi_int quat_to_eulerB321_n_out(void);
casadi_real quat_to_eulerB321_default_in(casadi_int i);
const char* quat_to_eulerB321_name_in(casadi_int i);
const char* quat_to_eulerB321_name_out(casadi_int i);
const casadi_int* quat_to_eulerB321_sparsity_in(casadi_int i);
const casadi_int* quat_to_eulerB321_sparsity_out(casadi_int i);
int quat_to_eulerB321_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
int quat_to_eulerB321_work_bytes(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
#define quat_to_eulerB321_SZ_ARG 1
#define quat_to_eulerB321_SZ_RES 3
#define quat_to_eulerB321_SZ_IW 0
#define quat_to_eulerB321_SZ_W 16
#ifdef __cplusplus
} /* extern "C" */
#endif