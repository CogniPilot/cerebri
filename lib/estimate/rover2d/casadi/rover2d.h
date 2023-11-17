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

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int predict(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int predict_alloc_mem(void);
int predict_init_mem(int mem);
void predict_free_mem(int mem);
int predict_checkout(void);
void predict_release(int mem);
void predict_incref(void);
void predict_decref(void);
casadi_int predict_n_in(void);
casadi_int predict_n_out(void);
casadi_real predict_default_in(casadi_int i);
const char* predict_name_in(casadi_int i);
const char* predict_name_out(casadi_int i);
const casadi_int* predict_sparsity_in(casadi_int i);
const casadi_int* predict_sparsity_out(casadi_int i);
int predict_work(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
#define predict_SZ_ARG 3
#define predict_SZ_RES 1
#define predict_SZ_IW 0
#define predict_SZ_W 12
#ifdef __cplusplus
} /* extern "C" */
#endif
