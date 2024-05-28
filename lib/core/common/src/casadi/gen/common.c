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

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) common_ ## ID
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
#define casadi_sq CASADI_PREFIX(sq)

casadi_real casadi_fabs(casadi_real x) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>0 ? x : -x;
#else
  return fabs(x);
#endif
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};

/* eulerB321_to_quat:(yaw,pitch,roll)->(q[4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  w[0]=0.;
  w[1]=arg[0]? arg[0][0] : 0;
  w[2]=cos(w[1]);
  w[3]=arg[1]? arg[1][0] : 0;
  w[4]=cos(w[3]);
  w[5]=(w[2]*w[4]);
  w[6]=cos(w[1]);
  w[7]=arg[2]? arg[2][0] : 0;
  w[8]=cos(w[7]);
  w[9]=(w[6]*w[8]);
  w[10]=sin(w[1]);
  w[11]=sin(w[3]);
  w[12]=(w[10]*w[11]);
  w[13]=sin(w[7]);
  w[14]=(w[12]*w[13]);
  w[9]=(w[9]+w[14]);
  w[14]=(w[5]+w[9]);
  w[15]=cos(w[3]);
  w[16]=cos(w[7]);
  w[17]=(w[15]*w[16]);
  w[14]=(w[14]+w[17]);
  w[0]=(w[0]<w[14]);
  w[14]=5.0000000000000000e-01;
  w[18]=1.;
  w[19]=(w[18]+w[5]);
  w[19]=(w[19]+w[9]);
  w[19]=(w[19]+w[17]);
  w[19]=sqrt(w[19]);
  w[19]=(w[14]*w[19]);
  w[20]=(w[0]?w[19]:0);
  w[21]=(!w[0]);
  w[22]=(w[9]<w[5]);
  w[23]=(w[17]<w[5]);
  w[22]=(w[22]&&w[23]);
  w[15]=(w[15]*w[13]);
  w[12]=(w[12]*w[16]);
  w[7]=sin(w[7]);
  w[6]=(w[6]*w[7]);
  w[12]=(w[12]-w[6]);
  w[6]=(w[15]-w[12]);
  w[23]=4.;
  w[24]=(w[18]+w[5]);
  w[24]=(w[24]-w[9]);
  w[24]=(w[24]-w[17]);
  w[24]=sqrt(w[24]);
  w[24]=(w[14]*w[24]);
  w[25]=(w[23]*w[24]);
  w[6]=(w[6]/w[25]);
  w[6]=(w[22]?w[6]:0);
  w[25]=(!w[22]);
  w[26]=(w[17]<w[9]);
  w[1]=sin(w[1]);
  w[7]=(w[1]*w[7]);
  w[2]=(w[2]*w[11]);
  w[16]=(w[2]*w[16]);
  w[7]=(w[7]+w[16]);
  w[3]=sin(w[3]);
  w[16]=(w[7]+w[3]);
  w[11]=(w[18]-w[5]);
  w[11]=(w[11]+w[9]);
  w[11]=(w[11]-w[17]);
  w[11]=sqrt(w[11]);
  w[11]=(w[14]*w[11]);
  w[27]=(w[23]*w[11]);
  w[16]=(w[16]/w[27]);
  w[16]=(w[26]?w[16]:0);
  w[27]=(!w[26]);
  w[10]=(w[10]*w[4]);
  w[2]=(w[2]*w[13]);
  w[1]=(w[1]*w[8]);
  w[2]=(w[2]-w[1]);
  w[1]=(w[10]-w[2]);
  w[18]=(w[18]-w[5]);
  w[18]=(w[18]-w[9]);
  w[18]=(w[18]+w[17]);
  w[18]=sqrt(w[18]);
  w[14]=(w[14]*w[18]);
  w[18]=(w[23]*w[14]);
  w[1]=(w[1]/w[18]);
  w[1]=(w[27]?w[1]:0);
  w[16]=(w[16]+w[1]);
  w[16]=(w[25]?w[16]:0);
  w[6]=(w[6]+w[16]);
  w[6]=(w[21]?w[6]:0);
  w[20]=(w[20]+w[6]);
  if (res[0]!=0) res[0][0]=w[20];
  w[20]=(w[15]-w[12]);
  w[6]=(w[23]*w[19]);
  w[20]=(w[20]/w[6]);
  w[20]=(w[0]?w[20]:0);
  w[6]=(w[22]?w[24]:0);
  w[16]=(w[2]+w[10]);
  w[1]=(w[23]*w[11]);
  w[16]=(w[16]/w[1]);
  w[16]=(w[26]?w[16]:0);
  w[1]=(w[7]-w[3]);
  w[18]=(w[23]*w[14]);
  w[1]=(w[1]/w[18]);
  w[1]=(w[27]?w[1]:0);
  w[16]=(w[16]+w[1]);
  w[16]=(w[25]?w[16]:0);
  w[6]=(w[6]+w[16]);
  w[6]=(w[21]?w[6]:0);
  w[20]=(w[20]+w[6]);
  if (res[0]!=0) res[0][1]=w[20];
  w[20]=(w[7]+w[3]);
  w[6]=(w[23]*w[19]);
  w[20]=(w[20]/w[6]);
  w[20]=(w[0]?w[20]:0);
  w[6]=(w[2]+w[10]);
  w[16]=(w[23]*w[24]);
  w[6]=(w[6]/w[16]);
  w[6]=(w[22]?w[6]:0);
  w[16]=(w[26]?w[11]:0);
  w[1]=(w[12]+w[15]);
  w[18]=(w[23]*w[14]);
  w[1]=(w[1]/w[18]);
  w[1]=(w[27]?w[1]:0);
  w[16]=(w[16]+w[1]);
  w[16]=(w[25]?w[16]:0);
  w[6]=(w[6]+w[16]);
  w[6]=(w[21]?w[6]:0);
  w[20]=(w[20]+w[6]);
  if (res[0]!=0) res[0][2]=w[20];
  w[10]=(w[10]-w[2]);
  w[19]=(w[23]*w[19]);
  w[10]=(w[10]/w[19]);
  w[0]=(w[0]?w[10]:0);
  w[7]=(w[7]-w[3]);
  w[24]=(w[23]*w[24]);
  w[7]=(w[7]/w[24]);
  w[22]=(w[22]?w[7]:0);
  w[12]=(w[12]+w[15]);
  w[23]=(w[23]*w[11]);
  w[12]=(w[12]/w[23]);
  w[26]=(w[26]?w[12]:0);
  w[27]=(w[27]?w[14]:0);
  w[26]=(w[26]+w[27]);
  w[25]=(w[25]?w[26]:0);
  w[22]=(w[22]+w[25]);
  w[21]=(w[21]?w[22]:0);
  w[0]=(w[0]+w[21]);
  if (res[0]!=0) res[0][3]=w[0];
  return 0;
}

int eulerB321_to_quat(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

int eulerB321_to_quat_alloc_mem(void) {
  return 0;
}

int eulerB321_to_quat_init_mem(int mem) {
  return 0;
}

void eulerB321_to_quat_free_mem(int mem) {
}

int eulerB321_to_quat_checkout(void) {
  return 0;
}

void eulerB321_to_quat_release(int mem) {
}

void eulerB321_to_quat_incref(void) {
}

void eulerB321_to_quat_decref(void) {
}

casadi_int eulerB321_to_quat_n_in(void) { return 3;}

casadi_int eulerB321_to_quat_n_out(void) { return 1;}

casadi_real eulerB321_to_quat_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* eulerB321_to_quat_name_in(casadi_int i) {
  switch (i) {
    case 0: return "yaw";
    case 1: return "pitch";
    case 2: return "roll";
    default: return 0;
  }
}

const char* eulerB321_to_quat_name_out(casadi_int i) {
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

const casadi_int* eulerB321_to_quat_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    default: return 0;
  }
}

const casadi_int* eulerB321_to_quat_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

int eulerB321_to_quat_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 28;
  return 0;
}

int eulerB321_to_quat_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 28*sizeof(casadi_real);
  return 0;
}

/* quat_to_eulerB321:(q_wb[4])->(yaw,pitch,roll) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  w[0]=2.;
  w[1]=arg[0]? arg[0][1] : 0;
  w[2]=arg[0]? arg[0][3] : 0;
  w[3]=(w[1]*w[2]);
  w[4]=arg[0]? arg[0][0] : 0;
  w[5]=arg[0]? arg[0][2] : 0;
  w[6]=(w[4]*w[5]);
  w[7]=(w[3]-w[6]);
  w[7]=(w[0]*w[7]);
  w[7]=(-w[7]);
  w[7]=asin(w[7]);
  w[8]=1.5707963267948966e+00;
  w[9]=(w[7]-w[8]);
  w[9]=casadi_fabs(w[9]);
  w[10]=1.0000000000000000e-03;
  w[9]=(w[9]<w[10]);
  w[11]=(w[5]*w[2]);
  w[12]=(w[4]*w[1]);
  w[13]=(w[11]-w[12]);
  w[13]=(w[0]*w[13]);
  w[3]=(w[3]+w[6]);
  w[3]=(w[0]*w[3]);
  w[6]=atan2(w[13],w[3]);
  w[6]=(w[9]?w[6]:0);
  w[14]=(!w[9]);
  w[8]=(w[7]+w[8]);
  w[8]=casadi_fabs(w[8]);
  w[8]=(w[8]<w[10]);
  w[13]=(-w[13]);
  w[3]=(-w[3]);
  w[13]=atan2(w[13],w[3]);
  w[13]=(w[8]?w[13]:0);
  w[3]=(!w[8]);
  w[10]=(w[1]*w[5]);
  w[15]=(w[4]*w[2]);
  w[10]=(w[10]+w[15]);
  w[10]=(w[0]*w[10]);
  w[4]=casadi_sq(w[4]);
  w[1]=casadi_sq(w[1]);
  w[15]=(w[4]+w[1]);
  w[5]=casadi_sq(w[5]);
  w[15]=(w[15]-w[5]);
  w[2]=casadi_sq(w[2]);
  w[15]=(w[15]-w[2]);
  w[10]=atan2(w[10],w[15]);
  w[10]=(w[3]?w[10]:0);
  w[13]=(w[13]+w[10]);
  w[13]=(w[14]?w[13]:0);
  w[6]=(w[6]+w[13]);
  if (res[0]!=0) res[0][0]=w[6];
  w[9]=(w[9]?w[7]:0);
  w[8]=(w[8]?w[7]:0);
  w[7]=(w[3]?w[7]:0);
  w[8]=(w[8]+w[7]);
  w[8]=(w[14]?w[8]:0);
  w[9]=(w[9]+w[8]);
  if (res[1]!=0) res[1][0]=w[9];
  w[11]=(w[11]+w[12]);
  w[0]=(w[0]*w[11]);
  w[4]=(w[4]+w[2]);
  w[4]=(w[4]-w[1]);
  w[4]=(w[4]-w[5]);
  w[0]=atan2(w[0],w[4]);
  w[3]=(w[3]?w[0]:0);
  w[14]=(w[14]?w[3]:0);
  if (res[2]!=0) res[2][0]=w[14];
  return 0;
}

int quat_to_eulerB321(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

int quat_to_eulerB321_alloc_mem(void) {
  return 0;
}

int quat_to_eulerB321_init_mem(int mem) {
  return 0;
}

void quat_to_eulerB321_free_mem(int mem) {
}

int quat_to_eulerB321_checkout(void) {
  return 0;
}

void quat_to_eulerB321_release(int mem) {
}

void quat_to_eulerB321_incref(void) {
}

void quat_to_eulerB321_decref(void) {
}

casadi_int quat_to_eulerB321_n_in(void) { return 1;}

casadi_int quat_to_eulerB321_n_out(void) { return 3;}

casadi_real quat_to_eulerB321_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* quat_to_eulerB321_name_in(casadi_int i) {
  switch (i) {
    case 0: return "q_wb";
    default: return 0;
  }
}

const char* quat_to_eulerB321_name_out(casadi_int i) {
  switch (i) {
    case 0: return "yaw";
    case 1: return "pitch";
    case 2: return "roll";
    default: return 0;
  }
}

const casadi_int* quat_to_eulerB321_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* quat_to_eulerB321_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    default: return 0;
  }
}

int quat_to_eulerB321_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 16;
  return 0;
}

int quat_to_eulerB321_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 3*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 16*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
