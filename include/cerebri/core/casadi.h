#ifndef CEREBRI_CORE_CASADI_H
#define CEREBRI_CORE_CASADI_H

#define CASADI_FUNC_ARGS(name)              \
    casadi_int iw[name##_SZ_IW];            \
    casadi_real w[name##_SZ_W];             \
    const casadi_real* args[name##_SZ_ARG]; \
    casadi_real* res[name##_SZ_RES];        \
    int mem = 0;

#define CASADI_FUNC_CALL(name) \
    name(args, res, iw, w, mem);

#endif // CEREBRI_CORE_CASADI_H
