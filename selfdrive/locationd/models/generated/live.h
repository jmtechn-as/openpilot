#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_360501912147953484);
void live_err_fun(double *nom_x, double *delta_x, double *out_3740584223096575636);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5064167050965321181);
void live_H_mod_fun(double *state, double *out_1453781278869036201);
void live_f_fun(double *state, double dt, double *out_3751384202408520747);
void live_F_fun(double *state, double dt, double *out_5155198399062426300);
void live_h_4(double *state, double *unused, double *out_5195838647191847946);
void live_H_4(double *state, double *unused, double *out_7955051523998970739);
void live_h_9(double *state, double *unused, double *out_8458312080195228661);
void live_H_9(double *state, double *unused, double *out_7713861877369380094);
void live_h_10(double *state, double *unused, double *out_237332352829652004);
void live_H_10(double *state, double *unused, double *out_3612149683525720665);
void live_h_12(double *state, double *unused, double *out_1947367096166371646);
void live_H_12(double *state, double *unused, double *out_2935595115967008944);
void live_h_31(double *state, double *unused, double *out_1876180537666478243);
void live_H_31(double *state, double *unused, double *out_4588389466626363363);
void live_h_32(double *state, double *unused, double *out_8866808176599821212);
void live_H_32(double *state, double *unused, double *out_6943490103389071897);
void live_h_13(double *state, double *unused, double *out_4441742876173568425);
void live_H_13(double *state, double *unused, double *out_796798030774768568);
void live_h_14(double *state, double *unused, double *out_8458312080195228661);
void live_H_14(double *state, double *unused, double *out_7713861877369380094);
void live_h_33(double *state, double *unused, double *out_2655887984277546659);
void live_H_33(double *state, double *unused, double *out_1437832461987505759);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}