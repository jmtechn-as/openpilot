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
void live_H(double *in_vec, double *out_2503166590185590312);
void live_err_fun(double *nom_x, double *delta_x, double *out_3794793126910250149);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5930455572415847170);
void live_H_mod_fun(double *state, double *out_9124384806770571381);
void live_f_fun(double *state, double dt, double *out_9188504082434773062);
void live_F_fun(double *state, double dt, double *out_5766622881826669580);
void live_h_4(double *state, double *unused, double *out_8885787855636796251);
void live_H_4(double *state, double *unused, double *out_463036728120118219);
void live_h_9(double *state, double *unused, double *out_3991381877534412098);
void live_H_9(double *state, double *unused, double *out_221847081490527574);
void live_h_10(double *state, double *unused, double *out_4759164273606619881);
void live_H_10(double *state, double *unused, double *out_977931642135124192);
void live_h_12(double *state, double *unused, double *out_4727203131249792018);
void live_H_12(double *state, double *unused, double *out_4556419679911843576);
void live_h_31(double *state, double *unused, double *out_6081235033987281596);
void live_H_31(double *state, double *unused, double *out_7301982712236857285);
void live_h_32(double *state, double *unused, double *out_760862035415444302);
void live_H_32(double *state, double *unused, double *out_4011239174441627199);
void live_h_13(double *state, double *unused, double *out_4917359291892214746);
void live_H_13(double *state, double *unused, double *out_9213972146494475321);
void live_h_14(double *state, double *unused, double *out_3991381877534412098);
void live_H_14(double *state, double *unused, double *out_221847081490527574);
void live_h_33(double *state, double *unused, double *out_5791450238291229571);
void live_H_33(double *state, double *unused, double *out_6054182333891346761);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}