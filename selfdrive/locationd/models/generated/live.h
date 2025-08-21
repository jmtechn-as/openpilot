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
void live_H(double *in_vec, double *out_8536018317951347315);
void live_err_fun(double *nom_x, double *delta_x, double *out_6164375506731098127);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5371067605881711340);
void live_H_mod_fun(double *state, double *out_655012525894693042);
void live_f_fun(double *state, double dt, double *out_7247431884026876858);
void live_F_fun(double *state, double dt, double *out_4635192806499064587);
void live_h_4(double *state, double *unused, double *out_6263134758060287904);
void live_H_4(double *state, double *unused, double *out_5016315836824433251);
void live_h_9(double *state, double *unused, double *out_4155182938745248125);
void live_H_9(double *state, double *unused, double *out_7905177389104512593);
void live_h_10(double *state, double *unused, double *out_1495179749976753703);
void live_H_10(double *state, double *unused, double *out_4452985845553750694);
void live_h_12(double *state, double *unused, double *out_8864295578003366566);
void live_H_12(double *state, double *unused, double *out_5763299923202667873);
void live_h_31(double *state, double *unused, double *out_7038920349504159392);
void live_H_31(double *state, double *unused, double *out_8382977894197040627);
void live_h_32(double *state, double *unused, double *out_2444227247274832805);
void live_H_32(double *state, double *unused, double *out_8514142679137781096);
void live_h_13(double *state, double *unused, double *out_8387813421989848261);
void live_H_13(double *state, double *unused, double *out_3078314380749121649);
void live_h_14(double *state, double *unused, double *out_4155182938745248125);
void live_H_14(double *state, double *unused, double *out_7905177389104512593);
void live_h_33(double *state, double *unused, double *out_2200804523929711373);
void live_H_33(double *state, double *unused, double *out_6913209174873653385);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}