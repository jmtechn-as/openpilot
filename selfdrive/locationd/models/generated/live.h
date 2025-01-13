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
void live_H(double *in_vec, double *out_1732441731476526626);
void live_err_fun(double *nom_x, double *delta_x, double *out_4454867101189482171);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5255716021691337529);
void live_H_mod_fun(double *state, double *out_726322911707756993);
void live_f_fun(double *state, double dt, double *out_6239111807402578215);
void live_F_fun(double *state, double dt, double *out_8664021659405379942);
void live_h_4(double *state, double *unused, double *out_8142156757979686188);
void live_H_4(double *state, double *unused, double *out_1877551153725506110);
void live_h_9(double *state, double *unused, double *out_1231946576368523809);
void live_H_9(double *state, double *unused, double *out_1636361507095915465);
void live_h_10(double *state, double *unused, double *out_8070861354773336648);
void live_H_10(double *state, double *unused, double *out_6670508049630711941);
void live_h_12(double *state, double *unused, double *out_7797497304850733974);
void live_H_12(double *state, double *unused, double *out_3141905254306455685);
void live_h_31(double *state, double *unused, double *out_3507491892078258665);
void live_H_31(double *state, double *unused, double *out_1489110903647101266);
void live_h_32(double *state, double *unused, double *out_6052620192175737357);
void live_H_32(double *state, double *unused, double *out_2729588334024807804);
void live_h_13(double *state, double *unused, double *out_2947297868852228405);
void live_H_13(double *state, double *unused, double *out_6964314085070506957);
void live_h_14(double *state, double *unused, double *out_1231946576368523809);
void live_H_14(double *state, double *unused, double *out_1636361507095915465);
void live_h_33(double *state, double *unused, double *out_6101989358040357876);
void live_H_33(double *state, double *unused, double *out_4639667908285958870);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}