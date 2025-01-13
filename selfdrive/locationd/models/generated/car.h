#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2424299756328140598);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9107298749992071872);
void car_H_mod_fun(double *state, double *out_371992781021667047);
void car_f_fun(double *state, double dt, double *out_3910341159372398453);
void car_F_fun(double *state, double dt, double *out_2495385078796925409);
void car_h_25(double *state, double *unused, double *out_5748993800625767294);
void car_H_25(double *state, double *unused, double *out_7053231232002920988);
void car_h_24(double *state, double *unused, double *out_7289435458421562664);
void car_H_24(double *state, double *unused, double *out_4817941035115112527);
void car_h_30(double *state, double *unused, double *out_2111544272371744944);
void car_H_30(double *state, double *unused, double *out_4476822500215013873);
void car_h_26(double *state, double *unused, double *out_492861048436952707);
void car_H_26(double *state, double *unused, double *out_3311727913128864764);
void car_h_27(double *state, double *unused, double *out_2614934933004594812);
void car_H_27(double *state, double *unused, double *out_6651585812015438784);
void car_h_29(double *state, double *unused, double *out_2890128995289100701);
void car_H_29(double *state, double *unused, double *out_8364948538884989817);
void car_h_28(double *state, double *unused, double *out_8717454368737789202);
void car_H_28(double *state, double *unused, double *out_4999396517755031225);
void car_h_31(double *state, double *unused, double *out_1711564963623121031);
void car_H_31(double *state, double *unused, double *out_7083877193879881416);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}