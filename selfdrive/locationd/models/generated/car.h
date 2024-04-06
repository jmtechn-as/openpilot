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
void car_err_fun(double *nom_x, double *delta_x, double *out_8428659455625657161);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2435728419690232497);
void car_H_mod_fun(double *state, double *out_817306993903023728);
void car_f_fun(double *state, double dt, double *out_5432806874620613033);
void car_F_fun(double *state, double dt, double *out_7015879500496735849);
void car_h_25(double *state, double *unused, double *out_7939230388010950143);
void car_H_25(double *state, double *unused, double *out_8312742630226899182);
void car_h_24(double *state, double *unused, double *out_5793576264067600587);
void car_H_24(double *state, double *unused, double *out_7961351844477152868);
void car_h_30(double *state, double *unused, double *out_3736678459348250537);
void car_H_30(double *state, double *unused, double *out_5794409671719650555);
void car_h_26(double *state, double *unused, double *out_1264994808312595108);
void car_H_26(double *state, double *unused, double *out_6392498124608596210);
void car_h_27(double *state, double *unused, double *out_8817065850962920513);
void car_H_27(double *state, double *unused, double *out_3570815600535707338);
void car_h_29(double *state, double *unused, double *out_3474231026905970974);
void car_H_29(double *state, double *unused, double *out_5284178327405258371);
void car_h_28(double *state, double *unused, double *out_8279424356236135061);
void car_H_28(double *state, double *unused, double *out_8080166729234762671);
void car_h_31(double *state, double *unused, double *out_46290893126426784);
void car_H_31(double *state, double *unused, double *out_5766290022375244734);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}