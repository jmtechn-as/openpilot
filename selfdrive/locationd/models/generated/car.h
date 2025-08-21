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
void car_err_fun(double *nom_x, double *delta_x, double *out_7175153034774180214);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_293629274085095843);
void car_H_mod_fun(double *state, double *out_2188577883657235519);
void car_f_fun(double *state, double dt, double *out_3994854500695217019);
void car_F_fun(double *state, double dt, double *out_9180351073273198473);
void car_h_25(double *state, double *unused, double *out_7124885558024582014);
void car_H_25(double *state, double *unused, double *out_4085512975129834147);
void car_h_24(double *state, double *unused, double *out_4471539831849915780);
void car_H_24(double *state, double *unused, double *out_2568692482978351610);
void car_h_30(double *state, double *unused, double *out_5795801079133946489);
void car_H_30(double *state, double *unused, double *out_4214851922273074217);
void car_h_26(double *state, double *unused, double *out_4245286174931378474);
void car_H_26(double *state, double *unused, double *out_7827016294003890371);
void car_h_27(double *state, double *unused, double *out_332642584392850235);
void car_H_27(double *state, double *unused, double *out_6389615234073499128);
void car_h_29(double *state, double *unused, double *out_7649269726914759482);
void car_H_29(double *state, double *unused, double *out_3704620577958682033);
void car_h_28(double *state, double *unused, double *out_5913808473067323233);
void car_H_28(double *state, double *unused, double *out_8787019595028212607);
void car_h_31(double *state, double *unused, double *out_2808436642971713892);
void car_H_31(double *state, double *unused, double *out_4054867013252873719);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}