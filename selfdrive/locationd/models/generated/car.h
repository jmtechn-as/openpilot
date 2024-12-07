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
void car_err_fun(double *nom_x, double *delta_x, double *out_2651103768401050149);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8277758829227357167);
void car_H_mod_fun(double *state, double *out_3020658419054260785);
void car_f_fun(double *state, double dt, double *out_4232540977944494096);
void car_F_fun(double *state, double dt, double *out_3633973986405621337);
void car_h_25(double *state, double *unused, double *out_7294994506742456200);
void car_H_25(double *state, double *unused, double *out_5022455929348502491);
void car_h_24(double *state, double *unused, double *out_4543929373308163603);
void car_H_24(double *state, double *unused, double *out_275946602381993457);
void car_h_30(double *state, double *unused, double *out_5094206498220989764);
void car_H_30(double *state, double *unused, double *out_4893116982205262421);
void car_h_26(double *state, double *unused, double *out_2057420446776657079);
void car_H_26(double *state, double *unused, double *out_5679309993458814395);
void car_h_27(double *state, double *unused, double *out_1725548905698020242);
void car_H_27(double *state, double *unused, double *out_2718353670404837510);
void car_h_29(double *state, double *unused, double *out_6037716351122652199);
void car_H_29(double *state, double *unused, double *out_5403348326519654605);
void car_h_28(double *state, double *unused, double *out_1311237145746312443);
void car_H_28(double *state, double *unused, double *out_7366978598084980856);
void car_h_31(double *state, double *unused, double *out_7137807838391327055);
void car_H_31(double *state, double *unused, double *out_5053101891225462919);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}