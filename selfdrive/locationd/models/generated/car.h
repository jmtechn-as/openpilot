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
void car_err_fun(double *nom_x, double *delta_x, double *out_5283071209818325702);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6148504474804144895);
void car_H_mod_fun(double *state, double *out_2433100626038737755);
void car_f_fun(double *state, double dt, double *out_2438546081678800452);
void car_F_fun(double *state, double dt, double *out_7089989771211385344);
void car_h_25(double *state, double *unused, double *out_2030834903709412354);
void car_H_25(double *state, double *unused, double *out_1367053570080972270);
void car_h_24(double *state, double *unused, double *out_3656283077841470618);
void car_H_24(double *state, double *unused, double *out_810160853526177703);
void car_h_30(double *state, double *unused, double *out_8652643913179466821);
void car_H_30(double *state, double *unused, double *out_1151279388426276357);
void car_h_26(double *state, double *unused, double *out_3729443189915925990);
void car_H_26(double *state, double *unused, double *out_1937472399679828331);
void car_h_27(double *state, double *unused, double *out_4635240224518110005);
void car_H_27(double *state, double *unused, double *out_1023483923374148554);
void car_h_29(double *state, double *unused, double *out_4792426892869239150);
void car_H_29(double *state, double *unused, double *out_1661510732740668541);
void car_h_28(double *state, double *unused, double *out_209879309219878472);
void car_H_28(double *state, double *unused, double *out_3420888284328862033);
void car_h_31(double *state, double *unused, double *out_8534636519246090077);
void car_H_31(double *state, double *unused, double *out_1311264297446476855);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}