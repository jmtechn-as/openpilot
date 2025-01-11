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
void live_H(double *in_vec, double *out_7735064062044669065);
void live_err_fun(double *nom_x, double *delta_x, double *out_3746502733237838210);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_19434943941211936);
void live_H_mod_fun(double *state, double *out_7404275291515255376);
void live_f_fun(double *state, double dt, double *out_5792787500173903696);
void live_F_fun(double *state, double dt, double *out_1606198340699486384);
void live_h_4(double *state, double *unused, double *out_4123454451027773584);
void live_H_4(double *state, double *unused, double *out_1343093665192579012);
void live_h_9(double *state, double *unused, double *out_5092385261794836187);
void live_H_9(double *state, double *unused, double *out_2814074071162198471);
void live_h_10(double *state, double *unused, double *out_8044802378308935646);
void live_H_10(double *state, double *unused, double *out_3381651542868614034);
void live_h_12(double *state, double *unused, double *out_1248501386643105198);
void live_H_12(double *state, double *unused, double *out_1964192690240172679);
void live_h_31(double *state, double *unused, double *out_8619418292046531508);
void live_H_31(double *state, double *unused, double *out_4709755722565186388);
void live_h_32(double *state, double *unused, double *out_253586207619748459);
void live_H_32(double *state, double *unused, double *out_5361824250975743202);
void live_h_13(double *state, double *unused, double *out_991055124662737915);
void live_H_13(double *state, double *unused, double *out_6666752244728303337);
void live_h_14(double *state, double *unused, double *out_5092385261794836187);
void live_H_14(double *state, double *unused, double *out_2814074071162198471);
void live_h_33(double *state, double *unused, double *out_6836706684534737351);
void live_H_33(double *state, double *unused, double *out_7860312727204043992);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}