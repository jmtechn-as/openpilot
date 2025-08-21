#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7175153034774180214) {
   out_7175153034774180214[0] = delta_x[0] + nom_x[0];
   out_7175153034774180214[1] = delta_x[1] + nom_x[1];
   out_7175153034774180214[2] = delta_x[2] + nom_x[2];
   out_7175153034774180214[3] = delta_x[3] + nom_x[3];
   out_7175153034774180214[4] = delta_x[4] + nom_x[4];
   out_7175153034774180214[5] = delta_x[5] + nom_x[5];
   out_7175153034774180214[6] = delta_x[6] + nom_x[6];
   out_7175153034774180214[7] = delta_x[7] + nom_x[7];
   out_7175153034774180214[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_293629274085095843) {
   out_293629274085095843[0] = -nom_x[0] + true_x[0];
   out_293629274085095843[1] = -nom_x[1] + true_x[1];
   out_293629274085095843[2] = -nom_x[2] + true_x[2];
   out_293629274085095843[3] = -nom_x[3] + true_x[3];
   out_293629274085095843[4] = -nom_x[4] + true_x[4];
   out_293629274085095843[5] = -nom_x[5] + true_x[5];
   out_293629274085095843[6] = -nom_x[6] + true_x[6];
   out_293629274085095843[7] = -nom_x[7] + true_x[7];
   out_293629274085095843[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2188577883657235519) {
   out_2188577883657235519[0] = 1.0;
   out_2188577883657235519[1] = 0;
   out_2188577883657235519[2] = 0;
   out_2188577883657235519[3] = 0;
   out_2188577883657235519[4] = 0;
   out_2188577883657235519[5] = 0;
   out_2188577883657235519[6] = 0;
   out_2188577883657235519[7] = 0;
   out_2188577883657235519[8] = 0;
   out_2188577883657235519[9] = 0;
   out_2188577883657235519[10] = 1.0;
   out_2188577883657235519[11] = 0;
   out_2188577883657235519[12] = 0;
   out_2188577883657235519[13] = 0;
   out_2188577883657235519[14] = 0;
   out_2188577883657235519[15] = 0;
   out_2188577883657235519[16] = 0;
   out_2188577883657235519[17] = 0;
   out_2188577883657235519[18] = 0;
   out_2188577883657235519[19] = 0;
   out_2188577883657235519[20] = 1.0;
   out_2188577883657235519[21] = 0;
   out_2188577883657235519[22] = 0;
   out_2188577883657235519[23] = 0;
   out_2188577883657235519[24] = 0;
   out_2188577883657235519[25] = 0;
   out_2188577883657235519[26] = 0;
   out_2188577883657235519[27] = 0;
   out_2188577883657235519[28] = 0;
   out_2188577883657235519[29] = 0;
   out_2188577883657235519[30] = 1.0;
   out_2188577883657235519[31] = 0;
   out_2188577883657235519[32] = 0;
   out_2188577883657235519[33] = 0;
   out_2188577883657235519[34] = 0;
   out_2188577883657235519[35] = 0;
   out_2188577883657235519[36] = 0;
   out_2188577883657235519[37] = 0;
   out_2188577883657235519[38] = 0;
   out_2188577883657235519[39] = 0;
   out_2188577883657235519[40] = 1.0;
   out_2188577883657235519[41] = 0;
   out_2188577883657235519[42] = 0;
   out_2188577883657235519[43] = 0;
   out_2188577883657235519[44] = 0;
   out_2188577883657235519[45] = 0;
   out_2188577883657235519[46] = 0;
   out_2188577883657235519[47] = 0;
   out_2188577883657235519[48] = 0;
   out_2188577883657235519[49] = 0;
   out_2188577883657235519[50] = 1.0;
   out_2188577883657235519[51] = 0;
   out_2188577883657235519[52] = 0;
   out_2188577883657235519[53] = 0;
   out_2188577883657235519[54] = 0;
   out_2188577883657235519[55] = 0;
   out_2188577883657235519[56] = 0;
   out_2188577883657235519[57] = 0;
   out_2188577883657235519[58] = 0;
   out_2188577883657235519[59] = 0;
   out_2188577883657235519[60] = 1.0;
   out_2188577883657235519[61] = 0;
   out_2188577883657235519[62] = 0;
   out_2188577883657235519[63] = 0;
   out_2188577883657235519[64] = 0;
   out_2188577883657235519[65] = 0;
   out_2188577883657235519[66] = 0;
   out_2188577883657235519[67] = 0;
   out_2188577883657235519[68] = 0;
   out_2188577883657235519[69] = 0;
   out_2188577883657235519[70] = 1.0;
   out_2188577883657235519[71] = 0;
   out_2188577883657235519[72] = 0;
   out_2188577883657235519[73] = 0;
   out_2188577883657235519[74] = 0;
   out_2188577883657235519[75] = 0;
   out_2188577883657235519[76] = 0;
   out_2188577883657235519[77] = 0;
   out_2188577883657235519[78] = 0;
   out_2188577883657235519[79] = 0;
   out_2188577883657235519[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3994854500695217019) {
   out_3994854500695217019[0] = state[0];
   out_3994854500695217019[1] = state[1];
   out_3994854500695217019[2] = state[2];
   out_3994854500695217019[3] = state[3];
   out_3994854500695217019[4] = state[4];
   out_3994854500695217019[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3994854500695217019[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3994854500695217019[7] = state[7];
   out_3994854500695217019[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9180351073273198473) {
   out_9180351073273198473[0] = 1;
   out_9180351073273198473[1] = 0;
   out_9180351073273198473[2] = 0;
   out_9180351073273198473[3] = 0;
   out_9180351073273198473[4] = 0;
   out_9180351073273198473[5] = 0;
   out_9180351073273198473[6] = 0;
   out_9180351073273198473[7] = 0;
   out_9180351073273198473[8] = 0;
   out_9180351073273198473[9] = 0;
   out_9180351073273198473[10] = 1;
   out_9180351073273198473[11] = 0;
   out_9180351073273198473[12] = 0;
   out_9180351073273198473[13] = 0;
   out_9180351073273198473[14] = 0;
   out_9180351073273198473[15] = 0;
   out_9180351073273198473[16] = 0;
   out_9180351073273198473[17] = 0;
   out_9180351073273198473[18] = 0;
   out_9180351073273198473[19] = 0;
   out_9180351073273198473[20] = 1;
   out_9180351073273198473[21] = 0;
   out_9180351073273198473[22] = 0;
   out_9180351073273198473[23] = 0;
   out_9180351073273198473[24] = 0;
   out_9180351073273198473[25] = 0;
   out_9180351073273198473[26] = 0;
   out_9180351073273198473[27] = 0;
   out_9180351073273198473[28] = 0;
   out_9180351073273198473[29] = 0;
   out_9180351073273198473[30] = 1;
   out_9180351073273198473[31] = 0;
   out_9180351073273198473[32] = 0;
   out_9180351073273198473[33] = 0;
   out_9180351073273198473[34] = 0;
   out_9180351073273198473[35] = 0;
   out_9180351073273198473[36] = 0;
   out_9180351073273198473[37] = 0;
   out_9180351073273198473[38] = 0;
   out_9180351073273198473[39] = 0;
   out_9180351073273198473[40] = 1;
   out_9180351073273198473[41] = 0;
   out_9180351073273198473[42] = 0;
   out_9180351073273198473[43] = 0;
   out_9180351073273198473[44] = 0;
   out_9180351073273198473[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9180351073273198473[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9180351073273198473[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9180351073273198473[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9180351073273198473[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9180351073273198473[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9180351073273198473[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9180351073273198473[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9180351073273198473[53] = -9.8000000000000007*dt;
   out_9180351073273198473[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9180351073273198473[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9180351073273198473[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9180351073273198473[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9180351073273198473[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9180351073273198473[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9180351073273198473[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9180351073273198473[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9180351073273198473[62] = 0;
   out_9180351073273198473[63] = 0;
   out_9180351073273198473[64] = 0;
   out_9180351073273198473[65] = 0;
   out_9180351073273198473[66] = 0;
   out_9180351073273198473[67] = 0;
   out_9180351073273198473[68] = 0;
   out_9180351073273198473[69] = 0;
   out_9180351073273198473[70] = 1;
   out_9180351073273198473[71] = 0;
   out_9180351073273198473[72] = 0;
   out_9180351073273198473[73] = 0;
   out_9180351073273198473[74] = 0;
   out_9180351073273198473[75] = 0;
   out_9180351073273198473[76] = 0;
   out_9180351073273198473[77] = 0;
   out_9180351073273198473[78] = 0;
   out_9180351073273198473[79] = 0;
   out_9180351073273198473[80] = 1;
}
void h_25(double *state, double *unused, double *out_7124885558024582014) {
   out_7124885558024582014[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4085512975129834147) {
   out_4085512975129834147[0] = 0;
   out_4085512975129834147[1] = 0;
   out_4085512975129834147[2] = 0;
   out_4085512975129834147[3] = 0;
   out_4085512975129834147[4] = 0;
   out_4085512975129834147[5] = 0;
   out_4085512975129834147[6] = 1;
   out_4085512975129834147[7] = 0;
   out_4085512975129834147[8] = 0;
}
void h_24(double *state, double *unused, double *out_4471539831849915780) {
   out_4471539831849915780[0] = state[4];
   out_4471539831849915780[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2568692482978351610) {
   out_2568692482978351610[0] = 0;
   out_2568692482978351610[1] = 0;
   out_2568692482978351610[2] = 0;
   out_2568692482978351610[3] = 0;
   out_2568692482978351610[4] = 1;
   out_2568692482978351610[5] = 0;
   out_2568692482978351610[6] = 0;
   out_2568692482978351610[7] = 0;
   out_2568692482978351610[8] = 0;
   out_2568692482978351610[9] = 0;
   out_2568692482978351610[10] = 0;
   out_2568692482978351610[11] = 0;
   out_2568692482978351610[12] = 0;
   out_2568692482978351610[13] = 0;
   out_2568692482978351610[14] = 1;
   out_2568692482978351610[15] = 0;
   out_2568692482978351610[16] = 0;
   out_2568692482978351610[17] = 0;
}
void h_30(double *state, double *unused, double *out_5795801079133946489) {
   out_5795801079133946489[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4214851922273074217) {
   out_4214851922273074217[0] = 0;
   out_4214851922273074217[1] = 0;
   out_4214851922273074217[2] = 0;
   out_4214851922273074217[3] = 0;
   out_4214851922273074217[4] = 1;
   out_4214851922273074217[5] = 0;
   out_4214851922273074217[6] = 0;
   out_4214851922273074217[7] = 0;
   out_4214851922273074217[8] = 0;
}
void h_26(double *state, double *unused, double *out_4245286174931378474) {
   out_4245286174931378474[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7827016294003890371) {
   out_7827016294003890371[0] = 0;
   out_7827016294003890371[1] = 0;
   out_7827016294003890371[2] = 0;
   out_7827016294003890371[3] = 0;
   out_7827016294003890371[4] = 0;
   out_7827016294003890371[5] = 0;
   out_7827016294003890371[6] = 0;
   out_7827016294003890371[7] = 1;
   out_7827016294003890371[8] = 0;
}
void h_27(double *state, double *unused, double *out_332642584392850235) {
   out_332642584392850235[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6389615234073499128) {
   out_6389615234073499128[0] = 0;
   out_6389615234073499128[1] = 0;
   out_6389615234073499128[2] = 0;
   out_6389615234073499128[3] = 1;
   out_6389615234073499128[4] = 0;
   out_6389615234073499128[5] = 0;
   out_6389615234073499128[6] = 0;
   out_6389615234073499128[7] = 0;
   out_6389615234073499128[8] = 0;
}
void h_29(double *state, double *unused, double *out_7649269726914759482) {
   out_7649269726914759482[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3704620577958682033) {
   out_3704620577958682033[0] = 0;
   out_3704620577958682033[1] = 1;
   out_3704620577958682033[2] = 0;
   out_3704620577958682033[3] = 0;
   out_3704620577958682033[4] = 0;
   out_3704620577958682033[5] = 0;
   out_3704620577958682033[6] = 0;
   out_3704620577958682033[7] = 0;
   out_3704620577958682033[8] = 0;
}
void h_28(double *state, double *unused, double *out_5913808473067323233) {
   out_5913808473067323233[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8787019595028212607) {
   out_8787019595028212607[0] = 1;
   out_8787019595028212607[1] = 0;
   out_8787019595028212607[2] = 0;
   out_8787019595028212607[3] = 0;
   out_8787019595028212607[4] = 0;
   out_8787019595028212607[5] = 0;
   out_8787019595028212607[6] = 0;
   out_8787019595028212607[7] = 0;
   out_8787019595028212607[8] = 0;
}
void h_31(double *state, double *unused, double *out_2808436642971713892) {
   out_2808436642971713892[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4054867013252873719) {
   out_4054867013252873719[0] = 0;
   out_4054867013252873719[1] = 0;
   out_4054867013252873719[2] = 0;
   out_4054867013252873719[3] = 0;
   out_4054867013252873719[4] = 0;
   out_4054867013252873719[5] = 0;
   out_4054867013252873719[6] = 0;
   out_4054867013252873719[7] = 0;
   out_4054867013252873719[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7175153034774180214) {
  err_fun(nom_x, delta_x, out_7175153034774180214);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_293629274085095843) {
  inv_err_fun(nom_x, true_x, out_293629274085095843);
}
void car_H_mod_fun(double *state, double *out_2188577883657235519) {
  H_mod_fun(state, out_2188577883657235519);
}
void car_f_fun(double *state, double dt, double *out_3994854500695217019) {
  f_fun(state,  dt, out_3994854500695217019);
}
void car_F_fun(double *state, double dt, double *out_9180351073273198473) {
  F_fun(state,  dt, out_9180351073273198473);
}
void car_h_25(double *state, double *unused, double *out_7124885558024582014) {
  h_25(state, unused, out_7124885558024582014);
}
void car_H_25(double *state, double *unused, double *out_4085512975129834147) {
  H_25(state, unused, out_4085512975129834147);
}
void car_h_24(double *state, double *unused, double *out_4471539831849915780) {
  h_24(state, unused, out_4471539831849915780);
}
void car_H_24(double *state, double *unused, double *out_2568692482978351610) {
  H_24(state, unused, out_2568692482978351610);
}
void car_h_30(double *state, double *unused, double *out_5795801079133946489) {
  h_30(state, unused, out_5795801079133946489);
}
void car_H_30(double *state, double *unused, double *out_4214851922273074217) {
  H_30(state, unused, out_4214851922273074217);
}
void car_h_26(double *state, double *unused, double *out_4245286174931378474) {
  h_26(state, unused, out_4245286174931378474);
}
void car_H_26(double *state, double *unused, double *out_7827016294003890371) {
  H_26(state, unused, out_7827016294003890371);
}
void car_h_27(double *state, double *unused, double *out_332642584392850235) {
  h_27(state, unused, out_332642584392850235);
}
void car_H_27(double *state, double *unused, double *out_6389615234073499128) {
  H_27(state, unused, out_6389615234073499128);
}
void car_h_29(double *state, double *unused, double *out_7649269726914759482) {
  h_29(state, unused, out_7649269726914759482);
}
void car_H_29(double *state, double *unused, double *out_3704620577958682033) {
  H_29(state, unused, out_3704620577958682033);
}
void car_h_28(double *state, double *unused, double *out_5913808473067323233) {
  h_28(state, unused, out_5913808473067323233);
}
void car_H_28(double *state, double *unused, double *out_8787019595028212607) {
  H_28(state, unused, out_8787019595028212607);
}
void car_h_31(double *state, double *unused, double *out_2808436642971713892) {
  h_31(state, unused, out_2808436642971713892);
}
void car_H_31(double *state, double *unused, double *out_4054867013252873719) {
  H_31(state, unused, out_4054867013252873719);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
