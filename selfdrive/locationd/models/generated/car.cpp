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
void err_fun(double *nom_x, double *delta_x, double *out_2651103768401050149) {
   out_2651103768401050149[0] = delta_x[0] + nom_x[0];
   out_2651103768401050149[1] = delta_x[1] + nom_x[1];
   out_2651103768401050149[2] = delta_x[2] + nom_x[2];
   out_2651103768401050149[3] = delta_x[3] + nom_x[3];
   out_2651103768401050149[4] = delta_x[4] + nom_x[4];
   out_2651103768401050149[5] = delta_x[5] + nom_x[5];
   out_2651103768401050149[6] = delta_x[6] + nom_x[6];
   out_2651103768401050149[7] = delta_x[7] + nom_x[7];
   out_2651103768401050149[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8277758829227357167) {
   out_8277758829227357167[0] = -nom_x[0] + true_x[0];
   out_8277758829227357167[1] = -nom_x[1] + true_x[1];
   out_8277758829227357167[2] = -nom_x[2] + true_x[2];
   out_8277758829227357167[3] = -nom_x[3] + true_x[3];
   out_8277758829227357167[4] = -nom_x[4] + true_x[4];
   out_8277758829227357167[5] = -nom_x[5] + true_x[5];
   out_8277758829227357167[6] = -nom_x[6] + true_x[6];
   out_8277758829227357167[7] = -nom_x[7] + true_x[7];
   out_8277758829227357167[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3020658419054260785) {
   out_3020658419054260785[0] = 1.0;
   out_3020658419054260785[1] = 0;
   out_3020658419054260785[2] = 0;
   out_3020658419054260785[3] = 0;
   out_3020658419054260785[4] = 0;
   out_3020658419054260785[5] = 0;
   out_3020658419054260785[6] = 0;
   out_3020658419054260785[7] = 0;
   out_3020658419054260785[8] = 0;
   out_3020658419054260785[9] = 0;
   out_3020658419054260785[10] = 1.0;
   out_3020658419054260785[11] = 0;
   out_3020658419054260785[12] = 0;
   out_3020658419054260785[13] = 0;
   out_3020658419054260785[14] = 0;
   out_3020658419054260785[15] = 0;
   out_3020658419054260785[16] = 0;
   out_3020658419054260785[17] = 0;
   out_3020658419054260785[18] = 0;
   out_3020658419054260785[19] = 0;
   out_3020658419054260785[20] = 1.0;
   out_3020658419054260785[21] = 0;
   out_3020658419054260785[22] = 0;
   out_3020658419054260785[23] = 0;
   out_3020658419054260785[24] = 0;
   out_3020658419054260785[25] = 0;
   out_3020658419054260785[26] = 0;
   out_3020658419054260785[27] = 0;
   out_3020658419054260785[28] = 0;
   out_3020658419054260785[29] = 0;
   out_3020658419054260785[30] = 1.0;
   out_3020658419054260785[31] = 0;
   out_3020658419054260785[32] = 0;
   out_3020658419054260785[33] = 0;
   out_3020658419054260785[34] = 0;
   out_3020658419054260785[35] = 0;
   out_3020658419054260785[36] = 0;
   out_3020658419054260785[37] = 0;
   out_3020658419054260785[38] = 0;
   out_3020658419054260785[39] = 0;
   out_3020658419054260785[40] = 1.0;
   out_3020658419054260785[41] = 0;
   out_3020658419054260785[42] = 0;
   out_3020658419054260785[43] = 0;
   out_3020658419054260785[44] = 0;
   out_3020658419054260785[45] = 0;
   out_3020658419054260785[46] = 0;
   out_3020658419054260785[47] = 0;
   out_3020658419054260785[48] = 0;
   out_3020658419054260785[49] = 0;
   out_3020658419054260785[50] = 1.0;
   out_3020658419054260785[51] = 0;
   out_3020658419054260785[52] = 0;
   out_3020658419054260785[53] = 0;
   out_3020658419054260785[54] = 0;
   out_3020658419054260785[55] = 0;
   out_3020658419054260785[56] = 0;
   out_3020658419054260785[57] = 0;
   out_3020658419054260785[58] = 0;
   out_3020658419054260785[59] = 0;
   out_3020658419054260785[60] = 1.0;
   out_3020658419054260785[61] = 0;
   out_3020658419054260785[62] = 0;
   out_3020658419054260785[63] = 0;
   out_3020658419054260785[64] = 0;
   out_3020658419054260785[65] = 0;
   out_3020658419054260785[66] = 0;
   out_3020658419054260785[67] = 0;
   out_3020658419054260785[68] = 0;
   out_3020658419054260785[69] = 0;
   out_3020658419054260785[70] = 1.0;
   out_3020658419054260785[71] = 0;
   out_3020658419054260785[72] = 0;
   out_3020658419054260785[73] = 0;
   out_3020658419054260785[74] = 0;
   out_3020658419054260785[75] = 0;
   out_3020658419054260785[76] = 0;
   out_3020658419054260785[77] = 0;
   out_3020658419054260785[78] = 0;
   out_3020658419054260785[79] = 0;
   out_3020658419054260785[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4232540977944494096) {
   out_4232540977944494096[0] = state[0];
   out_4232540977944494096[1] = state[1];
   out_4232540977944494096[2] = state[2];
   out_4232540977944494096[3] = state[3];
   out_4232540977944494096[4] = state[4];
   out_4232540977944494096[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4232540977944494096[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4232540977944494096[7] = state[7];
   out_4232540977944494096[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3633973986405621337) {
   out_3633973986405621337[0] = 1;
   out_3633973986405621337[1] = 0;
   out_3633973986405621337[2] = 0;
   out_3633973986405621337[3] = 0;
   out_3633973986405621337[4] = 0;
   out_3633973986405621337[5] = 0;
   out_3633973986405621337[6] = 0;
   out_3633973986405621337[7] = 0;
   out_3633973986405621337[8] = 0;
   out_3633973986405621337[9] = 0;
   out_3633973986405621337[10] = 1;
   out_3633973986405621337[11] = 0;
   out_3633973986405621337[12] = 0;
   out_3633973986405621337[13] = 0;
   out_3633973986405621337[14] = 0;
   out_3633973986405621337[15] = 0;
   out_3633973986405621337[16] = 0;
   out_3633973986405621337[17] = 0;
   out_3633973986405621337[18] = 0;
   out_3633973986405621337[19] = 0;
   out_3633973986405621337[20] = 1;
   out_3633973986405621337[21] = 0;
   out_3633973986405621337[22] = 0;
   out_3633973986405621337[23] = 0;
   out_3633973986405621337[24] = 0;
   out_3633973986405621337[25] = 0;
   out_3633973986405621337[26] = 0;
   out_3633973986405621337[27] = 0;
   out_3633973986405621337[28] = 0;
   out_3633973986405621337[29] = 0;
   out_3633973986405621337[30] = 1;
   out_3633973986405621337[31] = 0;
   out_3633973986405621337[32] = 0;
   out_3633973986405621337[33] = 0;
   out_3633973986405621337[34] = 0;
   out_3633973986405621337[35] = 0;
   out_3633973986405621337[36] = 0;
   out_3633973986405621337[37] = 0;
   out_3633973986405621337[38] = 0;
   out_3633973986405621337[39] = 0;
   out_3633973986405621337[40] = 1;
   out_3633973986405621337[41] = 0;
   out_3633973986405621337[42] = 0;
   out_3633973986405621337[43] = 0;
   out_3633973986405621337[44] = 0;
   out_3633973986405621337[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3633973986405621337[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3633973986405621337[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3633973986405621337[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3633973986405621337[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3633973986405621337[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3633973986405621337[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3633973986405621337[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3633973986405621337[53] = -9.8000000000000007*dt;
   out_3633973986405621337[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3633973986405621337[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3633973986405621337[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3633973986405621337[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3633973986405621337[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3633973986405621337[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3633973986405621337[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3633973986405621337[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3633973986405621337[62] = 0;
   out_3633973986405621337[63] = 0;
   out_3633973986405621337[64] = 0;
   out_3633973986405621337[65] = 0;
   out_3633973986405621337[66] = 0;
   out_3633973986405621337[67] = 0;
   out_3633973986405621337[68] = 0;
   out_3633973986405621337[69] = 0;
   out_3633973986405621337[70] = 1;
   out_3633973986405621337[71] = 0;
   out_3633973986405621337[72] = 0;
   out_3633973986405621337[73] = 0;
   out_3633973986405621337[74] = 0;
   out_3633973986405621337[75] = 0;
   out_3633973986405621337[76] = 0;
   out_3633973986405621337[77] = 0;
   out_3633973986405621337[78] = 0;
   out_3633973986405621337[79] = 0;
   out_3633973986405621337[80] = 1;
}
void h_25(double *state, double *unused, double *out_7294994506742456200) {
   out_7294994506742456200[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5022455929348502491) {
   out_5022455929348502491[0] = 0;
   out_5022455929348502491[1] = 0;
   out_5022455929348502491[2] = 0;
   out_5022455929348502491[3] = 0;
   out_5022455929348502491[4] = 0;
   out_5022455929348502491[5] = 0;
   out_5022455929348502491[6] = 1;
   out_5022455929348502491[7] = 0;
   out_5022455929348502491[8] = 0;
}
void h_24(double *state, double *unused, double *out_4543929373308163603) {
   out_4543929373308163603[0] = state[4];
   out_4543929373308163603[1] = state[5];
}
void H_24(double *state, double *unused, double *out_275946602381993457) {
   out_275946602381993457[0] = 0;
   out_275946602381993457[1] = 0;
   out_275946602381993457[2] = 0;
   out_275946602381993457[3] = 0;
   out_275946602381993457[4] = 1;
   out_275946602381993457[5] = 0;
   out_275946602381993457[6] = 0;
   out_275946602381993457[7] = 0;
   out_275946602381993457[8] = 0;
   out_275946602381993457[9] = 0;
   out_275946602381993457[10] = 0;
   out_275946602381993457[11] = 0;
   out_275946602381993457[12] = 0;
   out_275946602381993457[13] = 0;
   out_275946602381993457[14] = 1;
   out_275946602381993457[15] = 0;
   out_275946602381993457[16] = 0;
   out_275946602381993457[17] = 0;
}
void h_30(double *state, double *unused, double *out_5094206498220989764) {
   out_5094206498220989764[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4893116982205262421) {
   out_4893116982205262421[0] = 0;
   out_4893116982205262421[1] = 0;
   out_4893116982205262421[2] = 0;
   out_4893116982205262421[3] = 0;
   out_4893116982205262421[4] = 1;
   out_4893116982205262421[5] = 0;
   out_4893116982205262421[6] = 0;
   out_4893116982205262421[7] = 0;
   out_4893116982205262421[8] = 0;
}
void h_26(double *state, double *unused, double *out_2057420446776657079) {
   out_2057420446776657079[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5679309993458814395) {
   out_5679309993458814395[0] = 0;
   out_5679309993458814395[1] = 0;
   out_5679309993458814395[2] = 0;
   out_5679309993458814395[3] = 0;
   out_5679309993458814395[4] = 0;
   out_5679309993458814395[5] = 0;
   out_5679309993458814395[6] = 0;
   out_5679309993458814395[7] = 1;
   out_5679309993458814395[8] = 0;
}
void h_27(double *state, double *unused, double *out_1725548905698020242) {
   out_1725548905698020242[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2718353670404837510) {
   out_2718353670404837510[0] = 0;
   out_2718353670404837510[1] = 0;
   out_2718353670404837510[2] = 0;
   out_2718353670404837510[3] = 1;
   out_2718353670404837510[4] = 0;
   out_2718353670404837510[5] = 0;
   out_2718353670404837510[6] = 0;
   out_2718353670404837510[7] = 0;
   out_2718353670404837510[8] = 0;
}
void h_29(double *state, double *unused, double *out_6037716351122652199) {
   out_6037716351122652199[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5403348326519654605) {
   out_5403348326519654605[0] = 0;
   out_5403348326519654605[1] = 1;
   out_5403348326519654605[2] = 0;
   out_5403348326519654605[3] = 0;
   out_5403348326519654605[4] = 0;
   out_5403348326519654605[5] = 0;
   out_5403348326519654605[6] = 0;
   out_5403348326519654605[7] = 0;
   out_5403348326519654605[8] = 0;
}
void h_28(double *state, double *unused, double *out_1311237145746312443) {
   out_1311237145746312443[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7366978598084980856) {
   out_7366978598084980856[0] = 1;
   out_7366978598084980856[1] = 0;
   out_7366978598084980856[2] = 0;
   out_7366978598084980856[3] = 0;
   out_7366978598084980856[4] = 0;
   out_7366978598084980856[5] = 0;
   out_7366978598084980856[6] = 0;
   out_7366978598084980856[7] = 0;
   out_7366978598084980856[8] = 0;
}
void h_31(double *state, double *unused, double *out_7137807838391327055) {
   out_7137807838391327055[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5053101891225462919) {
   out_5053101891225462919[0] = 0;
   out_5053101891225462919[1] = 0;
   out_5053101891225462919[2] = 0;
   out_5053101891225462919[3] = 0;
   out_5053101891225462919[4] = 0;
   out_5053101891225462919[5] = 0;
   out_5053101891225462919[6] = 0;
   out_5053101891225462919[7] = 0;
   out_5053101891225462919[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2651103768401050149) {
  err_fun(nom_x, delta_x, out_2651103768401050149);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8277758829227357167) {
  inv_err_fun(nom_x, true_x, out_8277758829227357167);
}
void car_H_mod_fun(double *state, double *out_3020658419054260785) {
  H_mod_fun(state, out_3020658419054260785);
}
void car_f_fun(double *state, double dt, double *out_4232540977944494096) {
  f_fun(state,  dt, out_4232540977944494096);
}
void car_F_fun(double *state, double dt, double *out_3633973986405621337) {
  F_fun(state,  dt, out_3633973986405621337);
}
void car_h_25(double *state, double *unused, double *out_7294994506742456200) {
  h_25(state, unused, out_7294994506742456200);
}
void car_H_25(double *state, double *unused, double *out_5022455929348502491) {
  H_25(state, unused, out_5022455929348502491);
}
void car_h_24(double *state, double *unused, double *out_4543929373308163603) {
  h_24(state, unused, out_4543929373308163603);
}
void car_H_24(double *state, double *unused, double *out_275946602381993457) {
  H_24(state, unused, out_275946602381993457);
}
void car_h_30(double *state, double *unused, double *out_5094206498220989764) {
  h_30(state, unused, out_5094206498220989764);
}
void car_H_30(double *state, double *unused, double *out_4893116982205262421) {
  H_30(state, unused, out_4893116982205262421);
}
void car_h_26(double *state, double *unused, double *out_2057420446776657079) {
  h_26(state, unused, out_2057420446776657079);
}
void car_H_26(double *state, double *unused, double *out_5679309993458814395) {
  H_26(state, unused, out_5679309993458814395);
}
void car_h_27(double *state, double *unused, double *out_1725548905698020242) {
  h_27(state, unused, out_1725548905698020242);
}
void car_H_27(double *state, double *unused, double *out_2718353670404837510) {
  H_27(state, unused, out_2718353670404837510);
}
void car_h_29(double *state, double *unused, double *out_6037716351122652199) {
  h_29(state, unused, out_6037716351122652199);
}
void car_H_29(double *state, double *unused, double *out_5403348326519654605) {
  H_29(state, unused, out_5403348326519654605);
}
void car_h_28(double *state, double *unused, double *out_1311237145746312443) {
  h_28(state, unused, out_1311237145746312443);
}
void car_H_28(double *state, double *unused, double *out_7366978598084980856) {
  H_28(state, unused, out_7366978598084980856);
}
void car_h_31(double *state, double *unused, double *out_7137807838391327055) {
  h_31(state, unused, out_7137807838391327055);
}
void car_H_31(double *state, double *unused, double *out_5053101891225462919) {
  H_31(state, unused, out_5053101891225462919);
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
