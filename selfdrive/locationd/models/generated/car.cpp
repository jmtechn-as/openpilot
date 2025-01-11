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
void err_fun(double *nom_x, double *delta_x, double *out_5283071209818325702) {
   out_5283071209818325702[0] = delta_x[0] + nom_x[0];
   out_5283071209818325702[1] = delta_x[1] + nom_x[1];
   out_5283071209818325702[2] = delta_x[2] + nom_x[2];
   out_5283071209818325702[3] = delta_x[3] + nom_x[3];
   out_5283071209818325702[4] = delta_x[4] + nom_x[4];
   out_5283071209818325702[5] = delta_x[5] + nom_x[5];
   out_5283071209818325702[6] = delta_x[6] + nom_x[6];
   out_5283071209818325702[7] = delta_x[7] + nom_x[7];
   out_5283071209818325702[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6148504474804144895) {
   out_6148504474804144895[0] = -nom_x[0] + true_x[0];
   out_6148504474804144895[1] = -nom_x[1] + true_x[1];
   out_6148504474804144895[2] = -nom_x[2] + true_x[2];
   out_6148504474804144895[3] = -nom_x[3] + true_x[3];
   out_6148504474804144895[4] = -nom_x[4] + true_x[4];
   out_6148504474804144895[5] = -nom_x[5] + true_x[5];
   out_6148504474804144895[6] = -nom_x[6] + true_x[6];
   out_6148504474804144895[7] = -nom_x[7] + true_x[7];
   out_6148504474804144895[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2433100626038737755) {
   out_2433100626038737755[0] = 1.0;
   out_2433100626038737755[1] = 0;
   out_2433100626038737755[2] = 0;
   out_2433100626038737755[3] = 0;
   out_2433100626038737755[4] = 0;
   out_2433100626038737755[5] = 0;
   out_2433100626038737755[6] = 0;
   out_2433100626038737755[7] = 0;
   out_2433100626038737755[8] = 0;
   out_2433100626038737755[9] = 0;
   out_2433100626038737755[10] = 1.0;
   out_2433100626038737755[11] = 0;
   out_2433100626038737755[12] = 0;
   out_2433100626038737755[13] = 0;
   out_2433100626038737755[14] = 0;
   out_2433100626038737755[15] = 0;
   out_2433100626038737755[16] = 0;
   out_2433100626038737755[17] = 0;
   out_2433100626038737755[18] = 0;
   out_2433100626038737755[19] = 0;
   out_2433100626038737755[20] = 1.0;
   out_2433100626038737755[21] = 0;
   out_2433100626038737755[22] = 0;
   out_2433100626038737755[23] = 0;
   out_2433100626038737755[24] = 0;
   out_2433100626038737755[25] = 0;
   out_2433100626038737755[26] = 0;
   out_2433100626038737755[27] = 0;
   out_2433100626038737755[28] = 0;
   out_2433100626038737755[29] = 0;
   out_2433100626038737755[30] = 1.0;
   out_2433100626038737755[31] = 0;
   out_2433100626038737755[32] = 0;
   out_2433100626038737755[33] = 0;
   out_2433100626038737755[34] = 0;
   out_2433100626038737755[35] = 0;
   out_2433100626038737755[36] = 0;
   out_2433100626038737755[37] = 0;
   out_2433100626038737755[38] = 0;
   out_2433100626038737755[39] = 0;
   out_2433100626038737755[40] = 1.0;
   out_2433100626038737755[41] = 0;
   out_2433100626038737755[42] = 0;
   out_2433100626038737755[43] = 0;
   out_2433100626038737755[44] = 0;
   out_2433100626038737755[45] = 0;
   out_2433100626038737755[46] = 0;
   out_2433100626038737755[47] = 0;
   out_2433100626038737755[48] = 0;
   out_2433100626038737755[49] = 0;
   out_2433100626038737755[50] = 1.0;
   out_2433100626038737755[51] = 0;
   out_2433100626038737755[52] = 0;
   out_2433100626038737755[53] = 0;
   out_2433100626038737755[54] = 0;
   out_2433100626038737755[55] = 0;
   out_2433100626038737755[56] = 0;
   out_2433100626038737755[57] = 0;
   out_2433100626038737755[58] = 0;
   out_2433100626038737755[59] = 0;
   out_2433100626038737755[60] = 1.0;
   out_2433100626038737755[61] = 0;
   out_2433100626038737755[62] = 0;
   out_2433100626038737755[63] = 0;
   out_2433100626038737755[64] = 0;
   out_2433100626038737755[65] = 0;
   out_2433100626038737755[66] = 0;
   out_2433100626038737755[67] = 0;
   out_2433100626038737755[68] = 0;
   out_2433100626038737755[69] = 0;
   out_2433100626038737755[70] = 1.0;
   out_2433100626038737755[71] = 0;
   out_2433100626038737755[72] = 0;
   out_2433100626038737755[73] = 0;
   out_2433100626038737755[74] = 0;
   out_2433100626038737755[75] = 0;
   out_2433100626038737755[76] = 0;
   out_2433100626038737755[77] = 0;
   out_2433100626038737755[78] = 0;
   out_2433100626038737755[79] = 0;
   out_2433100626038737755[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2438546081678800452) {
   out_2438546081678800452[0] = state[0];
   out_2438546081678800452[1] = state[1];
   out_2438546081678800452[2] = state[2];
   out_2438546081678800452[3] = state[3];
   out_2438546081678800452[4] = state[4];
   out_2438546081678800452[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2438546081678800452[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2438546081678800452[7] = state[7];
   out_2438546081678800452[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7089989771211385344) {
   out_7089989771211385344[0] = 1;
   out_7089989771211385344[1] = 0;
   out_7089989771211385344[2] = 0;
   out_7089989771211385344[3] = 0;
   out_7089989771211385344[4] = 0;
   out_7089989771211385344[5] = 0;
   out_7089989771211385344[6] = 0;
   out_7089989771211385344[7] = 0;
   out_7089989771211385344[8] = 0;
   out_7089989771211385344[9] = 0;
   out_7089989771211385344[10] = 1;
   out_7089989771211385344[11] = 0;
   out_7089989771211385344[12] = 0;
   out_7089989771211385344[13] = 0;
   out_7089989771211385344[14] = 0;
   out_7089989771211385344[15] = 0;
   out_7089989771211385344[16] = 0;
   out_7089989771211385344[17] = 0;
   out_7089989771211385344[18] = 0;
   out_7089989771211385344[19] = 0;
   out_7089989771211385344[20] = 1;
   out_7089989771211385344[21] = 0;
   out_7089989771211385344[22] = 0;
   out_7089989771211385344[23] = 0;
   out_7089989771211385344[24] = 0;
   out_7089989771211385344[25] = 0;
   out_7089989771211385344[26] = 0;
   out_7089989771211385344[27] = 0;
   out_7089989771211385344[28] = 0;
   out_7089989771211385344[29] = 0;
   out_7089989771211385344[30] = 1;
   out_7089989771211385344[31] = 0;
   out_7089989771211385344[32] = 0;
   out_7089989771211385344[33] = 0;
   out_7089989771211385344[34] = 0;
   out_7089989771211385344[35] = 0;
   out_7089989771211385344[36] = 0;
   out_7089989771211385344[37] = 0;
   out_7089989771211385344[38] = 0;
   out_7089989771211385344[39] = 0;
   out_7089989771211385344[40] = 1;
   out_7089989771211385344[41] = 0;
   out_7089989771211385344[42] = 0;
   out_7089989771211385344[43] = 0;
   out_7089989771211385344[44] = 0;
   out_7089989771211385344[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7089989771211385344[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7089989771211385344[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7089989771211385344[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7089989771211385344[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7089989771211385344[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7089989771211385344[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7089989771211385344[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7089989771211385344[53] = -9.8000000000000007*dt;
   out_7089989771211385344[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7089989771211385344[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7089989771211385344[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7089989771211385344[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7089989771211385344[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7089989771211385344[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7089989771211385344[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7089989771211385344[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7089989771211385344[62] = 0;
   out_7089989771211385344[63] = 0;
   out_7089989771211385344[64] = 0;
   out_7089989771211385344[65] = 0;
   out_7089989771211385344[66] = 0;
   out_7089989771211385344[67] = 0;
   out_7089989771211385344[68] = 0;
   out_7089989771211385344[69] = 0;
   out_7089989771211385344[70] = 1;
   out_7089989771211385344[71] = 0;
   out_7089989771211385344[72] = 0;
   out_7089989771211385344[73] = 0;
   out_7089989771211385344[74] = 0;
   out_7089989771211385344[75] = 0;
   out_7089989771211385344[76] = 0;
   out_7089989771211385344[77] = 0;
   out_7089989771211385344[78] = 0;
   out_7089989771211385344[79] = 0;
   out_7089989771211385344[80] = 1;
}
void h_25(double *state, double *unused, double *out_2030834903709412354) {
   out_2030834903709412354[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1367053570080972270) {
   out_1367053570080972270[0] = 0;
   out_1367053570080972270[1] = 0;
   out_1367053570080972270[2] = 0;
   out_1367053570080972270[3] = 0;
   out_1367053570080972270[4] = 0;
   out_1367053570080972270[5] = 0;
   out_1367053570080972270[6] = 1;
   out_1367053570080972270[7] = 0;
   out_1367053570080972270[8] = 0;
}
void h_24(double *state, double *unused, double *out_3656283077841470618) {
   out_3656283077841470618[0] = state[4];
   out_3656283077841470618[1] = state[5];
}
void H_24(double *state, double *unused, double *out_810160853526177703) {
   out_810160853526177703[0] = 0;
   out_810160853526177703[1] = 0;
   out_810160853526177703[2] = 0;
   out_810160853526177703[3] = 0;
   out_810160853526177703[4] = 1;
   out_810160853526177703[5] = 0;
   out_810160853526177703[6] = 0;
   out_810160853526177703[7] = 0;
   out_810160853526177703[8] = 0;
   out_810160853526177703[9] = 0;
   out_810160853526177703[10] = 0;
   out_810160853526177703[11] = 0;
   out_810160853526177703[12] = 0;
   out_810160853526177703[13] = 0;
   out_810160853526177703[14] = 1;
   out_810160853526177703[15] = 0;
   out_810160853526177703[16] = 0;
   out_810160853526177703[17] = 0;
}
void h_30(double *state, double *unused, double *out_8652643913179466821) {
   out_8652643913179466821[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1151279388426276357) {
   out_1151279388426276357[0] = 0;
   out_1151279388426276357[1] = 0;
   out_1151279388426276357[2] = 0;
   out_1151279388426276357[3] = 0;
   out_1151279388426276357[4] = 1;
   out_1151279388426276357[5] = 0;
   out_1151279388426276357[6] = 0;
   out_1151279388426276357[7] = 0;
   out_1151279388426276357[8] = 0;
}
void h_26(double *state, double *unused, double *out_3729443189915925990) {
   out_3729443189915925990[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1937472399679828331) {
   out_1937472399679828331[0] = 0;
   out_1937472399679828331[1] = 0;
   out_1937472399679828331[2] = 0;
   out_1937472399679828331[3] = 0;
   out_1937472399679828331[4] = 0;
   out_1937472399679828331[5] = 0;
   out_1937472399679828331[6] = 0;
   out_1937472399679828331[7] = 1;
   out_1937472399679828331[8] = 0;
}
void h_27(double *state, double *unused, double *out_4635240224518110005) {
   out_4635240224518110005[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1023483923374148554) {
   out_1023483923374148554[0] = 0;
   out_1023483923374148554[1] = 0;
   out_1023483923374148554[2] = 0;
   out_1023483923374148554[3] = 1;
   out_1023483923374148554[4] = 0;
   out_1023483923374148554[5] = 0;
   out_1023483923374148554[6] = 0;
   out_1023483923374148554[7] = 0;
   out_1023483923374148554[8] = 0;
}
void h_29(double *state, double *unused, double *out_4792426892869239150) {
   out_4792426892869239150[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1661510732740668541) {
   out_1661510732740668541[0] = 0;
   out_1661510732740668541[1] = 1;
   out_1661510732740668541[2] = 0;
   out_1661510732740668541[3] = 0;
   out_1661510732740668541[4] = 0;
   out_1661510732740668541[5] = 0;
   out_1661510732740668541[6] = 0;
   out_1661510732740668541[7] = 0;
   out_1661510732740668541[8] = 0;
}
void h_28(double *state, double *unused, double *out_209879309219878472) {
   out_209879309219878472[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3420888284328862033) {
   out_3420888284328862033[0] = 1;
   out_3420888284328862033[1] = 0;
   out_3420888284328862033[2] = 0;
   out_3420888284328862033[3] = 0;
   out_3420888284328862033[4] = 0;
   out_3420888284328862033[5] = 0;
   out_3420888284328862033[6] = 0;
   out_3420888284328862033[7] = 0;
   out_3420888284328862033[8] = 0;
}
void h_31(double *state, double *unused, double *out_8534636519246090077) {
   out_8534636519246090077[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1311264297446476855) {
   out_1311264297446476855[0] = 0;
   out_1311264297446476855[1] = 0;
   out_1311264297446476855[2] = 0;
   out_1311264297446476855[3] = 0;
   out_1311264297446476855[4] = 0;
   out_1311264297446476855[5] = 0;
   out_1311264297446476855[6] = 0;
   out_1311264297446476855[7] = 0;
   out_1311264297446476855[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5283071209818325702) {
  err_fun(nom_x, delta_x, out_5283071209818325702);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6148504474804144895) {
  inv_err_fun(nom_x, true_x, out_6148504474804144895);
}
void car_H_mod_fun(double *state, double *out_2433100626038737755) {
  H_mod_fun(state, out_2433100626038737755);
}
void car_f_fun(double *state, double dt, double *out_2438546081678800452) {
  f_fun(state,  dt, out_2438546081678800452);
}
void car_F_fun(double *state, double dt, double *out_7089989771211385344) {
  F_fun(state,  dt, out_7089989771211385344);
}
void car_h_25(double *state, double *unused, double *out_2030834903709412354) {
  h_25(state, unused, out_2030834903709412354);
}
void car_H_25(double *state, double *unused, double *out_1367053570080972270) {
  H_25(state, unused, out_1367053570080972270);
}
void car_h_24(double *state, double *unused, double *out_3656283077841470618) {
  h_24(state, unused, out_3656283077841470618);
}
void car_H_24(double *state, double *unused, double *out_810160853526177703) {
  H_24(state, unused, out_810160853526177703);
}
void car_h_30(double *state, double *unused, double *out_8652643913179466821) {
  h_30(state, unused, out_8652643913179466821);
}
void car_H_30(double *state, double *unused, double *out_1151279388426276357) {
  H_30(state, unused, out_1151279388426276357);
}
void car_h_26(double *state, double *unused, double *out_3729443189915925990) {
  h_26(state, unused, out_3729443189915925990);
}
void car_H_26(double *state, double *unused, double *out_1937472399679828331) {
  H_26(state, unused, out_1937472399679828331);
}
void car_h_27(double *state, double *unused, double *out_4635240224518110005) {
  h_27(state, unused, out_4635240224518110005);
}
void car_H_27(double *state, double *unused, double *out_1023483923374148554) {
  H_27(state, unused, out_1023483923374148554);
}
void car_h_29(double *state, double *unused, double *out_4792426892869239150) {
  h_29(state, unused, out_4792426892869239150);
}
void car_H_29(double *state, double *unused, double *out_1661510732740668541) {
  H_29(state, unused, out_1661510732740668541);
}
void car_h_28(double *state, double *unused, double *out_209879309219878472) {
  h_28(state, unused, out_209879309219878472);
}
void car_H_28(double *state, double *unused, double *out_3420888284328862033) {
  H_28(state, unused, out_3420888284328862033);
}
void car_h_31(double *state, double *unused, double *out_8534636519246090077) {
  h_31(state, unused, out_8534636519246090077);
}
void car_H_31(double *state, double *unused, double *out_1311264297446476855) {
  H_31(state, unused, out_1311264297446476855);
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
