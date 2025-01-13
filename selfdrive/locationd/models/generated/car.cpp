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
void err_fun(double *nom_x, double *delta_x, double *out_2424299756328140598) {
   out_2424299756328140598[0] = delta_x[0] + nom_x[0];
   out_2424299756328140598[1] = delta_x[1] + nom_x[1];
   out_2424299756328140598[2] = delta_x[2] + nom_x[2];
   out_2424299756328140598[3] = delta_x[3] + nom_x[3];
   out_2424299756328140598[4] = delta_x[4] + nom_x[4];
   out_2424299756328140598[5] = delta_x[5] + nom_x[5];
   out_2424299756328140598[6] = delta_x[6] + nom_x[6];
   out_2424299756328140598[7] = delta_x[7] + nom_x[7];
   out_2424299756328140598[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9107298749992071872) {
   out_9107298749992071872[0] = -nom_x[0] + true_x[0];
   out_9107298749992071872[1] = -nom_x[1] + true_x[1];
   out_9107298749992071872[2] = -nom_x[2] + true_x[2];
   out_9107298749992071872[3] = -nom_x[3] + true_x[3];
   out_9107298749992071872[4] = -nom_x[4] + true_x[4];
   out_9107298749992071872[5] = -nom_x[5] + true_x[5];
   out_9107298749992071872[6] = -nom_x[6] + true_x[6];
   out_9107298749992071872[7] = -nom_x[7] + true_x[7];
   out_9107298749992071872[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_371992781021667047) {
   out_371992781021667047[0] = 1.0;
   out_371992781021667047[1] = 0;
   out_371992781021667047[2] = 0;
   out_371992781021667047[3] = 0;
   out_371992781021667047[4] = 0;
   out_371992781021667047[5] = 0;
   out_371992781021667047[6] = 0;
   out_371992781021667047[7] = 0;
   out_371992781021667047[8] = 0;
   out_371992781021667047[9] = 0;
   out_371992781021667047[10] = 1.0;
   out_371992781021667047[11] = 0;
   out_371992781021667047[12] = 0;
   out_371992781021667047[13] = 0;
   out_371992781021667047[14] = 0;
   out_371992781021667047[15] = 0;
   out_371992781021667047[16] = 0;
   out_371992781021667047[17] = 0;
   out_371992781021667047[18] = 0;
   out_371992781021667047[19] = 0;
   out_371992781021667047[20] = 1.0;
   out_371992781021667047[21] = 0;
   out_371992781021667047[22] = 0;
   out_371992781021667047[23] = 0;
   out_371992781021667047[24] = 0;
   out_371992781021667047[25] = 0;
   out_371992781021667047[26] = 0;
   out_371992781021667047[27] = 0;
   out_371992781021667047[28] = 0;
   out_371992781021667047[29] = 0;
   out_371992781021667047[30] = 1.0;
   out_371992781021667047[31] = 0;
   out_371992781021667047[32] = 0;
   out_371992781021667047[33] = 0;
   out_371992781021667047[34] = 0;
   out_371992781021667047[35] = 0;
   out_371992781021667047[36] = 0;
   out_371992781021667047[37] = 0;
   out_371992781021667047[38] = 0;
   out_371992781021667047[39] = 0;
   out_371992781021667047[40] = 1.0;
   out_371992781021667047[41] = 0;
   out_371992781021667047[42] = 0;
   out_371992781021667047[43] = 0;
   out_371992781021667047[44] = 0;
   out_371992781021667047[45] = 0;
   out_371992781021667047[46] = 0;
   out_371992781021667047[47] = 0;
   out_371992781021667047[48] = 0;
   out_371992781021667047[49] = 0;
   out_371992781021667047[50] = 1.0;
   out_371992781021667047[51] = 0;
   out_371992781021667047[52] = 0;
   out_371992781021667047[53] = 0;
   out_371992781021667047[54] = 0;
   out_371992781021667047[55] = 0;
   out_371992781021667047[56] = 0;
   out_371992781021667047[57] = 0;
   out_371992781021667047[58] = 0;
   out_371992781021667047[59] = 0;
   out_371992781021667047[60] = 1.0;
   out_371992781021667047[61] = 0;
   out_371992781021667047[62] = 0;
   out_371992781021667047[63] = 0;
   out_371992781021667047[64] = 0;
   out_371992781021667047[65] = 0;
   out_371992781021667047[66] = 0;
   out_371992781021667047[67] = 0;
   out_371992781021667047[68] = 0;
   out_371992781021667047[69] = 0;
   out_371992781021667047[70] = 1.0;
   out_371992781021667047[71] = 0;
   out_371992781021667047[72] = 0;
   out_371992781021667047[73] = 0;
   out_371992781021667047[74] = 0;
   out_371992781021667047[75] = 0;
   out_371992781021667047[76] = 0;
   out_371992781021667047[77] = 0;
   out_371992781021667047[78] = 0;
   out_371992781021667047[79] = 0;
   out_371992781021667047[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3910341159372398453) {
   out_3910341159372398453[0] = state[0];
   out_3910341159372398453[1] = state[1];
   out_3910341159372398453[2] = state[2];
   out_3910341159372398453[3] = state[3];
   out_3910341159372398453[4] = state[4];
   out_3910341159372398453[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3910341159372398453[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3910341159372398453[7] = state[7];
   out_3910341159372398453[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2495385078796925409) {
   out_2495385078796925409[0] = 1;
   out_2495385078796925409[1] = 0;
   out_2495385078796925409[2] = 0;
   out_2495385078796925409[3] = 0;
   out_2495385078796925409[4] = 0;
   out_2495385078796925409[5] = 0;
   out_2495385078796925409[6] = 0;
   out_2495385078796925409[7] = 0;
   out_2495385078796925409[8] = 0;
   out_2495385078796925409[9] = 0;
   out_2495385078796925409[10] = 1;
   out_2495385078796925409[11] = 0;
   out_2495385078796925409[12] = 0;
   out_2495385078796925409[13] = 0;
   out_2495385078796925409[14] = 0;
   out_2495385078796925409[15] = 0;
   out_2495385078796925409[16] = 0;
   out_2495385078796925409[17] = 0;
   out_2495385078796925409[18] = 0;
   out_2495385078796925409[19] = 0;
   out_2495385078796925409[20] = 1;
   out_2495385078796925409[21] = 0;
   out_2495385078796925409[22] = 0;
   out_2495385078796925409[23] = 0;
   out_2495385078796925409[24] = 0;
   out_2495385078796925409[25] = 0;
   out_2495385078796925409[26] = 0;
   out_2495385078796925409[27] = 0;
   out_2495385078796925409[28] = 0;
   out_2495385078796925409[29] = 0;
   out_2495385078796925409[30] = 1;
   out_2495385078796925409[31] = 0;
   out_2495385078796925409[32] = 0;
   out_2495385078796925409[33] = 0;
   out_2495385078796925409[34] = 0;
   out_2495385078796925409[35] = 0;
   out_2495385078796925409[36] = 0;
   out_2495385078796925409[37] = 0;
   out_2495385078796925409[38] = 0;
   out_2495385078796925409[39] = 0;
   out_2495385078796925409[40] = 1;
   out_2495385078796925409[41] = 0;
   out_2495385078796925409[42] = 0;
   out_2495385078796925409[43] = 0;
   out_2495385078796925409[44] = 0;
   out_2495385078796925409[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2495385078796925409[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2495385078796925409[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2495385078796925409[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2495385078796925409[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2495385078796925409[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2495385078796925409[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2495385078796925409[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2495385078796925409[53] = -9.8000000000000007*dt;
   out_2495385078796925409[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2495385078796925409[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2495385078796925409[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2495385078796925409[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2495385078796925409[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2495385078796925409[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2495385078796925409[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2495385078796925409[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2495385078796925409[62] = 0;
   out_2495385078796925409[63] = 0;
   out_2495385078796925409[64] = 0;
   out_2495385078796925409[65] = 0;
   out_2495385078796925409[66] = 0;
   out_2495385078796925409[67] = 0;
   out_2495385078796925409[68] = 0;
   out_2495385078796925409[69] = 0;
   out_2495385078796925409[70] = 1;
   out_2495385078796925409[71] = 0;
   out_2495385078796925409[72] = 0;
   out_2495385078796925409[73] = 0;
   out_2495385078796925409[74] = 0;
   out_2495385078796925409[75] = 0;
   out_2495385078796925409[76] = 0;
   out_2495385078796925409[77] = 0;
   out_2495385078796925409[78] = 0;
   out_2495385078796925409[79] = 0;
   out_2495385078796925409[80] = 1;
}
void h_25(double *state, double *unused, double *out_5748993800625767294) {
   out_5748993800625767294[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7053231232002920988) {
   out_7053231232002920988[0] = 0;
   out_7053231232002920988[1] = 0;
   out_7053231232002920988[2] = 0;
   out_7053231232002920988[3] = 0;
   out_7053231232002920988[4] = 0;
   out_7053231232002920988[5] = 0;
   out_7053231232002920988[6] = 1;
   out_7053231232002920988[7] = 0;
   out_7053231232002920988[8] = 0;
}
void h_24(double *state, double *unused, double *out_7289435458421562664) {
   out_7289435458421562664[0] = state[4];
   out_7289435458421562664[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4817941035115112527) {
   out_4817941035115112527[0] = 0;
   out_4817941035115112527[1] = 0;
   out_4817941035115112527[2] = 0;
   out_4817941035115112527[3] = 0;
   out_4817941035115112527[4] = 1;
   out_4817941035115112527[5] = 0;
   out_4817941035115112527[6] = 0;
   out_4817941035115112527[7] = 0;
   out_4817941035115112527[8] = 0;
   out_4817941035115112527[9] = 0;
   out_4817941035115112527[10] = 0;
   out_4817941035115112527[11] = 0;
   out_4817941035115112527[12] = 0;
   out_4817941035115112527[13] = 0;
   out_4817941035115112527[14] = 1;
   out_4817941035115112527[15] = 0;
   out_4817941035115112527[16] = 0;
   out_4817941035115112527[17] = 0;
}
void h_30(double *state, double *unused, double *out_2111544272371744944) {
   out_2111544272371744944[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4476822500215013873) {
   out_4476822500215013873[0] = 0;
   out_4476822500215013873[1] = 0;
   out_4476822500215013873[2] = 0;
   out_4476822500215013873[3] = 0;
   out_4476822500215013873[4] = 1;
   out_4476822500215013873[5] = 0;
   out_4476822500215013873[6] = 0;
   out_4476822500215013873[7] = 0;
   out_4476822500215013873[8] = 0;
}
void h_26(double *state, double *unused, double *out_492861048436952707) {
   out_492861048436952707[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3311727913128864764) {
   out_3311727913128864764[0] = 0;
   out_3311727913128864764[1] = 0;
   out_3311727913128864764[2] = 0;
   out_3311727913128864764[3] = 0;
   out_3311727913128864764[4] = 0;
   out_3311727913128864764[5] = 0;
   out_3311727913128864764[6] = 0;
   out_3311727913128864764[7] = 1;
   out_3311727913128864764[8] = 0;
}
void h_27(double *state, double *unused, double *out_2614934933004594812) {
   out_2614934933004594812[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6651585812015438784) {
   out_6651585812015438784[0] = 0;
   out_6651585812015438784[1] = 0;
   out_6651585812015438784[2] = 0;
   out_6651585812015438784[3] = 1;
   out_6651585812015438784[4] = 0;
   out_6651585812015438784[5] = 0;
   out_6651585812015438784[6] = 0;
   out_6651585812015438784[7] = 0;
   out_6651585812015438784[8] = 0;
}
void h_29(double *state, double *unused, double *out_2890128995289100701) {
   out_2890128995289100701[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8364948538884989817) {
   out_8364948538884989817[0] = 0;
   out_8364948538884989817[1] = 1;
   out_8364948538884989817[2] = 0;
   out_8364948538884989817[3] = 0;
   out_8364948538884989817[4] = 0;
   out_8364948538884989817[5] = 0;
   out_8364948538884989817[6] = 0;
   out_8364948538884989817[7] = 0;
   out_8364948538884989817[8] = 0;
}
void h_28(double *state, double *unused, double *out_8717454368737789202) {
   out_8717454368737789202[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4999396517755031225) {
   out_4999396517755031225[0] = 1;
   out_4999396517755031225[1] = 0;
   out_4999396517755031225[2] = 0;
   out_4999396517755031225[3] = 0;
   out_4999396517755031225[4] = 0;
   out_4999396517755031225[5] = 0;
   out_4999396517755031225[6] = 0;
   out_4999396517755031225[7] = 0;
   out_4999396517755031225[8] = 0;
}
void h_31(double *state, double *unused, double *out_1711564963623121031) {
   out_1711564963623121031[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7083877193879881416) {
   out_7083877193879881416[0] = 0;
   out_7083877193879881416[1] = 0;
   out_7083877193879881416[2] = 0;
   out_7083877193879881416[3] = 0;
   out_7083877193879881416[4] = 0;
   out_7083877193879881416[5] = 0;
   out_7083877193879881416[6] = 0;
   out_7083877193879881416[7] = 0;
   out_7083877193879881416[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2424299756328140598) {
  err_fun(nom_x, delta_x, out_2424299756328140598);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9107298749992071872) {
  inv_err_fun(nom_x, true_x, out_9107298749992071872);
}
void car_H_mod_fun(double *state, double *out_371992781021667047) {
  H_mod_fun(state, out_371992781021667047);
}
void car_f_fun(double *state, double dt, double *out_3910341159372398453) {
  f_fun(state,  dt, out_3910341159372398453);
}
void car_F_fun(double *state, double dt, double *out_2495385078796925409) {
  F_fun(state,  dt, out_2495385078796925409);
}
void car_h_25(double *state, double *unused, double *out_5748993800625767294) {
  h_25(state, unused, out_5748993800625767294);
}
void car_H_25(double *state, double *unused, double *out_7053231232002920988) {
  H_25(state, unused, out_7053231232002920988);
}
void car_h_24(double *state, double *unused, double *out_7289435458421562664) {
  h_24(state, unused, out_7289435458421562664);
}
void car_H_24(double *state, double *unused, double *out_4817941035115112527) {
  H_24(state, unused, out_4817941035115112527);
}
void car_h_30(double *state, double *unused, double *out_2111544272371744944) {
  h_30(state, unused, out_2111544272371744944);
}
void car_H_30(double *state, double *unused, double *out_4476822500215013873) {
  H_30(state, unused, out_4476822500215013873);
}
void car_h_26(double *state, double *unused, double *out_492861048436952707) {
  h_26(state, unused, out_492861048436952707);
}
void car_H_26(double *state, double *unused, double *out_3311727913128864764) {
  H_26(state, unused, out_3311727913128864764);
}
void car_h_27(double *state, double *unused, double *out_2614934933004594812) {
  h_27(state, unused, out_2614934933004594812);
}
void car_H_27(double *state, double *unused, double *out_6651585812015438784) {
  H_27(state, unused, out_6651585812015438784);
}
void car_h_29(double *state, double *unused, double *out_2890128995289100701) {
  h_29(state, unused, out_2890128995289100701);
}
void car_H_29(double *state, double *unused, double *out_8364948538884989817) {
  H_29(state, unused, out_8364948538884989817);
}
void car_h_28(double *state, double *unused, double *out_8717454368737789202) {
  h_28(state, unused, out_8717454368737789202);
}
void car_H_28(double *state, double *unused, double *out_4999396517755031225) {
  H_28(state, unused, out_4999396517755031225);
}
void car_h_31(double *state, double *unused, double *out_1711564963623121031) {
  h_31(state, unused, out_1711564963623121031);
}
void car_H_31(double *state, double *unused, double *out_7083877193879881416) {
  H_31(state, unused, out_7083877193879881416);
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
