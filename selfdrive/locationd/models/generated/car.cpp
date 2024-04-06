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
void err_fun(double *nom_x, double *delta_x, double *out_8428659455625657161) {
   out_8428659455625657161[0] = delta_x[0] + nom_x[0];
   out_8428659455625657161[1] = delta_x[1] + nom_x[1];
   out_8428659455625657161[2] = delta_x[2] + nom_x[2];
   out_8428659455625657161[3] = delta_x[3] + nom_x[3];
   out_8428659455625657161[4] = delta_x[4] + nom_x[4];
   out_8428659455625657161[5] = delta_x[5] + nom_x[5];
   out_8428659455625657161[6] = delta_x[6] + nom_x[6];
   out_8428659455625657161[7] = delta_x[7] + nom_x[7];
   out_8428659455625657161[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2435728419690232497) {
   out_2435728419690232497[0] = -nom_x[0] + true_x[0];
   out_2435728419690232497[1] = -nom_x[1] + true_x[1];
   out_2435728419690232497[2] = -nom_x[2] + true_x[2];
   out_2435728419690232497[3] = -nom_x[3] + true_x[3];
   out_2435728419690232497[4] = -nom_x[4] + true_x[4];
   out_2435728419690232497[5] = -nom_x[5] + true_x[5];
   out_2435728419690232497[6] = -nom_x[6] + true_x[6];
   out_2435728419690232497[7] = -nom_x[7] + true_x[7];
   out_2435728419690232497[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_817306993903023728) {
   out_817306993903023728[0] = 1.0;
   out_817306993903023728[1] = 0;
   out_817306993903023728[2] = 0;
   out_817306993903023728[3] = 0;
   out_817306993903023728[4] = 0;
   out_817306993903023728[5] = 0;
   out_817306993903023728[6] = 0;
   out_817306993903023728[7] = 0;
   out_817306993903023728[8] = 0;
   out_817306993903023728[9] = 0;
   out_817306993903023728[10] = 1.0;
   out_817306993903023728[11] = 0;
   out_817306993903023728[12] = 0;
   out_817306993903023728[13] = 0;
   out_817306993903023728[14] = 0;
   out_817306993903023728[15] = 0;
   out_817306993903023728[16] = 0;
   out_817306993903023728[17] = 0;
   out_817306993903023728[18] = 0;
   out_817306993903023728[19] = 0;
   out_817306993903023728[20] = 1.0;
   out_817306993903023728[21] = 0;
   out_817306993903023728[22] = 0;
   out_817306993903023728[23] = 0;
   out_817306993903023728[24] = 0;
   out_817306993903023728[25] = 0;
   out_817306993903023728[26] = 0;
   out_817306993903023728[27] = 0;
   out_817306993903023728[28] = 0;
   out_817306993903023728[29] = 0;
   out_817306993903023728[30] = 1.0;
   out_817306993903023728[31] = 0;
   out_817306993903023728[32] = 0;
   out_817306993903023728[33] = 0;
   out_817306993903023728[34] = 0;
   out_817306993903023728[35] = 0;
   out_817306993903023728[36] = 0;
   out_817306993903023728[37] = 0;
   out_817306993903023728[38] = 0;
   out_817306993903023728[39] = 0;
   out_817306993903023728[40] = 1.0;
   out_817306993903023728[41] = 0;
   out_817306993903023728[42] = 0;
   out_817306993903023728[43] = 0;
   out_817306993903023728[44] = 0;
   out_817306993903023728[45] = 0;
   out_817306993903023728[46] = 0;
   out_817306993903023728[47] = 0;
   out_817306993903023728[48] = 0;
   out_817306993903023728[49] = 0;
   out_817306993903023728[50] = 1.0;
   out_817306993903023728[51] = 0;
   out_817306993903023728[52] = 0;
   out_817306993903023728[53] = 0;
   out_817306993903023728[54] = 0;
   out_817306993903023728[55] = 0;
   out_817306993903023728[56] = 0;
   out_817306993903023728[57] = 0;
   out_817306993903023728[58] = 0;
   out_817306993903023728[59] = 0;
   out_817306993903023728[60] = 1.0;
   out_817306993903023728[61] = 0;
   out_817306993903023728[62] = 0;
   out_817306993903023728[63] = 0;
   out_817306993903023728[64] = 0;
   out_817306993903023728[65] = 0;
   out_817306993903023728[66] = 0;
   out_817306993903023728[67] = 0;
   out_817306993903023728[68] = 0;
   out_817306993903023728[69] = 0;
   out_817306993903023728[70] = 1.0;
   out_817306993903023728[71] = 0;
   out_817306993903023728[72] = 0;
   out_817306993903023728[73] = 0;
   out_817306993903023728[74] = 0;
   out_817306993903023728[75] = 0;
   out_817306993903023728[76] = 0;
   out_817306993903023728[77] = 0;
   out_817306993903023728[78] = 0;
   out_817306993903023728[79] = 0;
   out_817306993903023728[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5432806874620613033) {
   out_5432806874620613033[0] = state[0];
   out_5432806874620613033[1] = state[1];
   out_5432806874620613033[2] = state[2];
   out_5432806874620613033[3] = state[3];
   out_5432806874620613033[4] = state[4];
   out_5432806874620613033[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5432806874620613033[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5432806874620613033[7] = state[7];
   out_5432806874620613033[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7015879500496735849) {
   out_7015879500496735849[0] = 1;
   out_7015879500496735849[1] = 0;
   out_7015879500496735849[2] = 0;
   out_7015879500496735849[3] = 0;
   out_7015879500496735849[4] = 0;
   out_7015879500496735849[5] = 0;
   out_7015879500496735849[6] = 0;
   out_7015879500496735849[7] = 0;
   out_7015879500496735849[8] = 0;
   out_7015879500496735849[9] = 0;
   out_7015879500496735849[10] = 1;
   out_7015879500496735849[11] = 0;
   out_7015879500496735849[12] = 0;
   out_7015879500496735849[13] = 0;
   out_7015879500496735849[14] = 0;
   out_7015879500496735849[15] = 0;
   out_7015879500496735849[16] = 0;
   out_7015879500496735849[17] = 0;
   out_7015879500496735849[18] = 0;
   out_7015879500496735849[19] = 0;
   out_7015879500496735849[20] = 1;
   out_7015879500496735849[21] = 0;
   out_7015879500496735849[22] = 0;
   out_7015879500496735849[23] = 0;
   out_7015879500496735849[24] = 0;
   out_7015879500496735849[25] = 0;
   out_7015879500496735849[26] = 0;
   out_7015879500496735849[27] = 0;
   out_7015879500496735849[28] = 0;
   out_7015879500496735849[29] = 0;
   out_7015879500496735849[30] = 1;
   out_7015879500496735849[31] = 0;
   out_7015879500496735849[32] = 0;
   out_7015879500496735849[33] = 0;
   out_7015879500496735849[34] = 0;
   out_7015879500496735849[35] = 0;
   out_7015879500496735849[36] = 0;
   out_7015879500496735849[37] = 0;
   out_7015879500496735849[38] = 0;
   out_7015879500496735849[39] = 0;
   out_7015879500496735849[40] = 1;
   out_7015879500496735849[41] = 0;
   out_7015879500496735849[42] = 0;
   out_7015879500496735849[43] = 0;
   out_7015879500496735849[44] = 0;
   out_7015879500496735849[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7015879500496735849[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7015879500496735849[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7015879500496735849[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7015879500496735849[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7015879500496735849[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7015879500496735849[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7015879500496735849[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7015879500496735849[53] = -9.8000000000000007*dt;
   out_7015879500496735849[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7015879500496735849[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7015879500496735849[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7015879500496735849[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7015879500496735849[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7015879500496735849[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7015879500496735849[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7015879500496735849[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7015879500496735849[62] = 0;
   out_7015879500496735849[63] = 0;
   out_7015879500496735849[64] = 0;
   out_7015879500496735849[65] = 0;
   out_7015879500496735849[66] = 0;
   out_7015879500496735849[67] = 0;
   out_7015879500496735849[68] = 0;
   out_7015879500496735849[69] = 0;
   out_7015879500496735849[70] = 1;
   out_7015879500496735849[71] = 0;
   out_7015879500496735849[72] = 0;
   out_7015879500496735849[73] = 0;
   out_7015879500496735849[74] = 0;
   out_7015879500496735849[75] = 0;
   out_7015879500496735849[76] = 0;
   out_7015879500496735849[77] = 0;
   out_7015879500496735849[78] = 0;
   out_7015879500496735849[79] = 0;
   out_7015879500496735849[80] = 1;
}
void h_25(double *state, double *unused, double *out_7939230388010950143) {
   out_7939230388010950143[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8312742630226899182) {
   out_8312742630226899182[0] = 0;
   out_8312742630226899182[1] = 0;
   out_8312742630226899182[2] = 0;
   out_8312742630226899182[3] = 0;
   out_8312742630226899182[4] = 0;
   out_8312742630226899182[5] = 0;
   out_8312742630226899182[6] = 1;
   out_8312742630226899182[7] = 0;
   out_8312742630226899182[8] = 0;
}
void h_24(double *state, double *unused, double *out_5793576264067600587) {
   out_5793576264067600587[0] = state[4];
   out_5793576264067600587[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7961351844477152868) {
   out_7961351844477152868[0] = 0;
   out_7961351844477152868[1] = 0;
   out_7961351844477152868[2] = 0;
   out_7961351844477152868[3] = 0;
   out_7961351844477152868[4] = 1;
   out_7961351844477152868[5] = 0;
   out_7961351844477152868[6] = 0;
   out_7961351844477152868[7] = 0;
   out_7961351844477152868[8] = 0;
   out_7961351844477152868[9] = 0;
   out_7961351844477152868[10] = 0;
   out_7961351844477152868[11] = 0;
   out_7961351844477152868[12] = 0;
   out_7961351844477152868[13] = 0;
   out_7961351844477152868[14] = 1;
   out_7961351844477152868[15] = 0;
   out_7961351844477152868[16] = 0;
   out_7961351844477152868[17] = 0;
}
void h_30(double *state, double *unused, double *out_3736678459348250537) {
   out_3736678459348250537[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5794409671719650555) {
   out_5794409671719650555[0] = 0;
   out_5794409671719650555[1] = 0;
   out_5794409671719650555[2] = 0;
   out_5794409671719650555[3] = 0;
   out_5794409671719650555[4] = 1;
   out_5794409671719650555[5] = 0;
   out_5794409671719650555[6] = 0;
   out_5794409671719650555[7] = 0;
   out_5794409671719650555[8] = 0;
}
void h_26(double *state, double *unused, double *out_1264994808312595108) {
   out_1264994808312595108[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6392498124608596210) {
   out_6392498124608596210[0] = 0;
   out_6392498124608596210[1] = 0;
   out_6392498124608596210[2] = 0;
   out_6392498124608596210[3] = 0;
   out_6392498124608596210[4] = 0;
   out_6392498124608596210[5] = 0;
   out_6392498124608596210[6] = 0;
   out_6392498124608596210[7] = 1;
   out_6392498124608596210[8] = 0;
}
void h_27(double *state, double *unused, double *out_8817065850962920513) {
   out_8817065850962920513[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3570815600535707338) {
   out_3570815600535707338[0] = 0;
   out_3570815600535707338[1] = 0;
   out_3570815600535707338[2] = 0;
   out_3570815600535707338[3] = 1;
   out_3570815600535707338[4] = 0;
   out_3570815600535707338[5] = 0;
   out_3570815600535707338[6] = 0;
   out_3570815600535707338[7] = 0;
   out_3570815600535707338[8] = 0;
}
void h_29(double *state, double *unused, double *out_3474231026905970974) {
   out_3474231026905970974[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5284178327405258371) {
   out_5284178327405258371[0] = 0;
   out_5284178327405258371[1] = 1;
   out_5284178327405258371[2] = 0;
   out_5284178327405258371[3] = 0;
   out_5284178327405258371[4] = 0;
   out_5284178327405258371[5] = 0;
   out_5284178327405258371[6] = 0;
   out_5284178327405258371[7] = 0;
   out_5284178327405258371[8] = 0;
}
void h_28(double *state, double *unused, double *out_8279424356236135061) {
   out_8279424356236135061[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8080166729234762671) {
   out_8080166729234762671[0] = 1;
   out_8080166729234762671[1] = 0;
   out_8080166729234762671[2] = 0;
   out_8080166729234762671[3] = 0;
   out_8080166729234762671[4] = 0;
   out_8080166729234762671[5] = 0;
   out_8080166729234762671[6] = 0;
   out_8080166729234762671[7] = 0;
   out_8080166729234762671[8] = 0;
}
void h_31(double *state, double *unused, double *out_46290893126426784) {
   out_46290893126426784[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5766290022375244734) {
   out_5766290022375244734[0] = 0;
   out_5766290022375244734[1] = 0;
   out_5766290022375244734[2] = 0;
   out_5766290022375244734[3] = 0;
   out_5766290022375244734[4] = 0;
   out_5766290022375244734[5] = 0;
   out_5766290022375244734[6] = 0;
   out_5766290022375244734[7] = 0;
   out_5766290022375244734[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8428659455625657161) {
  err_fun(nom_x, delta_x, out_8428659455625657161);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2435728419690232497) {
  inv_err_fun(nom_x, true_x, out_2435728419690232497);
}
void car_H_mod_fun(double *state, double *out_817306993903023728) {
  H_mod_fun(state, out_817306993903023728);
}
void car_f_fun(double *state, double dt, double *out_5432806874620613033) {
  f_fun(state,  dt, out_5432806874620613033);
}
void car_F_fun(double *state, double dt, double *out_7015879500496735849) {
  F_fun(state,  dt, out_7015879500496735849);
}
void car_h_25(double *state, double *unused, double *out_7939230388010950143) {
  h_25(state, unused, out_7939230388010950143);
}
void car_H_25(double *state, double *unused, double *out_8312742630226899182) {
  H_25(state, unused, out_8312742630226899182);
}
void car_h_24(double *state, double *unused, double *out_5793576264067600587) {
  h_24(state, unused, out_5793576264067600587);
}
void car_H_24(double *state, double *unused, double *out_7961351844477152868) {
  H_24(state, unused, out_7961351844477152868);
}
void car_h_30(double *state, double *unused, double *out_3736678459348250537) {
  h_30(state, unused, out_3736678459348250537);
}
void car_H_30(double *state, double *unused, double *out_5794409671719650555) {
  H_30(state, unused, out_5794409671719650555);
}
void car_h_26(double *state, double *unused, double *out_1264994808312595108) {
  h_26(state, unused, out_1264994808312595108);
}
void car_H_26(double *state, double *unused, double *out_6392498124608596210) {
  H_26(state, unused, out_6392498124608596210);
}
void car_h_27(double *state, double *unused, double *out_8817065850962920513) {
  h_27(state, unused, out_8817065850962920513);
}
void car_H_27(double *state, double *unused, double *out_3570815600535707338) {
  H_27(state, unused, out_3570815600535707338);
}
void car_h_29(double *state, double *unused, double *out_3474231026905970974) {
  h_29(state, unused, out_3474231026905970974);
}
void car_H_29(double *state, double *unused, double *out_5284178327405258371) {
  H_29(state, unused, out_5284178327405258371);
}
void car_h_28(double *state, double *unused, double *out_8279424356236135061) {
  h_28(state, unused, out_8279424356236135061);
}
void car_H_28(double *state, double *unused, double *out_8080166729234762671) {
  H_28(state, unused, out_8080166729234762671);
}
void car_h_31(double *state, double *unused, double *out_46290893126426784) {
  h_31(state, unused, out_46290893126426784);
}
void car_H_31(double *state, double *unused, double *out_5766290022375244734) {
  H_31(state, unused, out_5766290022375244734);
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
