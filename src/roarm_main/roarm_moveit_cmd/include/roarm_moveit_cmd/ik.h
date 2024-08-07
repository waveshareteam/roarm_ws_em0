#include <memory>
#include <math.h>

#define ARM_L1_LENGTH_MM    126.06
#define ARM_L2_LENGTH_MM_A  236.82
#define ARM_L2_LENGTH_MM_B	30.00 
#define ARM_L3_LENGTH_MM_A_0	280.15 //214 
#define ARM_L3_LENGTH_MM_B_0	1.73

double l1  = ARM_L1_LENGTH_MM;
double l2A = ARM_L2_LENGTH_MM_A;
double l2B = ARM_L2_LENGTH_MM_B;
double l2  = sqrt(l2A * l2A + l2B * l2B);
double t2rad = atan2(l2B, l2A);
double l3A = ARM_L3_LENGTH_MM_A_0;
double l3B = ARM_L3_LENGTH_MM_B_0;
double l3  = sqrt(l3A * l3A + l3B * l3B);
double t3rad = atan2(l3B, l3A);

double BASE_point_RAD = 0;
double SHOULDER_point_RAD = 0;
double ELBOW_point_RAD = M_PI/2;
double EOAT_point_RAD = M_PI;
double EOAT_point_RAD_BUFFER;

double BASE_point_ANG  = 0;
double SHOULDER_point_ANG = 0;
double ELBOW_point_ANG = 90.0;
double EOAT_point_ANG  = 180.0;

bool nanIK;
bool nanFK;
double base_r;
// Simple Linkage IK:
// input the position of the end and return angle.
//   O----O
//  /
// O
// ---------------------------------------------------
// |       /beta           /delta                    |
//        O----LB---------X------                    |
// |     /       omega.   |       \LB                |
//      LA        .                < ----------------|
// |alpha     .          bIn         \LB -EP  <delta |
//    /psi.                           \LB -EP        |
// | /.   lambda          |                          |
// O- - - - - aIn - - - - X -                        |
// ---------------------------------------------------
// alpha, beta > 0 ; delta <= 0 ; aIn, bIn > 0
void simpleLinkageIkRad(double LA, double LB, double aIn, double bIn) {
  double psi, alpha, omega, beta, L2C, LC, lambda, delta;

  if (fabs(bIn) < 1e-6) {
    psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
    alpha = M_PI / 2.0 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
    beta = psi + omega - t3rad;
  } else {
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan2(bIn, aIn);
    psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
    alpha = M_PI / 2.0 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
    beta = psi + omega - t3rad;
  }

  delta = M_PI / 2.0 - alpha - beta;

  SHOULDER_point_RAD = alpha;
  ELBOW_point_RAD    = beta;
  EOAT_point_RAD_BUFFER  = delta;

  nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
}

void cartesian_to_polar(double x, double y, double* r, double* theta) {
    *r = sqrt(x * x + y * y);
    *theta = atan2(y, x);
}


void simpleLinkagefkRad(double LA, double LB, double aIn, double bIn) {
  double psi, alpha, omega, beta, L2C, LC, lambda, delta;

  if (fabs(bIn) < 1e-6) {
    psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
    alpha = M_PI / 2.0 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
    beta = psi + omega - t3rad;
  } else {
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan2(bIn, aIn);
    psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
    alpha = M_PI / 2.0 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
    beta = psi + omega - t3rad;
  }

  delta = M_PI / 2.0 - alpha - beta;

  SHOULDER_point_RAD = alpha;
  ELBOW_point_RAD    = beta;
  EOAT_point_RAD_BUFFER  = delta;

  nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
}

