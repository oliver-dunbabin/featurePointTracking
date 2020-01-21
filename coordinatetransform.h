#ifndef COORDINATETRANSFORM_H
#define COORDINATETRANSFORM_H

#include <math.h>
#include <string>

/**
 * DCMs, euler angles, and quaternions define a rotation from frame A to
 * frame B. For example; a motion capture quaternion defines a rotation
 * FROM the local INERTIAL frame TO the BODY frame.
 *
 * The coorTrans functions transform a quaternion q1 (which describes
 * rotation from local inertial frame A to body frame B1) to quaternion q
 * (which describes rotation from local inertial frame A to body frame B2).
 * That is, a rotation from A -> B1 and followed by a rotation from B1 -> B2.
 */

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void dcm2quat(double R[3][3], double q[4]);

void euler2dcm321(double *phi, double *theta, double *psi, double R[3][3]);

void euler2quat321(double *phi, double *theta, double *psi, double q[4]);

void quat2dcm(double q[4], double R[3][3]);

void dcm2euler321(double R[3][3], double *phi, double *theta, double *psi);

void quat2euler321(double q[4], double *phi, double *theta, double *psi);

void quatmult(double q1[4], double q2[4], double q[4]);

void quatinv(double q1[4], double q[4]);

void poseTrans(std::string orientation, double q[], double pos[3], double corrq[], double corrpos[3]);

void posTrans(std::string orientation, double pos[3], double corrpos[3]);

void attTrans(std::string orientation, double q1[4], double q[4]);

void attTransCorr(std::string orientation, double q1[4], double *dpsi, double q[4]);


#endif // COORDINATETRANSFORM_H
