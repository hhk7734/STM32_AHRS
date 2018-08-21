/**
 * filename : EKF.h
 *
 * created  : 2018/04/13
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : Extended Kalman Filter for Attitude and Heading Reference System
 */

/**
 * Tait-Bryan angles Z-Y-X rotation
 * roll  = atan2( 2*(qy*qz+qx*qw) , 1-2*(qx^2+qy^2) )
 * pitch = asin ( 2*(qy*qw-qx*qz) )
 * yaw   = atan2( 2*(qx*qy+qz*qw) , 1-2*(qy^2+qz^2) )
 */

#ifndef _EKF_H_
#define _EKF_H_

#include <Arduino.h>

#define EKF_X 0
#define EKF_Y 1
#define EKF_Z 2
#define EKF_W 3

#define EKF_Q_ROT 0.1f
#define EKF_R_ACC 200.0f
#define EKF_R_MAG 2000.0f

class EKF_
{
public :
    EKF_();
    ~EKF_();
    void predict(float *half_delta_angle);
    void update(float *unit_acc, float *unit_a_m); // a_m is acc X mag
    void get_quat(float *quat);

protected :

private :
    float quaternion[4] = {0.0, 0.0, 0.0, 1.0};
    float P[4][4]={{1.0, 0.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0, 0.0},
                    {0.0, 0.0, 1.0, 0.0},
                    {0.0, 0.0, 0.0, 1.0}};
    float F[4][4];
    float y[6];
    float H[6][4];
    float S[6][6];
    float K[4][6];
}; // EKF_

#endif // _EKF_H_
