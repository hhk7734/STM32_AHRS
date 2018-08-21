/**
 * filename : EKF.cpp
 *
 * created  : 2018/04/13
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : Extended Kalman Filter for Attitude and Heading Reference System
 */

#include "EKF.h"
#include <math.h>

//
// Static Variables initialization
// type EKF_::variables = init;
//

//
// Constructor & Destructor
//
EKF_::EKF_()
{
   // variables initialization
}
EKF_::~EKF_() {}

//
// Public
//
void EKF_::predict(float *half_delta_angle)
{
    //
    // x_(k|k-1) = f(x_(k-1|k-1))
    //
    float delta_quaternion[4];
    delta_quaternion[EKF_X] =   half_delta_angle[2] * quaternion[1]
                          - half_delta_angle[1] * quaternion[2]
                          + half_delta_angle[0] * quaternion[3];
    
    delta_quaternion[EKF_Y] = - half_delta_angle[2] * quaternion[0]
                          + half_delta_angle[0] * quaternion[2]
                          + half_delta_angle[1] * quaternion[3];
    
    delta_quaternion[EKF_Z] =   half_delta_angle[1] * quaternion[0]
                          - half_delta_angle[0] * quaternion[1]
                          + half_delta_angle[2] * quaternion[3];
    
    delta_quaternion[EKF_W] = - half_delta_angle[0] * quaternion[0]
                          - half_delta_angle[1] * quaternion[1]
                          - half_delta_angle[2] * quaternion[2];

    for (int8_t i = 0; i < 4; ++i)
    {
        quaternion[i] += delta_quaternion[i];
    }
    
    //
    // F = df/dx
    //
    F[0][0] =   1.0;                 F[0][1] =   half_delta_angle[2]; F[0][2] = - half_delta_angle[1]; F[0][3] = half_delta_angle[0];
    F[1][0] = - half_delta_angle[2]; F[1][1] =   1.0;                 F[1][2] =   half_delta_angle[0]; F[1][3] = half_delta_angle[1];
    F[2][0] =   half_delta_angle[1]; F[2][1] = - half_delta_angle[0]; F[2][2] =   1.0;                 F[2][3] = half_delta_angle[2];
    F[3][0] = - half_delta_angle[0]; F[3][1] = - half_delta_angle[1]; F[3][2] = - half_delta_angle[2]; F[3][3] =   1.0;

    //
    // P_(k|k-1) = F*P_(k-1|k-1)*F^T + Q
    // K is used instead of temp array
    //
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            K[i][j] = F[i][0]*P[0][j] + F[i][1]*P[1][j] + F[i][2]*P[2][j] + F[i][3]*P[3][j];
        }
    }
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            P[i][j] = K[i][0]*F[j][0] + K[i][1]*F[j][1] + K[i][2]*F[j][2] + K[i][3]*F[j][3];
        }
        P[i][i] += EKF_Q_ROT;
    }

    float quaternion_norm = sqrt(quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
    for (uint8_t i = 0; i < 4; ++i)
    {
        quaternion[i] /= quaternion_norm;
    }
}

void EKF_::update(float *unit_acc, float *unit_a_m)
{
    //
    // y = z - h(x_(k|k-1))
    //
    y[0] = unit_acc[0] - 2*(quaternion[EKF_X]*quaternion[EKF_Z] - quaternion[EKF_Y]*quaternion[EKF_W]);
    y[1] = unit_acc[1] - 2*(quaternion[EKF_X]*quaternion[EKF_W] + quaternion[EKF_Y]*quaternion[EKF_Z]);
    y[2] = unit_acc[2] - (1 - 2*(quaternion[EKF_X]*quaternion[EKF_X] + quaternion[EKF_Y]*quaternion[EKF_Y]));
    y[3] = unit_a_m[0] - 2*(quaternion[EKF_X]*quaternion[EKF_Y] + quaternion[EKF_Z]*quaternion[EKF_W]);
    y[4] = unit_a_m[1] - (1 - 2*(quaternion[EKF_X]*quaternion[EKF_X] + quaternion[EKF_Z]*quaternion[EKF_Z]));
    y[5] = unit_a_m[2] - 2*(quaternion[EKF_Y]*quaternion[EKF_Z] - quaternion[EKF_X]*quaternion[EKF_W]);

    //
    // H = dh/dx
    //
    H[0][0] =   2*quaternion[EKF_Z];    H[0][1] = - 2*quaternion[EKF_W];    H[0][2] =   2*quaternion[EKF_X];    H[0][3] = - 2*quaternion[EKF_Y];
    H[1][0] = - H[0][1];                H[1][1] =   H[0][0];                H[1][2] = - H[0][3];                H[1][3] =   H[0][2];
    H[2][0] = - H[0][2];                H[2][1] =   H[0][3];                H[2][2] =   H[0][0];                H[2][3] =   H[1][0];
    H[3][0] =   H[1][2];                H[3][1] =   H[0][2];                H[3][2] =   H[1][0];                H[3][3] =   H[0][0];
    H[4][0] =   H[2][0];                H[4][1] =   H[1][2];                H[4][2] = - H[0][0];                H[4][3] =   H[1][0];
    H[5][0] =   H[0][1];                H[5][1] =   H[0][0];                H[5][2] =   H[1][2];                H[5][3] =   H[2][0];
    
    //
    // S = H*P_(k|k-1)*(H^T) + R
    // K is used instead of P_(k|k-1)*(H^T)
    //
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 6; ++j)
        {
            K[i][j] = P[i][0]*H[j][0] + P[i][1]*H[j][1] + P[i][2]*H[j][2] + P[i][3]*H[j][3];
        }
    }
    for (uint8_t i = 0; i < 6; ++i)
    {
        for (uint8_t j = 0; j < 6; ++j)
        {
            S[i][j] = H[i][0]*K[0][j] + H[i][1]*K[1][j] + H[i][2]*K[2][j] + H[i][3]*K[3][j];
        }
        if(i < 3) S[i][i] += EKF_R_ACC;
        else      S[i][i] += EKF_R_MAG; 
    }

    //
    // K = P_(k|k-1)*(H^T)*(S^-1)
    // K*S = P_(k|k-1)*(H^T)
    //
    // Cholesky decomposition S=L*(L^*T)  (L^*T == conjugate transpose L)
    // S is used instead of L
    //
    for (uint8_t i = 0; i < 6; ++i)
    {
        for (uint8_t j = 0; j < i; ++j)
        {
            for (uint8_t k = 0; k < j; ++k)
            {
                S[i][j] -= S[i][k]*S[j][k];
            }
            S[i][j] /= S[j][j];
            
            S[i][i] -= S[i][j]*S[i][j];
        }
        S[i][i] = sqrt(S[i][i]);
    }
    // L*(L^*T)*(K^T) = P_(k|k-1)*(H^T)
    // L*A = P_(k|k-1)*(H^T)
    // K is used instead of A
    for (uint8_t k = 0; k < 4; ++k)
    {
        for (uint8_t i = 0; i < 6; ++i)
        {
            for (uint8_t j = 0; j < i; ++j)
            {
                K[k][i] -= S[i][j]*K[k][j];
            }
            K[k][i] /= S[i][i];
        }
    }
    //(L^*T)*(K^T) = A
    for (uint8_t k = 0; k < 4; ++k)
    {
        for (uint8_t i = 0; i < 6; ++i)
        {
            for (uint8_t j = 0; j < i; ++j)
            {
                K[k][5-i] -= S[5-j][5-i]*K[k][5-j];
            }
            K[k][5-i] /= S[5-i][5-i];
        }
    }

    //
    //  x = x + K*y
    //
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 6; ++j)
        {
            quaternion[i] += K[i][j]*y[j];
        }
    }
    float quaternion_norm = sqrt(quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
    for (uint8_t i = 0; i < 4; ++i)
    {
        quaternion[i] /= quaternion_norm;
    }

    //
    // P_(k|k) = (I-K*H)*P_(k|k-1)
    // F is used instead of temp array
    //
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            F[i][j] = K[i][0]*H[0][j] + K[i][1]*H[1][j] + K[i][2]*H[2][j] + K[i][3]*H[3][j];
        }
    }
    // K is used instead of temp array
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            K[i][j] = F[i][0]*P[0][j] + F[i][1]*P[1][j] + F[i][2]*P[2][j] + F[i][3]*P[3][j];
        }
    }
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            P[i][j] -= K[i][j];
        }
    }
}

void EKF_::get_quat(float *quat)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        quat[i]=quaternion[i];
    }
}

//
// Protected
//

//
// Private
//