/* This file has been autogenerated by Ivory
 * Compiler version  0.1.0.2
 */
#ifndef __SMACCM_INS_H__
#define __SMACCM_INS_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "ivory.h"
struct kalman_state {
    double orient[4U];
    double vel[3U];
    double pos[3U];
    double gyro_bias[3U];
    double wind[3U];
    double mag_ned[3U];
    double mag_xyz[3U];
} __attribute__((__packed__));
struct kalman_covariance {
    struct kalman_state cov_orient[4U];
    struct kalman_state cov_vel[3U];
    struct kalman_state cov_pos[3U];
    struct kalman_state cov_gyro_bias[3U];
    struct kalman_state cov_wind[3U];
    struct kalman_state cov_mag_ned[3U];
    struct kalman_state cov_mag_xyz[3U];
} __attribute__((__packed__));
extern struct kalman_state kalman_state;
extern struct kalman_covariance kalman_covariance;
void kalman_init(double n_var0, double n_var1, double n_var2, double n_var3,
                 double n_var4, double n_var5, double n_var6);
void kalman_predict(double n_var0, double n_var1, double n_var2, double n_var3,
                    double n_var4, double n_var5, double n_var6);
void vel_measure(double n_var0, double n_var1, double n_var2);
void pos_measure(double n_var0, double n_var1, double n_var2);
void pressure_measure(double n_var0);
void tas_measure(double n_var0);
void mag_measure(double n_var0, double n_var1, double n_var2);

#ifdef __cplusplus
}
#endif
#endif /* __SMACCM_INS_H__ */