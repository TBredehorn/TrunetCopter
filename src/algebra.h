#ifndef ALGEBRA_H_
#define ALGEBRA_H_

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

float invSqrt(float x);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void algebra_start(void);

#endif
