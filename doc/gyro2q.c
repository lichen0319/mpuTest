#if 1
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float dt)
{
  float recipNorm;
  float qa, qb, qc;

  gx = gx * M_PI_F / 180;
  gy = gy * M_PI_F / 180;
  gz = gz * M_PI_F / 180;

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  // Runge-Kutta
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
#else
static void sensfusion6UpdateQImpl(float gx, float gy, float gz, float ax, float dt)
{
  float recipNorm;
  float qDot1, qDot2, qDot3, qDot4;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
#endif

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

static float gravX, gravY, gravZ; // Unit vector in the estimated gravity direction

static void estimatedGravityDirection(float* gx, float* gy, float* gz)
{
  *gx = 2 * (q1 * q3 - q0 * q2);
  *gy = 2 * (q0 * q1 + q2 * q3);
  *gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx = gravX;
  float gy = gravY;
  float gz = gravZ;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  *yaw = atan2f(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180 / M_PI_F;
  *pitch = asinf(gx) * 180 / M_PI_F; //Pitch seems to be inverted
  *roll = atan2f(gy, gz) * 180 / M_PI_F;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float dt)
{
  sensfusion6UpdateQImpl(gx, gy, gz, dt);
  estimatedGravityDirection(&gravX, &gravY, &gravZ);
  sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);  
}

