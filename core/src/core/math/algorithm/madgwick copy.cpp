#include "core/algorithm/madgwick.hpp"
namespace core::algorithm {
/**
 * @brief
 *
 * @param accel Linear Acceleration m2/sec
 * @param gyro Angular Velocity in rad/sec
 */
void ImuFilter(const Vec3& accel, const Vec3& gyro) {
  // Variables and constants
  struct quaternion q_est_prev = q_est;
  // used as a place holder in equations 42 and 43
  struct quaternion q_est_dot = {0};

  // raw acceleration values
  Quat q_a{0, accel.x, accel.y, accel.z};

  float F_g[3] = {0};     // equation(15/21/25) objective function for gravity
  float J_g[3][4] = {0};  // jacobian matrix for gravity

  // struct quaternion gradient = {0};

  // Integrate angluar velocity to obtain position in angles. //
  Quat q_w{0, gyro.x, gyro.y, gyro.z};
  q_w *= 0.5;
  q_w = q_est * q_w;

  /* NOTE
   * Page 10 states equation (40) substitutes equation (13) into it. This seems
   * false, as he actually substitutes equation (12), q_se_dot_w, not equation
   * (13), q_se_w.
   *
   * // quat_scalar(&q_w, deltaT);               // equation (13) integrates the
   * angles velocity to position
   * // quat_add(&q_w, q_w, q_est_prev);         // addition part of equation
   * (13)
   */

  /* Compute the gradient by multiplying the jacobian matrix by the objective
   function. This is equation 20. The Jacobian matrix, J, is a 3x4 matrix of
   partial derivatives for each quaternion component in the x y z axes The
   objective function, F, is a 3x1 matrix for x y and z. To multiply these
   together, the inner dimensions must match, so use J'. I calculated "by hand"
   the transpose of J, so I will be using "hard coordinates" to get those values
   from J. The matrix multiplication can also be done hard coded to reduce code.

   Note: it is possible to compute the objective function with quaternion
   multiplication functions, but it does not take into account the many zeros
   that cancel terms out and is not optimized like the paper shows
   */

  quat_Normalization(&q_a);  // normalize the acceleration quaternion to be a
                             // unit quaternion
  // Compute the objective function for gravity, equation(15), simplified to
  // equation (25) due to the 0's in the acceleration reference quaternion
  F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) -
           q_a.q2;
  F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) -
           q_a.q3;
  F_g[2] =
    2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) -
    q_a.q4;

  // Compute the Jacobian matrix, equation (26), for gravity
  J_g[0][0] = -2 * q_est_prev.q3;
  J_g[0][1] = 2 * q_est_prev.q4;
  J_g[0][2] = -2 * q_est_prev.q1;
  J_g[0][3] = 2 * q_est_prev.q2;

  J_g[1][0] = 2 * q_est_prev.q2;
  J_g[1][1] = 2 * q_est_prev.q1;
  J_g[1][2] = 2 * q_est_prev.q4;
  J_g[1][3] = 2 * q_est_prev.q3;

  J_g[2][0] = 0;
  J_g[2][1] = -4 * q_est_prev.q2;
  J_g[2][2] = -4 * q_est_prev.q3;
  J_g[2][3] = 0;

  // now computer the gradient, equation (20), gradient = J_g'*F_g
  gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
  gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
  gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
  gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

  // Normalize the gradient, equation (44)
  quat_Normalization(&gradient);

  /* This is the sensor fusion part of the algorithm.
   Combining Gyroscope position angles calculated in the beginning, with the
   quaternion orienting the accelerometer to gravity created above. Noticably
   this new quaternion has not be created yet, I have only calculated the
   gradient in equation (19). Madgwick however uses assumptions with the step
   size and filter gains to optimize the gradient descent, combining it with the
   sensor fusion in equations (42-44). He says the step size has a var alpha,
   which he assumes to be very large. This dominates the previous estimation in
   equation (19) to the point you can ignore it. Eq. 36 has the filter gain
   Gamma, which is related to the step size and thus alpha. With alpha being
   very large, you can make assumptions to simplify the fusion equatoin of
   eq.36. Combining the simplification of the gradient descent equation with the
   simplification of the fusion equation gets you eq. 41 which can be subdivided
   into eqs 42-44.
  */
  quat_scalar(&gradient, BETA);         // multiply normalized gradient by beta
  quat_sub(&q_est_dot, q_w, gradient);  // subtract above from q_w, the
                                        // integrated gyro quaternion
  quat_scalar(&q_est_dot, DELTA_T);
  quat_add(&q_est, q_est_prev,
           q_est_dot);         // Integrate orientation rate to find position
  quat_Normalization(&q_est);  // normalize the orientation of the estimate
                               //(shown in diagram, plus always use unit
                               // quaternions for orientation)
}

/*
 returns as pointers, roll pitch and yaw from the quaternion generated in
 imu_filter Assume right hand system Roll is about the x axis, represented as
 phi Pitch is about the y axis, represented as theta Yaw is about the z axis,
 represented as psi (trident looking greek symbol)
 */
void eulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw) {
  *yaw = atan2f((2 * q.q2 * q.q3 - 2 * q.q1 * q.q4),
                (2 * q.q1 * q.q1 + 2 * q.q2 * q.q2 - 1));  // equation (7)
  *pitch = -asinf(2 * q.q2 * q.q4 + 2 * q.q1 * q.q3);      // equatino (8)
  *roll = atan2f((2 * q.q3 * q.q4 - 2 * q.q1 * q.q2),
                 (2 * q.q1 * q.q1 + 2 * q.q4 * q.q4 - 1));

  *yaw *= (180.0f / PI);
  *pitch *= (180.0f / PI);
  *roll *= (180.0f / PI);
}
}  // namespace core::algorithm