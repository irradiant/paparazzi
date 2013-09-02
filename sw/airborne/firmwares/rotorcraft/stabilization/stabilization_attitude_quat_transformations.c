/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_quat_transformations.c
 *  Quaternion transformation functions.
 */

#include "stabilization_attitude_quat_transformations.h"

void quat_from_rpy_cmd_i(struct Int32Quat* quat, struct Int32Eulers* cmd) {
  // use float conversion for now...
  struct FloatEulers cmd_f;
  EULERS_FLOAT_OF_BFP(cmd_f, *cmd);

  struct FloatQuat quat_f;
  quat_from_rpy_cmd_f(&quat_f, &cmd_f);

  // convert back to fixed point
  QUAT_BFP_OF_REAL(*quat, quat_f);
}

void quat_from_rpy_cmd_f(struct FloatQuat* quat, struct FloatEulers* cmd) {
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = cmd->phi;
  ov.y = cmd->theta;
  ov.z = 0.0;
  /* quaternion from that orientation vector */
  struct FloatQuat q_rp;
  FLOAT_QUAT_OF_ORIENTATION_VECT(q_rp, ov);

  /// @todo optimize yaw angle calculation

  /*
   * Instead of using the psi setpoint angle to rotate around the body z-axis,
   * calculate the real angle needed to align the projection of the body x-axis
   * onto the horizontal plane with the psi setpoint.
   *
   * angle between two vectors a and b:
   * angle = atan2(norm(cross(a,b)), dot(a,b)) * sign(dot(cross(a,b), n))
   * where n is the thrust vector (i.e. both a and b lie in that plane)
   */
  const struct FloatVect3 xaxis = {1.0, 0.0, 0.0};
  const struct FloatVect3 zaxis = {0.0, 0.0, 1.0};
  struct FloatVect3 a;
  FLOAT_QUAT_VMULT(a, q_rp, xaxis);

  // desired heading vect in earth x-y plane
  struct FloatVect3 psi_vect;
  psi_vect.x = cosf(cmd->psi);
  psi_vect.y = sinf(cmd->psi);
  psi_vect.z = 0.0;
  // normal is the direction of the thrust vector
  struct FloatVect3 normal;
  FLOAT_QUAT_VMULT(normal, q_rp, zaxis);

  // projection of desired heading onto body x-y plane
  // b = v - dot(v,n)*n
  float dot = FLOAT_VECT3_DOT_PRODUCT(psi_vect, normal);
  struct FloatVect3 dotn;
  FLOAT_VECT3_SMUL(dotn, normal, dot);

  // b = v - dot(v,n)*n
  struct FloatVect3 b;
  FLOAT_VECT3_DIFF(b, psi_vect, dotn);
  dot = FLOAT_VECT3_DOT_PRODUCT(a, b);
  struct FloatVect3 cross;
  VECT3_CROSS_PRODUCT(cross, a, b);
  // norm of the cross product
  float nc = FLOAT_VECT3_NORM(cross);
  // angle = atan2(norm(cross(a,b)), dot(a,b))
  float yaw2 = atan2(nc, dot) / 2.0;

  // negative angle if needed
  // sign(dot(cross(a,b), n)
  float dot_cross_ab = FLOAT_VECT3_DOT_PRODUCT(cross, normal);
  if (dot_cross_ab < 0) {
    yaw2 = -yaw2;
  }

  /* quaternion with yaw command */
  struct FloatQuat q_yaw;
  QUAT_ASSIGN(q_yaw, cosf(yaw2), 0.0, 0.0, sinf(yaw2));

  /* final setpoint: apply roll/pitch, then yaw around resulting body z-axis */
  FLOAT_QUAT_COMP(*quat, q_yaw, q_rp);
  FLOAT_QUAT_NORMALIZE(*quat);
  FLOAT_QUAT_WRAP_SHORTEST(*quat);
}
