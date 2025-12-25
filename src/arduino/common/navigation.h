/**
 * Navigation Header
 * Proportional controller for waypoint navigation
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

// Navigation gains (configure in main .ino)
extern float NAV_KP;
extern float NAV_KA;
extern float NAV_THRESHOLD;

// Velocity limits
extern float MAX_LINEAR_VEL;
extern float MAX_ANGULAR_VEL;

struct VelocityCommand {
  float v;        // Linear velocity (m/s)
  float omega;    // Angular velocity (rad/s)
  bool done;      // Goal reached flag
};

/**
 * Compute velocity commands to reach a goal
 * @param current_x: Robot x position (m)
 * @param current_y: Robot y position (m)
 * @param current_theta: Robot heading (rad)
 * @param goal_x: Goal x position (m)
 * @param goal_y: Goal y position (m)
 * @return VelocityCommand struct
 */
VelocityCommand computeVelocity(float current_x, float current_y,
                                float current_theta,
                                float goal_x, float goal_y) {
  VelocityCommand cmd;

  // Calculate distance to goal
  float dx = goal_x - current_x;
  float dy = goal_y - current_y;
  float rho = sqrt(dx*dx + dy*dy);

  // Calculate angle to goal
  float alpha = -current_theta + atan2(dy, dx);

  // Proportional controller
  cmd.v = NAV_KP * rho;
  cmd.omega = NAV_KA * alpha;

  // Apply velocity limits
  cmd.v = constrain(cmd.v, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  cmd.omega = constrain(cmd.omega, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  // Check if goal reached
  cmd.done = (rho < NAV_THRESHOLD);

  return cmd;
}

#endif // NAVIGATION_H
