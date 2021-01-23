// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Servo m_lift = new Servo(Constants.Arm.LIFT_PORT);
  private final Servo m_tilt = new Servo(Constants.Arm.TILT_PORT);
  private final Servo m_gripper = new Servo(Constants.Arm.GRIPPER_PORT);
  private final AnalogInput m_gripperRead = new AnalogInput(Constants.Arm.GRIPPER_FEEDBACK_PORT);
  private double m_liftPos;
  private double m_tiltPos;
  private double m_gripperPos;

  // Creates a new Arm subsystem
  public Arm() {
    reset();
  }

  // Reset position to resting state
  public void reset() {
    m_liftPos = 0.5;
    m_tiltPos = 0.5;
    m_gripperPos = 0.5;

    m_lift.set(m_liftPos);
    m_tilt.set(m_tiltPos);
    m_gripper.set(m_gripperPos);
  }

  /** 
   * Increment tilt motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementTilt(double delta) {
    m_tiltPos = saturateLimit(m_tiltPos + delta, Constants.Arm.TILT_MIN, Constants.Arm.TILT_MAX);
    m_tilt.set(m_tiltPos);
  }

  /**
   * Increment lift motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementLift(double delta) {
    m_liftPos = saturateLimit(m_liftPos + delta,  Constants.Arm.LIFT_MIN,  Constants.Arm.LIFT_MAX); 
    m_lift.set(m_liftPos);
  }

  /** 
   * Increment gripper motor position
   * 
   * @param delta Amount to change motor position
   */ 
  public void incrementGripper(double delta) {
    m_gripperPos = saturateLimit(m_gripperPos + delta,  Constants.Arm.GRIPPER_MIN,  Constants.Arm.GRIPPER_MAX); 
    m_gripper.set(m_gripperPos);
  }

  // Get gripper motor position from feedback signal
  public int getGripperPos() {
    return m_gripperRead.getAverageValue();
  }

  // Limit motor range to avoid moving beyond safe ranges
  public double saturateLimit(double val, double l_limit, double u_limit) {
    double outval = val;
    if(val > u_limit) {
      outval =  u_limit;
    } else if (val < l_limit) {
      outval = l_limit;
    }
    return outval;
  }

  @Override
  public void periodic() {
  }
}
