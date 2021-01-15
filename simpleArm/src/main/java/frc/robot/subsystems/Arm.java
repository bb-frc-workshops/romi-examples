// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Servo m_lift = new Servo(4);
  private final Servo m_tilt = new Servo(3);
  private final Servo m_gripper = new Servo(2);
  private final AnalogInput m_gripperRead = new AnalogInput(0);
  private double m_liftPos;
  private double m_tiltPos;
  private double m_gripperPos;

  /** Creates a new Arm. */
  public Arm() {
    reset();
  }

  public void reset() {
    m_liftPos = 0.5;
    m_tiltPos = 0.5;
    m_gripperPos = 0.5;

    m_lift.set(m_liftPos);
    m_tilt.set(m_tiltPos);
    m_gripper.set(m_gripperPos);
  }

  public void incrementTilt(double delta) {
    /* Spec: https://www.pololu.com/docs/0J76/4
     * Range should be 1200 (down) - 1900 (up) us 
     */
    m_tiltPos = saturateLimit(m_tiltPos + delta, 0, 1);;
    m_tilt.set(m_tiltPos);
  }

  public void incrementLift(double delta) {
    /* Spec: https://www.pololu.com/docs/0J76/4
     * Range should be 1000 (raised) - 1900 (lowered) us 
     */
    m_liftPos = saturateLimit(m_liftPos + delta, 0, 1); 
    m_lift.set(m_liftPos);
  }

  public void incrementGripper(double delta) {
    /* Spec: https://www.pololu.com/docs/0J76/4
     * Range should be 500 (open) - 2400 (closed) us 
     * AnalogIn range right now from 440 - 1850.
    */
    m_gripperPos = saturateLimit(m_gripperPos + delta, 0, 1); 
    m_gripper.set(m_gripperPos);
  }

  public int getGripperPos() {
    return m_gripperRead.getAverageValue();
  }

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
