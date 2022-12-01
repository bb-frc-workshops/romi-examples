// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    // Set min/max of motor position
    private static final double m_maxPosition = 1;
    private static final double m_minPosition = 0;
    private double m_position;

    // Create a servo object according to the location of the servo on the board
    private Servo m_Arm = new Servo(Constants.GPIO.EXT2_PWM);

    // Create an arm and reset to mid-point
    public Arm() {
        m_position = getMidPoint();
        m_Arm.set(m_position);
    }

    // Get servo position. Value is between min and max specified above.
    public double get() {
        return m_position;
    }

    // Set the position of the sero. Value is between min and max specified above. 
    // Any value beyond these boundaries are saturated to the boundary.
    public void set(double position) {
        m_position = saturate(position);
        m_Arm.set(m_position);
        System.out.println("Setting Position = " + m_position);

    }

    // Get the midpoint based on min/max settings of the servo.
    private double getMidPoint() {
        return (m_maxPosition - m_minPosition)/2;
    }

    // Saturate the value to the min/max limits specified above.
    private double saturate(double position) {
        double ret_position = position;
        if(ret_position < m_minPosition) {
            ret_position = m_minPosition;
        } else if(ret_position > m_maxPosition) {
            ret_position = m_maxPosition;
        }
        return ret_position;
    }
}