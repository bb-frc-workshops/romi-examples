// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickArmCommand extends CommandBase {

  private final Arm m_arm;
  private final Joystick m_joystick;

  private double m_delta = 0.005;
  private long m_startTime = 0;

  public JoystickArmCommand(Arm arm, Joystick joystick) {
    m_arm = arm;
    m_joystick = joystick;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() - m_startTime > 500 ) {
      System.out.println("Gripper Pos " + m_arm.getGripperPos());
      m_startTime = System.currentTimeMillis();
    }

    if(m_joystick.getRawButton(Constants.Button.TOPLEFT)) {
      m_arm.incrementLift(-m_delta);
    }
    if(m_joystick.getRawButton(Constants.Button.TOPRIGHT)) {
      m_arm.incrementLift(m_delta);
    }
    if(m_joystick.getRawButton(Constants.Button.BOTTOMLEFT)) {
      m_arm.incrementTilt(m_delta);
    }
    if(m_joystick.getRawButton(Constants.Button.BOTTOMRIGHT)) {
      m_arm.incrementTilt(-m_delta);
    }
    if(m_joystick.getRawButton(Constants.Button.Y)) {
      m_arm.incrementGripper(m_delta);
    }
    if(m_joystick.getRawButton(Constants.Button.B)) {
      m_arm.incrementGripper(-m_delta);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
