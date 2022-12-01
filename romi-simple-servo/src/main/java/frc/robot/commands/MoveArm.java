// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

 /**
   * Creates a new MoveArm. This will allow you to move your "arm" or motor. Pressing was is assumed
   * to be the "A" button will rotate the arm/servo to the right. Pressing the "B" button will rotate
   * the arm/servo to the left. For button mapping see the Constants.java file. Note, this command does not terminate.
   *
   * @param Arm This is the "arm" subsystem. This is the subsystem that has the Servo in it.
   * @param Joystick This is the controller you'd like to use for control of the "arm".
   */

public class MoveArm extends CommandBase {
    private final Joystick m_joystick;
    private final Arm m_arm;
    private final double delta = 0.01;

    public MoveArm(Arm arm, Joystick joystick) {
        m_joystick = joystick;
        m_arm = arm;
        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_joystick.getRawButton(Constants.Joystick.A)) {
            m_arm.set(m_arm.get() - delta);
            System.out.println("BUTTON A PRESSED");
        }   
        if(m_joystick.getRawButton(Constants.Joystick.B)) {
            m_arm.set(m_arm.get() + delta);
            System.out.println("BUTTON B PRESSED");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
