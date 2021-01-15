// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.function.Supplier;

public class TeleopCommand extends ParallelCommandGroup {
  /**
   * Creates a new Teleop Command. This will use Arcade Drive and
   * allow for arm movement based on the buttons.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public TeleopCommand(Arm arm, Drivetrain drivetrain, Joystick joystick) {

  Supplier<Double> xaxisSpeedSupplier = () -> joystick.getRawAxis(1);
  Supplier<Double> zaxisRotateSuppplier =  () -> -joystick.getRawAxis(2);
  
    addCommands(
        new ArcadeDrive(drivetrain, xaxisSpeedSupplier, zaxisRotateSuppplier),
        new JoystickArmCommand(arm, joystick)
    );
  }
}
