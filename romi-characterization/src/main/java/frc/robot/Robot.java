/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick m_stick;

  Supplier<Double> m_leftEncoderPosition;
  Supplier<Double> m_rightEncoderPosition;
  Supplier<Double> m_leftEncoderRate;
  Supplier<Double> m_rightEncoderRate;
  Supplier<Double> m_gyroAngleRadians;

  NetworkTableEntry m_autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry m_telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry m_rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  
  String m_data = "";

  int m_counter = 0;
  double m_startTime = 0;
  double m_priorAutoSpeed = 0;

  double[] m_numberArray = new double[10];
  ArrayList<Double> m_entries = new ArrayList<>();
  
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final RomiGyro m_gyro = new RomiGyro();

  public Robot() {
    super(0.005);
    LiveWindow.disableAllTelemetry();
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // Set up the data providers

    // Note that the angle from the gyro must be negated because 
    // getAngle returns a clockwise positive
    m_gyroAngleRadians = () -> -1 * Math.toRadians(-m_gyro.getAngleZ());

    m_leftEncoderPosition = m_drivetrain::getLeftDistance;
    m_leftEncoderRate = m_drivetrain::getLeftEncoderRate;

    m_rightEncoderPosition = m_drivetrain::getRightDistance;
    m_rightEncoderRate = m_drivetrain::getRightEncoderRate;

    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // These aren't actually used by the characterization tool, but serve as 
    // a debugging aid
    SmartDashboard.putNumber("l_encoder_pos", m_leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", m_leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", m_rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", m_rightEncoderRate.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    m_drivetrain.resetEncoders();
    m_startTime = Timer.getFPGATimestamp();
    m_counter = 0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // NOTE: This logic is basically copied from the project that frc-characterization
    // generates
    
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = m_leftEncoderPosition.get();
    double leftRate = m_leftEncoderRate.get();

    double rightPosition = m_rightEncoderPosition.get();
    double rightRate = m_rightEncoderRate.get();

    // The battery voltage is scaled to 12V
    double battery = RobotController.getBatteryVoltage();

    double motorVolts = battery * Math.abs(m_priorAutoSpeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = m_autoSpeedEntry.getDouble(0);
    m_priorAutoSpeed = autospeed;

    // Command motors to do things
    m_drivetrain.tankDrive(
      (m_rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    m_numberArray[0] = now;
    m_numberArray[1] = battery;
    m_numberArray[2] = autospeed;
    m_numberArray[3] = leftMotorVolts;
    m_numberArray[4] = rightMotorVolts;
    m_numberArray[5] = leftPosition;
    m_numberArray[6] = rightPosition;
    m_numberArray[7] = leftRate;
    m_numberArray[8] = rightRate;
    m_numberArray[9] = m_gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : m_numberArray) {
      m_entries.add(num);
    }
    m_counter++;
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_drivetrain.arcadeDrive(-m_stick.getY(Hand.kLeft), m_stick.getX(Hand.kRight));
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - m_startTime;
    System.out.println("Robot disabled");
    m_drivetrain.tankDrive(0, 0);

    // data processing step
    m_data = m_entries.toString();
    m_data = m_data.substring(1, m_data.length() - 1) + ", ";
    m_telemetryEntry.setString(m_data);
    m_entries.clear();
    System.out.println("Collected: " + m_counter + " in " + elapsedTime + " seconds");
    m_data = "";
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
