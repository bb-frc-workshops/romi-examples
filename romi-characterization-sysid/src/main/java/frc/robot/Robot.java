/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private class QuasistaticVoltageSupplier implements Supplier<Double> {
    private double rampRate = 0.0;
    private double currentVoltage = 0.0;
    private int counter = 0;

    public QuasistaticVoltageSupplier(double rampRate) {
      this.rampRate = rampRate;
    }

    @Override
    public Double get() {
      // Simple rate limiter. Without it Romi will keep running for at least
      // a couple of seconds after disabling it
      if (++counter == 10) {
        this.currentVoltage = this.rampRate * (Timer.getFPGATimestamp() - m_startTime);
        this.counter = 0;
      }
      return this.currentVoltage;
    }
  }

  private XboxController m_stick;

  private Supplier<Double> m_leftEncoderPosition;
  private Supplier<Double> m_rightEncoderPosition;
  private Supplier<Double> m_leftEncoderRate;
  private Supplier<Double> m_rightEncoderRate;
  private Supplier<Double> m_gyroAngleRadians;
  private Supplier<Double> m_gyroAngleRate;
  private Supplier<Double> m_voltageSupplier;

  private NetworkTableEntry m_sysIdTelemetryEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTelemetry");
  private NetworkTableEntry m_sysIdVoltageCommandEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdVoltageCommand");
  private NetworkTableEntry m_sysIdTestTypeEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTestType");
  private NetworkTableEntry m_sysIdRotateEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdRotate");
  private NetworkTableEntry m_sysIdTestEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTest");
  private NetworkTableEntry m_sysIdWrongMechEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdWrongMech");
  private NetworkTableEntry m_sysIdOverflowEntry = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdOverflow");

  private int m_counter = 0;
  private double m_startTime = 0;
  private double m_motorVoltage = 0;

  private double[] m_numberArray = new double[9];
  private ArrayList<Double> m_entries = new ArrayList<>();
  
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final RomiGyro m_gyro = new RomiGyro();

  public Robot() {
    super(0.005); // 5 ms
    LiveWindow.disableAllTelemetry();
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_stick = new XboxController(0);

    // Note that the angle from the gyro must be negated because 
    // getAngle returns a clockwise positive
    m_gyroAngleRadians = () -> -1 * Math.toRadians(-m_gyro.getAngleZ());
    m_gyroAngleRate = () -> m_gyro.getRateZ();

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
    SmartDashboard.putNumber("Left Encoder Position", m_leftEncoderPosition.get());
    SmartDashboard.putNumber("Left Encoder Rate", m_leftEncoderRate.get());
    SmartDashboard.putNumber("Right Encoder Position", m_rightEncoderPosition.get());
    SmartDashboard.putNumber("Right Encoder Rate", m_rightEncoderRate.get());
    SmartDashboard.putNumber("Motor Voltage", m_motorVoltage);
  }

  /**
   * Initialization code for autonomous mode which will be called each time the
   * robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");

    m_drivetrain.resetEncoders();
    m_gyro.reset();

    m_startTime = Timer.getFPGATimestamp();
    m_voltageSupplier = createVoltageSupplier();
    m_counter = 0;
  }

  private Supplier<Double> createVoltageSupplier() {
    String test = m_sysIdTestEntry.getString(null);
    if (!("Drivetrain".equals(test) || "Drivetrain (Angular)".equals(test))) {
      m_sysIdWrongMechEntry.setBoolean(true);
    } else {
      // Retreive test type & voltage from network table
      double reqVoltage = m_sysIdVoltageCommandEntry.getDouble(0);
      String testType = m_sysIdTestTypeEntry.getString(null);

      if ("Quasistatic".equals(testType)) {
        return new QuasistaticVoltageSupplier(reqVoltage);
      } else if ("Dynamic".equals(testType)) {
        return () -> reqVoltage;
      }
    }
    return () -> 0.0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double now = Timer.getFPGATimestamp();

    // Retrieve values to send back before telling the motors to do something
    double leftPosition = m_leftEncoderPosition.get();
    double leftRate = m_leftEncoderRate.get();

    double rightPosition = m_rightEncoderPosition.get();
    double rightRate = m_rightEncoderRate.get();

    double gyroAngle = m_gyroAngleRadians.get();
    double gyroAngleRate = m_gyroAngleRate.get();

    double leftMotorVolts = m_motorVoltage;
    double rightMotorVolts = m_motorVoltage;

    double battery = RobotController.getBatteryVoltage();
    m_motorVoltage = MathUtil.clamp(m_voltageSupplier.get(), -battery, battery);

    // Command motors to do things
    m_drivetrain.tankDriveVolts(
      (m_sysIdRotateEntry.getBoolean(false) ? -1 : 1) * m_motorVoltage, m_motorVoltage
    );

    m_numberArray[0] = now;
    m_numberArray[1] = leftMotorVolts;
    m_numberArray[2] = rightMotorVolts;
    m_numberArray[3] = leftPosition;
    m_numberArray[4] = rightPosition;
    m_numberArray[5] = leftRate;
    m_numberArray[6] = rightRate;
    m_numberArray[7] = gyroAngle;
    m_numberArray[8] = gyroAngleRate;

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
    m_drivetrain.arcadeDrive(-m_stick.getLeftY(), m_stick.getRightX());
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - m_startTime;
    System.out.println("Robot disabled");
    m_drivetrain.tankDriveVolts(0, 0);

    m_sysIdOverflowEntry.setBoolean(m_entries.size() >= 36_000);

    // data processing step
    String data = m_entries.stream().map(String::valueOf).collect(Collectors.joining(","));
    m_sysIdTelemetryEntry.setString(data);
    m_entries.clear();
    System.out.println("Collected: " + m_counter + " in " + elapsedTime + " seconds");
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
