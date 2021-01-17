/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

/**
 * Add your docs here.
 */
public class RomiGyro {
  private SimDevice m_simRomiGyro;
  private SimDouble m_simRateX;
  private SimDouble m_simRateY;
  private SimDouble m_simRateZ;
  private SimDouble m_simAngleX;
  private SimDouble m_simAngleY;
  private SimDouble m_simAngleZ;

  private double m_angleXOffset = 0;
  private double m_angleYOffset = 0;
  private double m_angleZOffset = 0;

  public RomiGyro() {
    // Create the SimDevice to match the registered name on the Romi
    m_simRomiGyro = SimDevice.create("Gyro:RomiGyro");
    if (m_simRomiGyro != null) {
      // Create the init field. No need to save a reference since it's one shot
      m_simRomiGyro.createBoolean("init", Direction.kOutput, true);
      m_simRateX = m_simRomiGyro.createDouble("rate_x", Direction.kInput, 0.0);
      m_simRateY = m_simRomiGyro.createDouble("rate_y", Direction.kInput, 0.0);
      m_simRateZ = m_simRomiGyro.createDouble("rate_z", Direction.kInput, 0.0);

      m_simAngleX = m_simRomiGyro.createDouble("angle_x", Direction.kInput, 0.0);
      m_simAngleY = m_simRomiGyro.createDouble("angle_y", Direction.kInput, 0.0);
      m_simAngleZ = m_simRomiGyro.createDouble("angle_z", Direction.kInput, 0.0);
    }
  }

  public double getRateX() {
    if (m_simRateX != null) {
      return m_simRateX.get();
    }

    return 0.0;
  }

  public double getRateY() {
    if (m_simRateY != null) {
      return m_simRateY.get();
    }

    return 0.0;
  }

  public double getRateZ() {
    if (m_simRateZ != null) {
      return m_simRateZ.get();
    }

    return 0.0;
  }

  public double getAngleX() {
    if (m_simAngleX != null) {
      return m_simAngleX.get() - m_angleXOffset;
    }

    return 0.0;
  }

  public double getAngleY() {
    if (m_simAngleY != null) {
      return m_simAngleY.get() - m_angleYOffset;
    }

    return 0.0;
  }

  public double getAngleZ() {
    if (m_simAngleZ != null) {
      return m_simAngleZ.get() - m_angleZOffset;
    }

    return 0.0;
  }

  public void reset() {
    if (m_simAngleX != null) {
      m_angleXOffset = m_simAngleX.get();
      m_angleYOffset = m_simAngleY.get();
      m_angleZOffset = m_simAngleZ.get();
    }
  }
}
