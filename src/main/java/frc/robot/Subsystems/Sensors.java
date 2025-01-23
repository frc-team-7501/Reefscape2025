// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscMapping;

public class Sensors extends SubsystemBase {
  /** Creates a new Sensors. */
  // Other "Fake" Sensors
  private boolean isFieldCentric;
  private double speedMultiplier;
  private static Sensors instance;

  public Sensors() {
    // Set the default delivery method to Launcher.
    isFieldCentric = true;
    speedMultiplier = MiscMapping.NORMAL_MULTIPLIER;
  }

  public static Sensors getInstance() {
    if (instance == null)
      instance = new Sensors();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getIsFieldCentric() {
    return isFieldCentric;
  }

  public void setIsFieldCentric(boolean isCentric) {
    isFieldCentric = isCentric;
  }

  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public void setSpeedMultiplier(double multiplier) {
    speedMultiplier = multiplier;
  }
}
