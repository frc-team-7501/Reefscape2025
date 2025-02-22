// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Climb extends SubsystemBase {
  private final TalonFX m_ClimbMotor = new TalonFX(CANMapping.TALONFX_CLIMB);
  private static Climb instance;
 
  // Creates a new Climb. 
  public Climb() {}

  public static Climb getInstance() {
    if (instance == null)
    instance = new Climb();
    return instance;
  }

  public void moveClimb(double ClimbPower) {
    m_ClimbMotor.set(ClimbPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ClimbMotor.getPosition();
  }

  public void stop() {
    m_ClimbMotor.set(0.0);
  }
}