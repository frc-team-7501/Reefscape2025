// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Intake extends SubsystemBase {
  private final TalonSRX m_IntakeMotor = new TalonSRX(CANMapping.INTAKE_TALONSRX);
  private static Intake instance;
  
  /** Creates a new Intake. */
  public Intake() {
  }

  public static Intake getInstance() {
    if (instance == null)
      instance = new Intake();
    return instance;
  }

  public void runIntake(double IntakePower, boolean override, boolean sensor) {
    SmartDashboard.putBoolean("IntakeSen", sensor);
    if (sensor && !override){
      m_IntakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    } else {
      m_IntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakePower);
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_IntakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }
}