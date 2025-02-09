// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Differential extends SubsystemBase {
  /** Creates a new Differential. */
  private final SparkMax differentialMotorR;
  private final SparkMax differentialMotorL;
  private static Differential instance;
  private SparkClosedLoopController differentialMotorRPID;
  private SparkClosedLoopController differentialMotorLPID;
  private SparkMaxConfig differentialMotorRConfig;
  private SparkMaxConfig differentialMotorLConfig;

  public Differential() {
    // Create motors, configuration, and PID
    differentialMotorR = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_R, MotorType.kBrushless);
    differentialMotorL = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_R, MotorType.kBrushless);
    differentialMotorRPID = differentialMotorR.getClosedLoopController();
    differentialMotorRConfig = new SparkMaxConfig();
    differentialMotorLConfig = new SparkMaxConfig();

    // PID values
    differentialMotorRConfig.closedLoop
        .p(0.002)
        .i(0.0)
        .d(0.0)
        .outputRange(-0.01, 0.01);

    differentialMotorR.configure(differentialMotorRConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    differentialMotorL.configure(differentialMotorLConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void resetEncoder() {
    differentialMotorR.getEncoder().setPosition(0);
    differentialMotorL.getEncoder().setPosition(0);
  }

  public static Differential getInstance() {
    if (instance == null)
      instance = new Differential();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Output for the right motor
    SmartDashboard.putNumber("DiffPosR", differentialMotorR.getEncoder().getPosition());
    SmartDashboard.putNumber("DiffPowR", differentialMotorR.getAppliedOutput());
    SmartDashboard.putNumber("DiffCurR", differentialMotorR.getOutputCurrent());

    SmartDashboard.putNumber("DiffPosL", differentialMotorL.getEncoder().getPosition());
    SmartDashboard.putNumber("DiffPowL", differentialMotorL.getAppliedOutput());
    SmartDashboard.putNumber("DiffCurL", differentialMotorL.getOutputCurrent());

  }

  public double getDifferentialPosition() {
    return differentialMotorR.getEncoder().getPosition();
  }

  // Manual command
  public void moveDifferential(double speedR, double speedL) {
    differentialMotorR.set(speedR);
    differentialMotorL.set(speedL);
  }

  // PID command
  public void pidSetPosition(double positionR, double positionL) {
    differentialMotorRPID.setReference(positionR, ControlType.kPosition);
    differentialMotorLPID.setReference(positionL, ControlType.kPosition);
  }

  public void stop() {
    differentialMotorR.stopMotor();
    differentialMotorL.stopMotor();
  }
}
