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

public class Funnel extends SubsystemBase {
  /** Creates a new Funnel. */
  private final SparkMax funnelMotor;
  private static Funnel instance;
  private SparkClosedLoopController funnelMotorPID;
  private SparkMaxConfig funnelMotorConfig;


  public Funnel() {
    // Create motors, configuration, and PID
    funnelMotor = new SparkMax(CANMapping.SPARKMAX_FUNNEL, MotorType.kBrushless);
    funnelMotorPID = funnelMotor.getClosedLoopController();
    funnelMotorConfig = new SparkMaxConfig();

    // PID values
    funnelMotorConfig.closedLoop
        .p(0.04)
        .i(0.0)
        .d(0.0)
        .outputRange(-0.1, 0.1);

        funnelMotor.configure(funnelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SmartDashboard.putNumber("FunPos", funnelMotor.getEncoder().getPosition());
  }

  public void resetEncoder() {
    funnelMotor.getEncoder().setPosition(0);
  }

  public static Funnel getInstance() {
    if (instance == null)
      instance = new Funnel();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getFunnelPosition() {
    return funnelMotor.getEncoder().getPosition();
  }

  // Manual command
  public void moveFunnel(double speed) {
    funnelMotor.set(speed);
  }

  // PID command
  public void pidSetPosition(double position) {
    funnelMotorPID.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    funnelMotor.stopMotor();
  }
}
