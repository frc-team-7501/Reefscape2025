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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.DIOMapping;

public class Differential extends SubsystemBase {
  /** Creates a new Differential. */
  private final SparkMax differentialMotorR;
  private final SparkMax differentialMotorL;
  private static Differential instance;
  private SparkClosedLoopController differentialMotorRPID;
  private SparkClosedLoopController differentialMotorLPID;
  private SparkMaxConfig differentialMotorRConfig;
  private SparkMaxConfig differentialMotorLConfig;
  private final DutyCycleEncoder differentialEncoderL = 
      new DutyCycleEncoder(
        DIOMapping.DIFFERENTIAL_ENCODER_L);
  private final DutyCycleEncoder differentialEncoderR = 
      new DutyCycleEncoder(
        DIOMapping.DIFFERENTIAL_ENCODER_R);

  private final PIDController differentialPIDControllerL = new PIDController(0.10, 0.0, 0.0);
  private final PIDController differentialPIDControllerR = new PIDController(0.10, 0.0, 0.0);

  public Differential() {
    // Create motors, configuration, and PID
    differentialMotorR = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_R, MotorType.kBrushless);
    differentialMotorL = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_L, MotorType.kBrushless);
    differentialMotorRConfig = new SparkMaxConfig();
    differentialMotorLConfig = new SparkMaxConfig();

    // PID values
    
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
    
    SmartDashboard.putNumber("DiffPosL", getDifferentialPositionL());
    SmartDashboard.putNumber("DiffPosR", getDifferentialPositionR());


  }

  public double getDifferentialPositionL() {
    return differentialEncoderL.get();
  }
  
  public double getDifferentialPositionR() {
    return differentialEncoderR.get();
  }

  // Manual command
  public void moveDifferential(double speedR, double speedL) {
    differentialMotorR.set(speedR);
    differentialMotorL.set(speedL);
  }

  // PID command
  public void pidSetPosition(double positionR, double positionL) {
    differentialMotorR.set(-differentialPIDControllerR.calculate(differentialEncoderR.get(), positionR));
    differentialMotorL.set(differentialPIDControllerL.calculate(differentialEncoderL.get(), positionL));
    SmartDashboard.putNumber("DiffPowR",-differentialPIDControllerR.calculate(differentialEncoderR.get(), positionR));
    SmartDashboard.putNumber("DiffPowL",differentialPIDControllerL.calculate(differentialEncoderL.get(), positionL));
  }

  public void stop() {
    differentialMotorR.stopMotor();
    differentialMotorL.stopMotor();
  }
}
