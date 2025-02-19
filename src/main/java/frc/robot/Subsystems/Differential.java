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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.DIOMapping;
import edu.wpi.first.wpilibj.Preferences;


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
  private final String KeyL;
  private final String KeyR;
  

  private final PIDController differentialPIDControllerL = new PIDController(2.0, 0.0, 0.0);
  private final PIDController differentialPIDControllerR = new PIDController(2.0, 0.0, 0.0);
  // MDH Change to use Trapezoid PID
  //private final TrapezoidProfile.Constraints diffConstraintsL = new TrapezoidProfile.Constraints(1.75, 0.75);
  //private final ProfiledPIDController differentialPPIDControllerL = new ProfiledPIDController(0.10, 0.0, 0.0, diffConstraintsL,0.02 );

  public Differential() {
    // Create motors, configuration, and PID
    differentialMotorR = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_R, MotorType.kBrushless);
    differentialMotorL = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_L, MotorType.kBrushless);
    differentialMotorRConfig = new SparkMaxConfig();
    differentialMotorRConfig.inverted(true);
    differentialMotorR.configure(differentialMotorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // PID values
    
  }

  private double clampOutput(double val, double limit) {
    return Math.signum(val) * Math.min(Math.abs(val), limit);
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


    differentialMotorR.set(clampOutput(-differentialPIDControllerR.calculate(differentialEncoderR.get(), positionR), 0.2));
    differentialMotorL.set(clampOutput (differentialPIDControllerL.calculate(differentialEncoderL.get(), positionL), 0.2));
    SmartDashboard.putNumber("DiffPowR",clampOutput(-differentialPIDControllerR.calculate(differentialEncoderR.get(), positionR), 0.2));
    SmartDashboard.putNumber("DiffPowL",clampOutput( differentialPIDControllerL.calculate(differentialEncoderL.get(), positionL), 0.2));
    SmartDashboard.putNumber("AppOutput", differentialMotorL.getAppliedOutput());
  }

  public void stop() {
    differentialMotorR.stopMotor();
    differentialMotorL.stopMotor();
  }
}
