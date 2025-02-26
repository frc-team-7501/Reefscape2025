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
  private SparkMaxConfig differentialMotorRConfig;
  private final DutyCycleEncoder differentialEncoderL = new DutyCycleEncoder(
      DIOMapping.DIFFERENTIAL_ENCODER_L);
  private final DutyCycleEncoder differentialEncoderR = new DutyCycleEncoder(
      DIOMapping.DIFFERENTIAL_ENCODER_R);
  private double offsetL = 0;
  private double offsetR = 0;
  private double lastRawL = 0;
  private double lastRawR = 0;
  private double Kp = 5.0;
  private double Ki = 0.0;
  private double Kd = 0.0;

  // private final PIDController differentialPIDControllerL = new PIDController(2.0, 0.0, 0.0);
  // private final PIDController differentialPIDControllerR = new PIDController(2.0, 0.0, 0.0);
  
  private final TrapezoidProfile.Constraints diffConstraints = new
  TrapezoidProfile.Constraints(1.0, 0.5);

  private final ProfiledPIDController differentialPIDControllerL = new 
  ProfiledPIDController(Kp, Ki, Kd, diffConstraints, 0.02 );

  private final ProfiledPIDController differentialPIDControllerR = new 
  ProfiledPIDController(Kp, Ki, Kd, diffConstraints, 0.02);

  public Differential() {
    // Create motors, configuration, and PID
    differentialMotorR = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_R, MotorType.kBrushed);
    differentialMotorL = new SparkMax(CANMapping.SPARKMAX_DIFFERENTIAL_L, MotorType.kBrushed);

    differentialMotorRConfig = new SparkMaxConfig();

    differentialMotorR.configure(differentialMotorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    differentialEncoderR.setInverted(true);

    // differentialPIDControllerL.reset(getDifferentialPositionL(), 0.0);
    // differentialPIDControllerR.reset(getDifferentialPositionR(), 0.0);
  }

  public static Differential getInstance() {
    if (instance == null)
      instance = new Differential();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AppOutputL", differentialMotorL.getAppliedOutput());
    SmartDashboard.putNumber("AppOutputR", differentialMotorR.getAppliedOutput());
    SmartDashboard.putNumber("LeftPos", getContinuousPositionL());
    SmartDashboard.putNumber("RghtPos", getContinuousPositionR());
    SmartDashboard.putNumber("setpoint", differentialPIDControllerL.getPositionError());
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

  private double clampOutput(double val, double limit) {
    return Math.signum(val) * Math.min(Math.abs(val), limit);
  }

  public void updateOffsetIfResetL() {
    double currentRawL = differentialEncoderL.get();
    if ((currentRawL - lastRawL) > 0.9) {
      offsetL--;
    } else if ((lastRawL - currentRawL) > 0.9) {
      offsetL++;
    }
    lastRawL = currentRawL;
  }

  public void updateOffsetIfResetR() {
    double currentRawR = differentialEncoderR.get();
    if ((currentRawR - lastRawR) > 0.9) {
      offsetR--;
    } else if ((lastRawR - currentRawR) > 0.9) {
      offsetR++;
    }
    lastRawR = currentRawR;
  }

  public double getContinuousPositionL() {
    updateOffsetIfResetL();
    return differentialEncoderL.get() + offsetL;
  }

  public double getContinuousPositionR() {
    updateOffsetIfResetR();
    return differentialEncoderR.get() + offsetR - 0.11;
  }

  // PID command
  public void pidSetPosition(double positionR, double positionL) {

    differentialMotorR.set(clampOutput(-differentialPIDControllerR.calculate(getContinuousPositionR(), positionR), 0.50));
    differentialMotorL.set(clampOutput( differentialPIDControllerL.calculate(getContinuousPositionL(), positionL), 0.50));
    // differentialMotorL.set(clampOutput( differentialPIDControllerL.calculate(getContinuousPositionL(), positionL) + ((getContinuousPositionL() - 0.09) * 0.25), 0.50));
  }

  public void stop() {
    differentialMotorR.stopMotor();
    differentialMotorL.stopMotor();
  }
}