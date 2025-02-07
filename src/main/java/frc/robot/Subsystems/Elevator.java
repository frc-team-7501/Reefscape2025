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

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevatorMotorB;
  private final SparkMax elevatorMotorT;
  private static Elevator instance;
  private SparkClosedLoopController elevatorMotorBPID;
  private SparkMaxConfig elevatorMotorBConfig;
  private SparkMaxConfig elevatorMotorTConfig;

  public Elevator() {
    // Create motors, configuration, and PID
    elevatorMotorB = new SparkMax(CANMapping.SPARKMAX_ELEVATOR_B, MotorType.kBrushless);
    elevatorMotorT = new SparkMax(CANMapping.SPARKMAX_ELEVATOR_T, MotorType.kBrushless);
    elevatorMotorBPID = elevatorMotorB.getClosedLoopController();
    elevatorMotorBConfig = new SparkMaxConfig();
    elevatorMotorTConfig = new SparkMaxConfig();

    // PID values
    elevatorMotorBConfig.closedLoop
        .p(0.01)
        .i(0.001)
        .d(0)
        .outputRange(-0.2, 0.2);

    elevatorMotorB.configure(elevatorMotorBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Top motor follows the bottom motor.
    elevatorMotorTConfig.follow(elevatorMotorB.getDeviceId(), false);
    elevatorMotorT.configure(elevatorMotorTConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void resetEncoder() {
    elevatorMotorB.getEncoder().setPosition(0);
    elevatorMotorT.getEncoder().setPosition(0);
  }

  public static Elevator getInstance() {
    if (instance == null)
      instance = new Elevator();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getElevatorPosition() {
    return elevatorMotorB.getEncoder().getPosition();
  }

  // Manual command
  public void moveElevator(double speed) {
    elevatorMotorB.set(speed);
    SmartDashboard.putNumber("ElevPos", elevatorMotorB.getEncoder().getPosition());
  }

  // PID command
  public void pidSetPosition(double position) {
    elevatorMotorBPID.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    elevatorMotorB.stopMotor();
    elevatorMotorT.stopMotor();
  }
}
