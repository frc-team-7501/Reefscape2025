// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevatorMotorB = new SparkMax(CANMapping.SPARKMAX_ELEVATOR_B, MotorType.kBrushless);
  private final SparkMax elevatorMotorT = new SparkMax(CANMapping.SPARKMAX_ELEVATOR_T, MotorType.kBrushless);
  private static Elevator instance;

  public Elevator() {
    // // ElevatorMotorL.restoreFactoryDefaults();
    // encoder = elevatorMotorB.getEncoder();
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

  public void moveElevator(double speed) {
    elevatorMotorB.set(speed);
    elevatorMotorT.set(speed);
    SmartDashboard.putNumber("ElevatorPos", elevatorMotorB.getEncoder().getPosition());
  }

  public void stop() {
    elevatorMotorB.stopMotor();
    elevatorMotorT.stopMotor();
  }
}
