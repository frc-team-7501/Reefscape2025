// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevatorMotorL = new SparkMax(CANMapping.SPARKMAX_ELEVATOR_L, MotorType.kBrushless);
  private final SparkMax elevatorMotorR = new SparkMax(CANMapping.SPARKMAX_ELEVATOR_R, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private static Elevator instance;

  public Elevator() {
    // ElevatorMotorL.restoreFactoryDefaults();
    encoder = elevatorMotorL.getEncoder();
  }

  public void resetEncoder() {
    elevatorMotorL.getEncoder().setPosition(0);
    elevatorMotorR.getEncoder().setPosition(0);
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
    return encoder.getPosition();
  }
  public void moveElevator(double speed) {
    elevatorMotorL.set(speed);
    elevatorMotorR.set(speed);
  }

  public void stop() {
    elevatorMotorL.stopMotor();
    elevatorMotorR.stopMotor();
  }
}
