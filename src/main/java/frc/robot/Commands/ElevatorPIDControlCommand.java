// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Sensors;

public class ElevatorPIDControlCommand extends Command {
  /** Creates a new LiftControlCommand. */
  private final Elevator elevator;
  private final Sensors sensors;
  private int diffLevel;
  private double elevatorPosition;

  public ElevatorPIDControlCommand(Elevator elevator, Sensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.sensors = sensors;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    diffLevel = sensors.getDifferentialLevel();
    if (diffLevel == 0) {
      // set elevator for level zero
      elevatorPosition = -1.4;
    } else if (diffLevel == 1) {
      // set elevator for level one
      elevatorPosition = -1.4;
    } else if (diffLevel == 2) {
      // set elevator for level two
      elevatorPosition = -8.5;
    } else if (diffLevel == 3) {
      // set elevator for level three
      elevatorPosition = -22.7;
    } else if (diffLevel == 4) {
      // set elevator for level four
      elevatorPosition = -26.1;
    } else if (diffLevel == 10) {
      // set elevator for Algae Lower Level
      elevatorPosition = 0.01;
    } else if (diffLevel == 11) {
      // set elevator for Algae Upper Level
      elevatorPosition = -5.5;
    } else if (diffLevel == 23) {
      // set elevator for Scoring L3
      elevatorPosition = -1.5;
    } else if (diffLevel == 24) {
      // set elevator for Scoring L4
      elevatorPosition = -14.71;
    } else {
      // set elevator for else
      elevatorPosition = -0.2;
    }
    elevator.pidSetPosition(elevatorPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}