// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorControlCommand extends Command {
  /** Creates a new LiftControlCommand. */
  private final Elevator elevator;
  private final DoubleSupplier elevatorSpeed;
  private double elevatorSpeedDouble = 0;

  public ElevatorControlCommand(Elevator elevator, DoubleSupplier elevatorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.elevatorSpeed = elevatorSpeed;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSpeedDouble = elevatorSpeed.getAsDouble() * 0.2;

    // Set to zero to compensate for stick drift
    if (Math.abs(elevatorSpeedDouble) < 0.05)
      elevatorSpeedDouble = 0;

    elevator.moveElevator(elevatorSpeedDouble);
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