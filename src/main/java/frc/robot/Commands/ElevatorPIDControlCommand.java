// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.Elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ElevatorPIDControlCommand extends PIDCommand {
  private final Elevator elevator;

  /** Creates a new LiftPIDControlCommand. */
  public ElevatorPIDControlCommand(final Elevator elevator, final double position) {
    super(
        // The controller that the command will use
        new PIDController(0.04, 0, 0),
        // This should return the measurement
        () -> elevator.getElevatorPosition(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          elevator.moveElevator(position);
          //SmartDashboard.putNumber("Lift Output", output);
          SmartDashboard.putNumber("Lift",elevator.getElevatorPosition());
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(elevator);
    this.elevator = elevator;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(15);
    getController().setSetpoint(0);
  }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     elevator.moveElevator(0);
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}