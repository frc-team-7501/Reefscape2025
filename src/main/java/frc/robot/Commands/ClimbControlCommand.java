// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ClimbControlCommand extends Command {
  // Creates a new ClimbControlCommand.
  private final Climb Climb;
  private final DoubleSupplier ClimbPower;
  private double ClimbPowerDouble;

  public ClimbControlCommand(Climb Climb, DoubleSupplier ClimbPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Climb = Climb;
    this.ClimbPower = ClimbPower;
    addRequirements(Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimbPowerDouble = ClimbPower.getAsDouble();
    // // Stop the lift from moving down (only) if either limit switch is triggered
    // if ((Climb.getClimbPosition() > 12.0) && ClimbPowerDouble > 0) {
    // Climb.moveClimb(0);
    // } else {

    if (ClimbPowerDouble < 0.2 && ClimbPowerDouble > -0.2) {
      ClimbPowerDouble = 0.0;
    }
      Climb.moveClimb(ClimbPowerDouble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}