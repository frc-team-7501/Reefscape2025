// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Differential;
import java.util.function.DoubleSupplier;

public class DifferentialControlCommand extends Command {
  /** Creates a new Lift Control Command. */
  private final Differential differential;
  private final DoubleSupplier differentialSpeedR;
  // private final DoubleSupplier differentialSpeedL;
  private double differentialSpeedDoubleR = 0;  
  // private double differentialSpeedDoubleL = 0;

  public DifferentialControlCommand(Differential differential, DoubleSupplier differentialSpeedR) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.differential = differential;
    this.differentialSpeedR = differentialSpeedR;
    // this.differentialSpeedL = differentialSpeedL;
    addRequirements(differential);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    differentialSpeedDoubleR = differentialSpeedR.getAsDouble() * 0.2;
    // differentialSpeedDoubleL = differentialSpeedL.getAsDouble() * 0.2;

    // Set to zero to compensate for stick drift
    if (Math.abs(differentialSpeedDoubleR) < 0.05)
    differentialSpeedDoubleR = 0;

    // if (Math.abs(differentialSpeedDoubleL) < 0.05)
    // differentialSpeedDoubleL = 0;

    differential.moveDifferential(differentialSpeedDoubleR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    differential.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}