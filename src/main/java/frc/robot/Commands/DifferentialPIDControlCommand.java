// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Differential;

public class DifferentialPIDControlCommand extends Command {
  /** Creates a new LiftControlCommand. */
  private final Differential differential;
  private final double differentialPositionR;
  private final double differentialPositionL;

  public DifferentialPIDControlCommand(Differential differential, double differentialPositionR, double differentialPositionL) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.differential = differential;
    this.differentialPositionR = differentialPositionR;
    this.differentialPositionL = differentialPositionL;

    addRequirements(differential);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    differential.pidSetPosition(differentialPositionR, differentialPositionL);
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