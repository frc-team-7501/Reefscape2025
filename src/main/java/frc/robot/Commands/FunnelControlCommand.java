// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Funnel;
import java.util.function.DoubleSupplier;

public class FunnelControlCommand extends Command {
  /** Creates a new LiftControlCommand. */
  private final Funnel funnel;
  private final DoubleSupplier funnelSpeed;
  private double funnelSpeedDouble = 0;

  public FunnelControlCommand(Funnel funnel, DoubleSupplier funnelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.funnel = funnel;
    this.funnelSpeed = funnelSpeed;
    addRequirements(funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSpeedDouble = funnelSpeed.getAsDouble() * 0.2;

    // Set to zero to compensate for stick drift
    if (Math.abs(funnelSpeedDouble) < 0.05)
    funnelSpeedDouble = 0;

    funnel.moveFunnel(funnelSpeedDouble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}