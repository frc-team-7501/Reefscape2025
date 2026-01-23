// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Funnel;

public class FunnelPIDControlCommand extends Command {
  /** Creates a new FunnelPIDControlCommand. */
  private final Funnel funnel;
  private final double funnelPosition;

  public FunnelPIDControlCommand(Funnel funnel, double funnelPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.funnel = funnel;
    this.funnelPosition = funnelPosition;
    addRequirements(funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnel.pidSetPosition(funnelPosition);
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