// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sensors;

public class IntakeControlCommand extends Command {
  /** Creates a new LaunchCommand. */
  private final Intake Intake;
  private double intakeSpeed;
  private boolean override;
  private Sensors sensors;

  public IntakeControlCommand(Intake intake, Sensors sensors, Double IntakeSpeed, boolean override) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake = intake;
    this.intakeSpeed = IntakeSpeed;
    this.override = override;
    this.sensors = sensors;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Intake.runIntake(intakeSpeed, override, sensors.getIntakeSensor());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}