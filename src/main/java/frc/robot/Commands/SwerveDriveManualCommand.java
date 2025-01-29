// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Sensors;

public class SwerveDriveManualCommand extends Command {
  /** Creates a new SwerveDriveManualCommand. */
  private final Drivetrain driveTrain;
  private final Sensors sensors;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotateSupplier;
  private final DoubleSupplier pixyTriggerSupplier;
  private final BooleanSupplier fieldRelative;

  public SwerveDriveManualCommand(
      final Drivetrain driveTrain,
      final Sensors sensors,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotateSupplier,
      DoubleSupplier pixyTriggerSupplier,
      BooleanSupplier fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.sensors = sensors;
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotateSupplier = rotateSupplier;
    this.pixyTriggerSupplier = pixyTriggerSupplier;
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: create a stop in Drivetrain
    // driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(
        (Math.signum(forwardSupplier.getAsDouble()) * Math.pow(forwardSupplier.getAsDouble(), 2)),
        (Math.signum(strafeSupplier.getAsDouble()) * Math.pow(strafeSupplier.getAsDouble(), 2)),
        (Math.signum(rotateSupplier.getAsDouble()) * Math.pow(rotateSupplier.getAsDouble(), 2)),
        fieldRelative.getAsBoolean(),
        sensors.getSpeedMultiplier(),
        pixyTriggerSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrain.drive(0, 0, 0, true);
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
