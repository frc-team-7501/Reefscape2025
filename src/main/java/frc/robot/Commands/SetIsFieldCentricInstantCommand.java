// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIsFieldCentricInstantCommand extends InstantCommand {
  private final Sensors sensors;
  private final boolean isCentric;
  public SetIsFieldCentricInstantCommand(Sensors sensors, boolean isCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sensors = sensors;
    this.isCentric = isCentric;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensors.setIsFieldCentric(isCentric);
  }
}
