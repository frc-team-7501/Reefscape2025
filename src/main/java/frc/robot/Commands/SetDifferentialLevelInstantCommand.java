// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetDifferentialLevelInstantCommand extends InstantCommand {
  private Sensors sensors;
  private int diffLevel;
  public SetDifferentialLevelInstantCommand(Sensors sensors, int diffLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sensors = sensors;
    this.diffLevel = diffLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensors.setDifferentialLevel(diffLevel);
  }
}
