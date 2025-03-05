// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Differential;
import frc.robot.Subsystems.Sensors;

public class DifferentialPIDControlCommand extends Command {
  private final Differential differential;
  private final Sensors sensors;
  private int diffLevel;
  private final int reefLRCSelector;
  private double differentialPositionR;
  private double differentialPositionL;

  public DifferentialPIDControlCommand(Differential differential, Sensors sensors, int reefLRCSelector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.differential = differential;
    this.sensors = sensors;
    this.reefLRCSelector = reefLRCSelector;

    addRequirements(differential);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    diffLevel = sensors.getDifferentialLevel();
    if (sensors.getIntakeSensor() && diffLevel == 0) {
      differentialPositionR = 0.60;
      differentialPositionL = 0.60;
    } else if (diffLevel == 0) {
      // set differential for level zero
      differentialPositionR = 0.01;
      differentialPositionL = 0.01;
    } else if (diffLevel == 1) {
      // set differential for level one
      differentialPositionR = 0.13;
      differentialPositionL = 0.13;
    } else if (diffLevel == 2) {
      // set differential for level two
      differentialPositionR = -0.03;
      differentialPositionL = 0.29;
    } else if (diffLevel == 3) {
      // set differential for level three
      differentialPositionR = 0.44;
      differentialPositionL = 0.44;
    } else if (diffLevel == 4) {
      // set differential for level four
      differentialPositionR = 0.45;
      differentialPositionL = 0.45;
    } else if (diffLevel == 11) {
      // set differential for Algae UPPER Level
      differentialPositionR = 0.10;
      differentialPositionL = 0.43;
    } else if (diffLevel == 10) {
      // set differential for Algae LOWER Level
      differentialPositionR = 0.42;
      differentialPositionL = 0.42;
    } else if (diffLevel == 13) {
      // set differential for Autonomous Trough
      differentialPositionR = 0.02;
      differentialPositionL = 0.28;
    } else {
      // set differential for else
      differentialPositionR = 0.13;
      differentialPositionL = 0.13;
    }
    /********************************************************
     * turn the differential left, right, or return to center
     *******************************************************/
    if (reefLRCSelector == 0) {
      // Left
      differentialPositionL -= 0.04;
      differentialPositionR += 0.04;
      if (diffLevel == 2) {
        differentialPositionL += 0.02;
        differentialPositionR -= 0.02;
      }
    } else if (reefLRCSelector == 1) {
      // Right
      differentialPositionL += 0.04;
      differentialPositionR -= 0.04;
      if (diffLevel == 2) {
        differentialPositionL -= 0.02;
        differentialPositionR += 0.02;
      }
    } else if (reefLRCSelector == 2) {
      // Center
      differentialPositionL += 0.0;
      differentialPositionR += 0.0;
    }
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