// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonBoardMapping;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.FunnelMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Commands.ClimbControlCommand;
import frc.robot.Commands.DifferentialPIDControlCommand;
import frc.robot.Commands.ElevatorPIDControlCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.ResetElevatorEncodersInstantCommand;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetDifferentialLevelInstantCommand;
import frc.robot.Commands.SetIsFieldCentricInstantCommand;
import frc.robot.Commands.SetLRCSelectorInstantCommand;
import frc.robot.Commands.SetReefRotationInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonBeamBreakCommand;
import frc.robot.Commands.Autonomous.AutonDriveCommand;
import frc.robot.Commands.FunnelControlCommand;
import frc.robot.Commands.FunnelPIDControlCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Differential;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Funnel;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sensors;
import frc.robot.utils.ExtendedXboxController;

public class RobotContainer {
    private final ExtendedXboxController m_Xbox = new ExtendedXboxController(ControllerMapping.XBOX);
    private final ExtendedXboxController m_Xbox2 = new ExtendedXboxController(ControllerMapping.XBOX2);
    private final GenericHID m_board = new GenericHID(ControllerMapping.BBOARD);

    // create subsystems
    private final Differential differential = Differential.getInstance();
    private final Drivetrain driveTrain = Drivetrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Sensors sensors = Sensors.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Funnel funnel = Funnel.getInstance();
    private final Climb climb = Climb.getInstance();

    ////////////////////////////////
    // #region [ AUTON COMMANDS ]

    // #region LeftBreakout
    private final Command LeftBreakout = new SequentialCommandGroup(
            // new ResetGyroYawInstantCommand(driveTrain),
            new SetLRCSelectorInstantCommand(sensors, 0),
            new InstantCommand(
                    // Rotations in degrees must be multiplied by: (Math.PI / 180)
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move to first auto-align point
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new DifferentialPIDControlCommand(differential, sensors),
                    new AutonDriveCommand(driveTrain, (new Pose2d(30, 0, new Rotation2d((Math.PI / 180) * 180))))),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),
                        
            new SetLRCSelectorInstantCommand(sensors, 1),

            // Move to wall
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, -120, new Rotation2d((Math.PI / 180) * 300)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Move to source
            new ParallelRaceGroup(
                    new AutonBeamBreakCommand(sensors),
                    new ParallelDeadlineGroup(new WaitCommand(7),
                            new AutonDriveCommand(driveTrain,
                                    (new Pose2d(210, -120, new Rotation2d((Math.PI / 180) * 300)))),
                            new IntakeControlCommand(intake, sensors, -1.0, false),
                            new SetDifferentialLevelInstantCommand(sensors, 0),
                            new ElevatorPIDControlCommand(elevator, sensors),
                            new DifferentialPIDControlCommand(differential, sensors))),

            // Move to wall
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(50, -120, new Rotation2d((Math.PI / 180) * 180)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Move to Auto-Align
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(50, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.5),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(40, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(1),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors))
    );
    // #endregion LeftBreakout

    // #region RightBreakout
    private final Command RightBreakout = new SequentialCommandGroup(
            // new ResetGyroYawInstantCommand(driveTrain),
            new SetLRCSelectorInstantCommand(sensors, 1),
            new InstantCommand(
                    // Rotations in degrees must be multiplied by: (Math.PI / 180)
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move to first auto-align point
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new DifferentialPIDControlCommand(differential, sensors),
                    new AutonDriveCommand(driveTrain, (new Pose2d(30, 0, new Rotation2d((Math.PI / 180) * 180))))),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),
                        
            new SetLRCSelectorInstantCommand(sensors, 0),

            // Move to wall
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 120, new Rotation2d((Math.PI / 180) * 60)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Move to source
            new ParallelRaceGroup(
                    new AutonBeamBreakCommand(sensors),
                    new ParallelDeadlineGroup(new WaitCommand(7),
                            new AutonDriveCommand(driveTrain,
                                    (new Pose2d(210, 120, new Rotation2d((Math.PI / 180) * 60)))),
                            new IntakeControlCommand(intake, sensors, -1.0, false),
                            new SetDifferentialLevelInstantCommand(sensors, 0),
                            new ElevatorPIDControlCommand(elevator, sensors),
                            new DifferentialPIDControlCommand(differential, sensors))),

            // Move to wall
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 120, new Rotation2d((Math.PI / 180) * 180)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Move to Auto-Align
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(1),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors))
    );
    // #endregion RightBreakout

    // #region CenterLeftL4
    private final Command CenterLeftL4 = new SequentialCommandGroup(
            // new ResetGyroYawInstantCommand(driveTrain),
            new SetLRCSelectorInstantCommand(sensors, 0),
            new InstantCommand(
                    // Rotations in degrees must be multiplied by: (Math.PI / 180)
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move to first auto-align point
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new DifferentialPIDControlCommand(differential, sensors),
                    new AutonDriveCommand(driveTrain, (new Pose2d(30, 0, new Rotation2d((Math.PI / 180) * 180))))),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.25),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))));
    // #endregion CenterLeftL4

    // #region CenterRightL4
    private final Command CenterRightL4 = new SequentialCommandGroup(
            // new ResetGyroYawInstantCommand(driveTrain),
            new SetLRCSelectorInstantCommand(sensors, 1),
            new InstantCommand(
                    // Rotations in degrees must be multiplied by: (Math.PI / 180)
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move to first auto-align point
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new DifferentialPIDControlCommand(differential, sensors),
                    new AutonDriveCommand(driveTrain, (new Pose2d(30, 0, new Rotation2d((Math.PI / 180) * 180))))),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.25),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(20, 0, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)));
    // #endregion CenterRightL4

    // #region RightL4Single
    private final Command RightL4Single = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetPose(new Pose2d(0, 0, new Rotation2d(180))),
                    driveTrain),

            // Moves Funnel, Differential, and Elevator to the highest Reef Level
            new ParallelDeadlineGroup(new WaitCommand(1),

                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetLRCSelectorInstantCommand(sensors, 0),
            new SetIsFieldCentricInstantCommand(sensors, false),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(5),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Outputs the Coral onto the Reef for half a second
            new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Backup
            new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.25, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetIsFieldCentricInstantCommand(sensors, true));

    // #endregion RightL4Single

    // #region LeftL4Single
    private final Command LeftL4Single = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetPose(new Pose2d(0, 0, new Rotation2d(180))),
                    driveTrain),

            // Moves Funnel, Differential, and Elevator to the highest Reef Level
            new ParallelDeadlineGroup(new WaitCommand(1),

                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetLRCSelectorInstantCommand(sensors, 1),
            new SetIsFieldCentricInstantCommand(sensors, false),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(5),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Outputs the Coral onto the Reef for half a second
            new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Backup
            new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.25, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0,
                            () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetIsFieldCentricInstantCommand(sensors, true));

    // #endregion LeftL4Single

    // #region PushRightL4
    private final Command PushRightL4 = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move the other bot fowards
            new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(5, 0, new Rotation2d((Math.PI / 180) * 180))))),

            // Move to auto align starting position
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(0, -48, new Rotation2d((Math.PI / 180) * 180))))),

            // Move again to auto align position
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(60, -48, new Rotation2d((Math.PI / 180) * 180)))),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetLRCSelectorInstantCommand(sensors, 1),
            new SetIsFieldCentricInstantCommand(sensors, false),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.5),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(60, -48, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetIsFieldCentricInstantCommand(sensors, true),

            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(60, -48, new Rotation2d((Math.PI / 180) * 60))))),

            new ParallelDeadlineGroup(new WaitCommand(4),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(127, 68, new Rotation2d((Math.PI / 180) * 60))))),

            new ParallelDeadlineGroup(new WaitCommand(4),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(262, 78, new Rotation2d((Math.PI / 180) * 60))))));
    // #endregion PushRightL4

    // #region PushLeftL4
    private final Command PushLeftL4 = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move the other bot fowards
            new ParallelDeadlineGroup(new WaitCommand(0.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(5, 0, new Rotation2d((Math.PI / 180) * 180))))),

            // Move to auto align starting position
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(0, 48, new Rotation2d((Math.PI / 180) * 180))))),

            // Move again to auto align position
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(60, 48, new Rotation2d((Math.PI / 180) * 180)))),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetLRCSelectorInstantCommand(sensors, 0),
            new SetIsFieldCentricInstantCommand(sensors, false),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.5),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Outputs coral onto the L4 Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(60, 48, new Rotation2d((Math.PI / 180) * 180)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> true),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            new SetIsFieldCentricInstantCommand(sensors, true),

            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(60, 48, new Rotation2d((Math.PI / 180) * 300))))),

            new ParallelDeadlineGroup(new WaitCommand(4),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(127, -68, new Rotation2d((Math.PI / 180) * 300))))),

            new ParallelDeadlineGroup(new WaitCommand(4),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(262, -78, new Rotation2d((Math.PI / 180) * 300))))));
    // #endregion PushLeftL4

    // #region RightStation Competition Ready
    private final Command RightStation = new SequentialCommandGroup(
            // new ResetGyroYawInstantCommand(driveTrain),
            new InstantCommand(
                    // Rotations in degrees must be multiplied by: (Math.PI / 180)
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),

            // Move to first auto-align point
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new DifferentialPIDControlCommand(differential, sensors),
                    new AutonDriveCommand(driveTrain, (new Pose2d(75, 0, new Rotation2d((Math.PI / 180) * 135))))),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.25),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs the Coral onto the Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(75, 15, new Rotation2d((Math.PI / 180) * 120)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ConditionalCommand(
                    // on true
                    new ParallelDeadlineGroup(new WaitCommand(2),
                            new AutonDriveCommand(driveTrain,
                                    (new Pose2d(100, 15, new Rotation2d((Math.PI / 180) * 50)))),
                            new IntakeControlCommand(intake, sensors, -1.0, false),
                            new SetDifferentialLevelInstantCommand(sensors, 0),
                            new ElevatorPIDControlCommand(elevator, sensors),
                            new DifferentialPIDControlCommand(differential, sensors)),
                    // on false
                    new ParallelRaceGroup(
                            new AutonBeamBreakCommand(sensors),
                            new ParallelDeadlineGroup(new WaitCommand(5),
                                    new AutonDriveCommand(driveTrain,
                                            (new Pose2d(235, 65, new Rotation2d((Math.PI / 180) * 60)))),
                                    new IntakeControlCommand(intake, sensors, -1.0, false),
                                    new SetDifferentialLevelInstantCommand(sensors, 0),
                                    new ElevatorPIDControlCommand(elevator, sensors),
                                    new DifferentialPIDControlCommand(differential, sensors))),
                    // conditional
                    () -> sensors.getIntakeSensor()),

            // Start moving to the second dropoff point
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(150, 0, new Rotation2d((Math.PI / 180) * 50)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(1),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1),
                                    new DifferentialPIDControlCommand(differential, sensors)))),

            // Outputs the Coral onto the Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(150, 0, new Rotation2d((Math.PI / 180) * 60)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors),
                                    new DifferentialPIDControlCommand(differential, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelRaceGroup(
                    new AutonBeamBreakCommand(sensors),
                    new ParallelDeadlineGroup(new WaitCommand(5),
                            new AutonDriveCommand(driveTrain,
                                    (new Pose2d(235, 65, new Rotation2d((Math.PI / 180) * 60)))),
                            new IntakeControlCommand(intake, sensors, -1.0, false),
                            new SetDifferentialLevelInstantCommand(sensors, 0),
                            new ElevatorPIDControlCommand(elevator, sensors),
                            new DifferentialPIDControlCommand(differential, sensors))));

    // #endregion RightStation

    // #region LeftStation Competition Ready

    private final Command LeftStation = new SequentialCommandGroup(
            // new ResetGyroYawInstantCommand(driveTrain),
            new InstantCommand(
                    // Rotations in degrees must be multiplied by: (Math.PI / 180)
                    () -> driveTrain.resetPose(
                            new Pose2d(0, 0, new Rotation2d((Math.PI / 180) * 180))),
                    driveTrain),
            new SetLRCSelectorInstantCommand(sensors, 1),

            // Move to first auto-align point
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new DifferentialPIDControlCommand(differential, sensors),
                    new AutonDriveCommand(driveTrain, (new Pose2d(75, 0, new Rotation2d((Math.PI / 180) * 225))))),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2.25),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.5),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1.75),
                                    new DifferentialPIDControlCommand(differential, sensors),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Outputs the Coral onto the Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain, (new Pose2d(75, -15, new Rotation2d((Math.PI / 180) * 225)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ConditionalCommand(
                    // on true
                    new ParallelDeadlineGroup(new WaitCommand(2),
                            new AutonDriveCommand(driveTrain,
                                    (new Pose2d(110, -25, new Rotation2d((Math.PI / 180) * 310)))),
                            new IntakeControlCommand(intake, sensors, -1.0, false),
                            new SetDifferentialLevelInstantCommand(sensors, 0),
                            new ElevatorPIDControlCommand(elevator, sensors),
                            new DifferentialPIDControlCommand(differential, sensors)),
                    // on false
                    new ParallelRaceGroup(
                            new AutonBeamBreakCommand(sensors),
                            new ParallelDeadlineGroup(new WaitCommand(5),
                                    new AutonDriveCommand(driveTrain,
                                            (new Pose2d(235, -65, new Rotation2d((Math.PI / 180) * 300)))),
                                    new IntakeControlCommand(intake, sensors, -1.0, false),
                                    new SetDifferentialLevelInstantCommand(sensors, 0),
                                    new ElevatorPIDControlCommand(elevator, sensors),
                                    new DifferentialPIDControlCommand(differential, sensors))),
                    // conditional
                    () -> sensors.getIntakeSensor()),

            // Start moving to the second dropoff point
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(150, 0, new Rotation2d((Math.PI / 180) * 310)))),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 0),
                    new ElevatorPIDControlCommand(elevator, sensors),
                    new DifferentialPIDControlCommand(differential, sensors)),

            // Auto Align using Vision to April Tags
            new ParallelDeadlineGroup(new WaitCommand(2),
                    new SwerveDriveManualCommand(driveTrain, sensors, () -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0,
                            () -> false),
                    new IntakeControlCommand(intake, sensors, -1.0, false),
                    new SetDifferentialLevelInstantCommand(sensors, 4),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(1),
                                    new ElevatorPIDControlCommand(elevator, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(1),
                                    new DifferentialPIDControlCommand(differential, sensors)))),

            // Outputs the Coral onto the Reef
            new ParallelDeadlineGroup(new WaitCommand(1.5),
                    new AutonDriveCommand(driveTrain,
                            (new Pose2d(150, 0, new Rotation2d((Math.PI / 180) * 300)))),
                    new SetDifferentialLevelInstantCommand(sensors, 24),
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(new WaitCommand(0.2),
                                    new DifferentialPIDControlCommand(differential, sensors)),
                            new ParallelDeadlineGroup(new WaitCommand(0.4),
                                    new ElevatorPIDControlCommand(elevator, sensors),
                                    new DifferentialPIDControlCommand(differential, sensors)))),

            // Stop moving and retract scoring elements to home position
            new ParallelRaceGroup(
                    new AutonBeamBreakCommand(sensors),
                    new ParallelDeadlineGroup(new WaitCommand(5),
                            new AutonDriveCommand(driveTrain,
                                    (new Pose2d(235, -65, new Rotation2d((Math.PI / 180) * 300)))),
                            new IntakeControlCommand(intake, sensors, -1.0, false),
                            new SetDifferentialLevelInstantCommand(sensors, 0),
                            new ElevatorPIDControlCommand(elevator, sensors),
                            new DifferentialPIDControlCommand(differential, sensors))));

    // #endregion LeftStation
    ////////////////////////////////
    // #endregion

    // Create commands

    // Swerve Control
    private final Command swerveDriveManualCommand = new SwerveDriveManualCommand(
            driveTrain,
            sensors,
            () -> m_Xbox.getLeftY(),
            () -> m_Xbox.getLeftX(),
            () -> m_Xbox.getRightX(),
            () -> m_Xbox.getLeftTriggerAxis(),
            () -> sensors.getIsFieldCentric());

    // Elevator Manual Control
    // private final Command elevatorControlCommand = new
    // ElevatorControlCommand(elevator, () -> m_Xbox2.getLeftY());

    // Climb Manual Control
    private final Command climbControlCommand = new ClimbControlCommand(climb, () -> m_Xbox2.getLeftY() * 0.5,
            sensors);

    // Funnel Manual Control
    private final Command funnelControlCommand = new FunnelControlCommand(funnel, () -> m_Xbox2.getRightY());

    // Differential Manual Control
    // private final Command differentialControlCommand = new
    // DifferentialControlCommand(differential, () -> m_Xbox2.getRightY(), () ->
    // m_Xbox2.getRightY() * -1);

    /*************************** BUTTONS************************** */
    // #region Button Bindings
    private void configureButtonBindings() {

        // Back button on the drive controller resets gyroscope.
        m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));

        // Reset the differential encoders.
        m_Xbox2.b_Back().onTrue(new ResetElevatorEncodersInstantCommand(elevator));

        // Turbo Button
        // m_Xbox.b_RightBumper()
        // .onTrue(new SetSpeedMultiplierInstantCommand(sensors,
        // MiscMapping.TURBO_MULTIPLIER));
        // m_Xbox.b_RightBumper()
        // .onFalse(new SetSpeedMultiplierInstantCommand(sensors,
        // MiscMapping.NORMAL_MULTIPLIER));

        // Output
        m_Xbox.b_RightBumper()
                .onTrue(new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SetDifferentialLevelInstantCommand(sensors, 24),
                                new WaitCommand(0.0),
                                () -> sensors.getDifferentialLevel() == 4),
                        new ConditionalCommand(
                                new SetDifferentialLevelInstantCommand(sensors, 23),
                                new WaitCommand(0.0),
                                () -> sensors.getDifferentialLevel() == 3),
                        new ConditionalCommand(
                                new IntakeControlCommand(intake, sensors, -1.0, true),
                                new WaitCommand(0.0),
                                () -> ((sensors.getDifferentialLevel() == 2) || (sensors
                                        .getDifferentialLevel() == 1))),
                        new DifferentialPIDControlCommand(differential, sensors),
                        new ElevatorPIDControlCommand(elevator, sensors)
                // new IntakeControlCommand(intake, sensors, -1.0, false),
                ));
        m_Xbox.b_RightBumper()
                .onFalse(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors)));

        // Field Centric Toggle
        m_Xbox.b_LeftBumper()
                .onTrue(new SetIsFieldCentricInstantCommand(sensors, false));
        m_Xbox.b_LeftBumper()
                .onFalse(new SetIsFieldCentricInstantCommand(sensors, true));

        // Funnel positions
        new JoystickButton(m_board, ButtonBoardMapping.BB_CLIMBUP)
                .onTrue(new FunnelPIDControlCommand(funnel, FunnelMapping.UPPER_FUN));

        new JoystickButton(m_board, ButtonBoardMapping.BB_CLIMBDOWN)
                .onTrue(new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN));

        // Elevator setpoints
        new JoystickButton(m_board, ButtonBoardMapping.BB_TRANSFER)
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new SetDifferentialLevelInstantCommand(sensors, 0),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors)));

        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF1)
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 1),
                        new ParallelRaceGroup(
                                new DifferentialPIDControlCommand(differential,
                                        sensors),
                                new WaitCommand(1)),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, -1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 1),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential,
                                        sensors))));

        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF2)
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 2),
                        new ParallelRaceGroup(
                                new DifferentialPIDControlCommand(differential,
                                        sensors),
                                new WaitCommand(1),
                                new IntakeControlCommand(intake, sensors, -1.0, false)),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, -1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 2),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential,
                                        sensors))));

        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF3)
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new SetDifferentialLevelInstantCommand(sensors, 3),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors)));

        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF4)
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new SetDifferentialLevelInstantCommand(sensors, 4),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors)));

        // Algae removal positions
        new JoystickButton(m_board, ButtonBoardMapping.BB_ALGAEL)
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 10),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, -1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 10),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential,
                                        sensors))));

        new JoystickButton(m_board, ButtonBoardMapping.BB_ALGAEU)
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 11),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, -1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 11),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential,
                                        sensors))));

        // Reef Rotation
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS1)
                .onTrue(new SetReefRotationInstantCommand(sensors, 1));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS1)
                .onFalse(new SetReefRotationInstantCommand(sensors, 0));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS2)
                .onTrue(new SetReefRotationInstantCommand(sensors, 2));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS2)
                .onFalse(new SetReefRotationInstantCommand(sensors, 0));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS3)
                .onTrue(new SetReefRotationInstantCommand(sensors, 3));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS3)
                .onFalse(new SetReefRotationInstantCommand(sensors, 0));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS4)
                .onTrue(new SetReefRotationInstantCommand(sensors, 4));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS4)
                .onFalse(new SetReefRotationInstantCommand(sensors, 0));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS5)
                .onTrue(new SetReefRotationInstantCommand(sensors, 5));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS5)
                .onFalse(new SetReefRotationInstantCommand(sensors, 0));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS6)
                .onTrue(new SetReefRotationInstantCommand(sensors, 6));
        new JoystickButton(m_board, ButtonBoardMapping.BB_REEF_POS6)
                .onFalse(new SetReefRotationInstantCommand(sensors, 0));

        // Reef scoring selection
        new JoystickButton(m_board, ButtonBoardMapping.BB_RIGHTALIGN)
                .onTrue(new SetLRCSelectorInstantCommand(sensors, 1));
        new JoystickButton(m_board, ButtonBoardMapping.BB_LEFTALIGN)
                .onTrue(new SetLRCSelectorInstantCommand(sensors, 0));
    }
    // #endregion

    public RobotContainer() {

        configureButtonBindings();

        // differential.setDefaultCommand(differentialControlCommand);
        // differential.setDefaultCommand(differentialPIDControlCommand);

        driveTrain.setDefaultCommand(swerveDriveManualCommand);
        // elevator.setDefaultCommand(elevatorControlCommand);
        funnel.setDefaultCommand(funnelControlCommand);
        climb.setDefaultCommand(climbControlCommand);
    }

    // #region TeleopInit
    public void teleopInit() {
        driveTrain.setBrakeMode(MiscMapping.BRAKE_OFF);
    }
    // #endregion

    // #region AutonomousInit
    public void autonomousInit() {
        driveTrain.setBrakeMode(MiscMapping.BRAKE_ON);
        driveTrain.resetYaw();
    }
    // #endregion

    public Command getAutonomousCommand() {
        // return RightL4Single;
        // return LeftL4Single;
        // return CenterRightL4; // Ready
        // return CenterLeftL4; // Ready
        // return RightBreakout; // Ready
        // return LeftBreakout; // Ready
        return RightStation; // Ready
        // return LeftStation; // Ready
        // return PushRightL4; // Ready
        // return PushLeftL4; // Ready
    }
}