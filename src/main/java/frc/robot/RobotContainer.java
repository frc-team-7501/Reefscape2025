// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.FunnelMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Commands.ClimbControlCommand;
import frc.robot.Commands.DifferentialControlCommand;
import frc.robot.Commands.DifferentialPIDControlCommand;
import frc.robot.Commands.ElevatorControlCommand;
import frc.robot.Commands.ElevatorPIDControlCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.ResetElevatorEncodersInstantCommand;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetDifferentialLevelInstantCommand;
import frc.robot.Commands.SetIsFieldCentricInstantCommand;
import frc.robot.Commands.SetSpeedMultiplierInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonAutoAlignCommand;
import frc.robot.Commands.Autonomous.AutonDriveCommand;
import frc.robot.Commands.Autonomous.AutonFunnelCommand;
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

    // #region DefaultAuton - DO NOT USE, NOT TESTED
    private final Command DefaultAuton = new SequentialCommandGroup(
            new InstantCommand(
                    () -> driveTrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
                    driveTrain),
                    new AutonDriveCommand(driveTrain, new Pose2d(-24, 0, new Rotation2d(0))),
            new ParallelRaceGroup(
                    new AutonAutoAlignCommand(driveTrain, sensors),
                    new WaitCommand(2)),
            new AutonFunnelCommand(funnel, FunnelMapping.LOWER_FUN)
    // "Elevator stuff here"
    // new AutonDriveCommand(driveTrain, new Pose2d(-24, -134, new Rotation2d(0))),
    // new AutonDriveCommand(driveTrain, new Pose2d(-35, -134, new Rotation2d(0)))
    );

    // #endregion
    ////////////////////////////////

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
                .onTrue(new IntakeControlCommand(intake, sensors, 1.0, true));
        m_Xbox.b_RightBumper()
                .onFalse(new IntakeControlCommand(intake, sensors, 0.0, true));

        // Funnel positions
        m_Xbox.b_X()
                .onTrue(new FunnelPIDControlCommand(funnel, FunnelMapping.UPPER_FUN));

        m_Xbox.b_Y()
                .onTrue(new FunnelPIDControlCommand(funnel, FunnelMapping.LOWER_FUN));

        // Elevator setpoints
        m_Xbox2.b_A()
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new SetDifferentialLevelInstantCommand(sensors, 0),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors, 2)));
        m_Xbox2.b_X()
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 1),
                        new ParallelRaceGroup(
                                new DifferentialPIDControlCommand(differential, sensors,
                                        2),
                                new WaitCommand(1)),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, -1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 2),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential, sensors,
                                        2))));
        m_Xbox2.b_Y()
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new SetDifferentialLevelInstantCommand(sensors, 3),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors, 2)));
        m_Xbox2.b_B()
                .onTrue(new ParallelCommandGroup(
                        new IntakeControlCommand(intake, sensors, -1.0, false),
                        new SetDifferentialLevelInstantCommand(sensors, 4),
                        new ElevatorPIDControlCommand(elevator, sensors),
                        new DifferentialPIDControlCommand(differential, sensors, 2)));
        // Algae removal positions
        m_Xbox.b_A()
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 10),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, 1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 10),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential, sensors, 2))));
        m_Xbox.b_B()
                .onTrue(new SequentialCommandGroup(
                        new SetDifferentialLevelInstantCommand(sensors, 11),
                        new ParallelRaceGroup(
                                new DifferentialPIDControlCommand(differential, sensors,
                                        2),
                                new WaitCommand(2)),
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, sensors, 1.0, false),
                                new SetDifferentialLevelInstantCommand(sensors, 11),
                                new ElevatorPIDControlCommand(elevator, sensors),
                                new DifferentialPIDControlCommand(differential, sensors,
                                        2))));

        // Field Centric Toggle
        m_Xbox.b_LeftBumper()
                .onTrue(new SetIsFieldCentricInstantCommand(sensors, false));
        m_Xbox.b_LeftBumper()
                .onFalse(new SetIsFieldCentricInstantCommand(sensors, true));

        // Reef scoring selection
        m_Xbox2.b_RightBumper().onTrue(new ParallelCommandGroup(
                new IntakeControlCommand(intake, sensors, 1.0, false),
                new ElevatorPIDControlCommand(elevator, sensors),
                new DifferentialPIDControlCommand(differential, sensors, 1)));

        m_Xbox2.b_RightBumper().onFalse(new ParallelCommandGroup(
                new IntakeControlCommand(intake, sensors, 1.0, false),
                new ElevatorPIDControlCommand(elevator, sensors),
                new DifferentialPIDControlCommand(differential, sensors, 2)));

        m_Xbox2.b_LeftBumper().onTrue(new ParallelCommandGroup(
                new IntakeControlCommand(intake, sensors, 1.0, false),
                new ElevatorPIDControlCommand(elevator, sensors),
                new DifferentialPIDControlCommand(differential, sensors, 0)));

        m_Xbox2.b_LeftBumper().onFalse(new ParallelCommandGroup(
                new IntakeControlCommand(intake, sensors, 1.0, false),
                new ElevatorPIDControlCommand(elevator, sensors),
                new DifferentialPIDControlCommand(differential, sensors, 2)));
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
        return DefaultAuton;
    }
}
