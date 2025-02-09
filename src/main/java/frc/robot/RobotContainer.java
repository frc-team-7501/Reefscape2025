// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.ElevatorMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Commands.DifferentialControlCommand;
import frc.robot.Commands.DifferentialPIDControlCommand;
import frc.robot.Commands.ElevatorControlCommand;
import frc.robot.Commands.ElevatorPIDControlCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.ResetElevatorEncodersInstantCommand;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetIsFieldCentricInstantCommand;
import frc.robot.Commands.SetSpeedMultiplierInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonDriveCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Differential;
import frc.robot.Subsystems.Elevator;
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

    ////////////////////////////////
    // #region [ AUTON COMMANDS ]

    // #region DefaultAuton - DO NOT USE, NOT TESTED
    private final Command DefaultAuton = new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0)));

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
    private final Command elevatorControlCommand = new ElevatorControlCommand(elevator, () -> m_Xbox2.getLeftY());

    // Differential Manual Control
    private final Command differentialControlCommand = new DifferentialControlCommand(differential, () -> m_Xbox2.getRightY(), () -> m_Xbox2.getRightY() * -1);

    // #region Button Bindings
    private void configureButtonBindings() {

        // Back button on the drive controller resets gyroscope.
        m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));

        m_Xbox2.b_Back().onTrue(new ResetElevatorEncodersInstantCommand(elevator));

        // Turbo Button
        m_Xbox.b_RightBumper()
                .onTrue(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.TURBO_MULTIPLIER));
        m_Xbox.b_RightBumper()
                .onFalse(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.NORMAL_MULTIPLIER));

        // Elevator setpoints
        m_Xbox2.b_A()
                .onTrue(new ParallelCommandGroup(
                        new ElevatorPIDControlCommand(elevator, ElevatorMapping.Level0),
                        new IntakeControlCommand(intake, sensors, -0.5, false)
                        //, new DifferentialPIDControlCommand(differential, 0, 0)
                        ));
        m_Xbox2.b_X()
                .onTrue(new ParallelCommandGroup(
                        new ElevatorPIDControlCommand(elevator, ElevatorMapping.Level2),
                        new IntakeControlCommand(intake, sensors, 0.0, false)
                        //, new DifferentialPIDControlCommand(differential, 0, 0)
                        ));
        m_Xbox2.b_Y()
                .onTrue(new ElevatorPIDControlCommand(elevator, ElevatorMapping.Level3));
        m_Xbox2.b_B()
                .onTrue(new ElevatorPIDControlCommand(elevator, ElevatorMapping.Level4));

        
        // Field Centric Toggle
        m_Xbox.b_LeftBumper()
                .onTrue(new SetIsFieldCentricInstantCommand(sensors, false));
        m_Xbox.b_LeftBumper()
                .onFalse(new SetIsFieldCentricInstantCommand(sensors, true));
    }
    // #endregion'

    public RobotContainer() {

        configureButtonBindings();

        differential.setDefaultCommand(differentialControlCommand);
        driveTrain.setDefaultCommand(swerveDriveManualCommand);
        elevator.setDefaultCommand(elevatorControlCommand);
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
