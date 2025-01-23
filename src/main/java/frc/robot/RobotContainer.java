// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Commands.ResetGyroYawInstantCommand;
import frc.robot.Commands.SetIsFieldCentricInstantCommand;
import frc.robot.Commands.SetSpeedMultiplierInstantCommand;
import frc.robot.Commands.SwerveDriveManualCommand;
import frc.robot.Commands.Autonomous.AutonDriveCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Sensors;
import frc.robot.utils.ExtendedXboxController;

public class RobotContainer {
    private final ExtendedXboxController m_Xbox = new ExtendedXboxController(ControllerMapping.XBOX);
    private final ExtendedXboxController m_Xbox2 = new ExtendedXboxController(ControllerMapping.XBOX2);

    // create subsystems
    private final Drivetrain driveTrain = Drivetrain.getInstance();
    private final Sensors sensors = Sensors.getInstance();

    ////////////////////////////////
    // #region [ AUTON COMMANDS ]

    // #region DefaultAuton - DO NOT USE, NOT TESTED
    private final Command DefaultAuton = new AutonDriveCommand(driveTrain, new Pose2d(60, 0, new Rotation2d(0)));

    // #endregion
    ////////////////////////////////

    // Create commands
    private final Command swerveDriveManualCommand = new SwerveDriveManualCommand(
            driveTrain,
            sensors,
            () -> m_Xbox.getLeftY(),
            () -> m_Xbox.getLeftX(),
            () -> m_Xbox.getRightX(),
            () -> m_Xbox.getLeftTriggerAxis(),
            () -> sensors.getIsFieldCentric());

    public RobotContainer() {
        configureButtonBindings();
        driveTrain.setDefaultCommand(swerveDriveManualCommand);
    }

    // #region Button Bindings
    private void configureButtonBindings() {

        // Back button on the drive controller resets gyroscope.
        m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));

        // Turbo Button
        m_Xbox.b_RightBumper()
                .onTrue(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.TURBO_MULTIPLIER));
        m_Xbox.b_RightBumper()
                .onFalse(new SetSpeedMultiplierInstantCommand(sensors, MiscMapping.NORMAL_MULTIPLIER));

        // Field Centric Toggle
        m_Xbox.b_LeftBumper()
                .onTrue(new SetIsFieldCentricInstantCommand(sensors, false));
        m_Xbox.b_LeftBumper()
                .onFalse(new SetIsFieldCentricInstantCommand(sensors, true));
    }
    // #endregion

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
