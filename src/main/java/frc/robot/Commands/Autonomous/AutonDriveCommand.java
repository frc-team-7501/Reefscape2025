package frc.robot.Commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.MiscMapping;
import frc.robot.Subsystems.Drivetrain;

public class AutonDriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController angleController;
    private final Pose2d targetPose2d;
    private static final double DriveSpeed = 148; // inches per second, max value of 220
    private static final double DriveAcceleration = 96; // *should be in inches/s

    private static final double DriveP = 0.02;
    private static final double DriveI = 0.001;
    private static final double DriveD = 0.0;

    // trapezoidal PID controller for X position control
    private final ProfiledPIDController X_PID_Controller = new ProfiledPIDController(
            DriveP,
            DriveI,
            DriveD,
            new TrapezoidProfile.Constraints(
                    DriveSpeed,
                    DriveAcceleration));

                      //X_PID_Controller.trapezoidal.Constraints(DriveSpeed, DriveAcceleration);

    // trapezoidal PID controller for X position control
    private final ProfiledPIDController Y_PID_Controller = new ProfiledPIDController(
            DriveP,
            DriveI,
            DriveD,
            new TrapezoidProfile.Constraints(
                    DriveSpeed,
                    DriveAcceleration));

    public AutonDriveCommand(final Drivetrain drivetrain, final Pose2d targetPose2d) {
        super();

        this.drivetrain = drivetrain;
        this.targetPose2d = targetPose2d;

        xController = Constants.DriveTrain.PID_X.toPIDController();
        yController = Constants.DriveTrain.PID_Y.toPIDController();
        angleController = Constants.DriveTrain.PID_T.toPIDController();

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

        X_PID_Controller.setGoal(targetPose2d.getX() * MiscMapping.xConversionInches);
        Y_PID_Controller.setGoal(targetPose2d.getY() * MiscMapping.xConversionInches);
        X_PID_Controller.setTolerance(3);
        Y_PID_Controller.setTolerance(3);
        // yController.setSetpoint(targetPose2d.getY() * MiscMapping.yConversionInches);
        angleController.setSetpoint(targetPose2d.getRotation().getRadians());
        SmartDashboard.putNumber("target X", targetPose2d.getX());
        // SmartDashboard.putNumber("target Y", targetPose2d.getY() *
        // MiscMapping.yConversionInches);
        // SmartDashboard.putNumber("target Z",
        // targetPose2d.getRotation().getRadians());
    }

    private double clampOutput(double val, double limit) {
        return Math.signum(val) * Math.min(Math.abs(val), limit);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double outputX = X_PID_Controller.calculate(currentPose.getX());
        double outputY = Y_PID_Controller.calculate(currentPose.getY());

        // double outputX = X_PID_Controller.calculate(currentPose.getX(),
        // targetPose2d.getX() * MiscMapping.xConversionInches);
        // double outputX = xController.calculate(currentPose.getX());
        // double outputX = 0;
        // double outputY = yController.calculate(currentPose.getY());
        // double outputY = 0;
        double outputT = angleController.calculate(currentPose.getRotation().getRadians());
        // double outputT = 0;

        SmartDashboard.putNumber("outputX1", outputX);
        SmartDashboard.putNumber("outputY1", outputY);
        SmartDashboard.putNumber("outputT1", outputT);
        SmartDashboard.putNumber("Xpos", currentPose.getX() / MiscMapping.xConversionInches);
        SmartDashboard.putNumber("Ypos", currentPose.getY() / MiscMapping.yConversionInches);
        SmartDashboard.putNumber("Tpos", currentPose.getRotation().getRadians());

        // drivetrain.driveRawFieldRelative
        outputX = clampOutput(outputX, 1);
        outputY = clampOutput(outputY, 1);
        outputT = clampOutput(outputT, 0.75);

        // .putNumber("Clamped outputX", outputX);
        // SmartDashboard.putNumber("Clamped outputY", outputY);
        // SmartDashboard.putNumber("Clamped outputT", outputT);

        drivetrain.drive(-outputX, -outputY, -outputT, true, 1, 0.5, 0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {

        SmartDashboard.putBoolean("x atSetpoint", X_PID_Controller.atSetpoint());
        SmartDashboard.putBoolean("y atSetpoint", Y_PID_Controller.atSetpoint());
        SmartDashboard.putBoolean("angle atSetpoint", angleController.atSetpoint());

        boolean condition = X_PID_Controller.atGoal() && Y_PID_Controller.atGoal() && angleController.atSetpoint();

        // SmartDashboard.putBoolean("condition", condition);

        return condition;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonDriveCommand.end() called!");
        // drivetrain.drive(0, 0, 0, true);
        drivetrain.stop();
    }
}
