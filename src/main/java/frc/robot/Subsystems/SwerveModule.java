// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  // Only for Trapezoidal PID commands.
  //private static final double kWheelRadius = 0.0508;
  //private static final int kEncoderResolution = 4096;
  //private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  //private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private double showTurnOutput;
  private TalonFX m_driveMotor;
  private SparkMax m_turningMotor;

  // private final CANSparkMax m_driveEncoder;
  private CANcoder m_turningEncoder;

  // Use if we use encoders for Drive Motors.
  //private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);
  
  //TODO: refine PID controller for turning PID
  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController = new PIDController(0.2, 0.02, 0);
  
  //private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
  //    0.1,
  //    0,
  //    0,
  //    new TrapezoidProfile.Constraints(
  //        kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));


  // Potential use option.
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotor     CAN address for the drive motor.
   * @param turningMotor   CAN address for the turning motor.
   * @param turningEncoder CAN address for the turning encoder.
   */

  public SwerveModule(
      int driveMotor,
      int turningMotor,
      int turningEncoderChannel) {
    m_driveMotor = new TalonFX(driveMotor);
    m_turningMotor = new SparkMax(turningMotor, MotorType.kBrushless);

    // Factory reset, so we get the SPARK MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // m_driveMotor.restoreFactoryDefaults();

    m_turningEncoder = new CANcoder(turningEncoderChannel);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getVelocity().getValueAsDouble(), getRotation());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getPosition().getValueAsDouble(), getRotation());
  }

  public Rotation2d getRotation() {
    return new Rotation2d(m_turningEncoder.getPosition().getValueAsDouble() * 2 * Math.PI);
  }

  public double showRotation() {
    return m_turningEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  public double showTurnPower() {
    return showTurnOutput;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // @SuppressWarnings("deprecation")
    @SuppressWarnings("deprecation")
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = state.speedMetersPerSecond;

    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition().getValueAsDouble() * 2 * Math.PI,
        state.angle.getRadians());
    showTurnOutput = turnOutput;

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(-turnOutput);
  }
}
