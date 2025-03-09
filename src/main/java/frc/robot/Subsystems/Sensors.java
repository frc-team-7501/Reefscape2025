// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOMapping;
import frc.robot.Constants.MiscMapping;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Sensors extends SubsystemBase {
  /* Creates new Sensors. */
  private DigitalInput intakeSensor = new DigitalInput(DIOMapping.INTAKE_SENSOR);
  private DigitalInput climbSensor = new DigitalInput(DIOMapping.CLIMB_SENSOR);
  // Other "Fake" Sensors
  private boolean isFieldCentric;
  private double speedMultiplier;
  private static Sensors instance;
  private PhotonCamera photonCamera = new PhotonCamera("reefCam");
  private int targetID;
  private PhotonTrackedTarget target;
  private double photonYaw;
  private double photonArea;
  private int diffLevel;
  private int reefRotation = 0;
  private int LRC;

  public Sensors() {
    // Set the default delivery method to Launcher.
    isFieldCentric = true;
    speedMultiplier = MiscMapping.NORMAL_MULTIPLIER;
  }

  public static Sensors getInstance() {
    if (instance == null)
      instance = new Sensors();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("climbSen", climbSensor.get());
  }

  public void setReefRotation(int reefRotation) {
    this.reefRotation = reefRotation;
  }

  public int getReefRotation () {
    return reefRotation;
  }

  public void setDifferentialLevel(int diffLevelin) {
    this.diffLevel = diffLevelin;
  }

  public int getDifferentialLevel() {
    return diffLevel;
  }

  public void setLRCSelector(int LRC) {
    this.LRC = LRC;
  }

  public int getLRCSelector() {
    return LRC;
  }

  public boolean getIntakeSensor() {
    return !intakeSensor.get();
  }

  public boolean getClimbSensor() {
    return !climbSensor.get();
  }

  public boolean getIsFieldCentric() {
    return isFieldCentric;
  }

  public void setIsFieldCentric(boolean isCentric) {
    isFieldCentric = isCentric;
  }

  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public void setSpeedMultiplier(double multiplier) {
    speedMultiplier = multiplier;
  }

  public double[] getReefPosition() {
    var result = photonCamera.getLatestResult();
    target = result.getBestTarget();
    // Defaults values to zero
    double[] photonPositions = { 0.0, 0.0, 0.0, 0.0 };
    // If it sees a target
    if (result.hasTargets()) {
      targetID = target.getFiducialId();
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      if ((targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22)) {
        photonPositions[MiscMapping.VISX] = cameraToTarget.getTranslation().getX();
        photonPositions[MiscMapping.VISY] = cameraToTarget.getTranslation().getY();
        // Fix the rotation values for PID commands
        if (cameraToTarget.getRotation().getZ() * (180 / Math.PI) > 0) {
          photonPositions[MiscMapping.VISZ] = cameraToTarget.getRotation().getZ() * (180 / Math.PI) - 180;
        } else {
          photonPositions[MiscMapping.VISZ] = cameraToTarget.getRotation().getZ() * (180 / Math.PI) + 180;
        }
        photonPositions[MiscMapping.VISFOUND] = 1.0;
      }
    }
    SmartDashboard.putNumber("photonX", photonPositions[MiscMapping.VISX]);
    SmartDashboard.putNumber("photonY", photonPositions[MiscMapping.VISY]);
    SmartDashboard.putNumber("photonYaw", photonPositions[MiscMapping.VISZ]);
    SmartDashboard.putNumber("photonTarg", photonPositions[MiscMapping.VISFOUND]);
    return photonPositions;
  }
}
