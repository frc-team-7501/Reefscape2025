// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

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

  public void setDifferentialLevel(int diffLevelin) {
    this.diffLevel = diffLevelin;
  }

  public int getDifferentialLevel() {
    return diffLevel; 
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

  public double getPhotonVisionYaw() {
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) {
      // Sees a target
      target = result.getBestTarget();
      targetID = target.getFiducialId();
      SmartDashboard.putNumber("photonYaw", target.getYaw());
      if ((targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22)) {
        photonYaw = target.getYaw();
        return photonYaw;
      } else {
        return 0.0;
      }
    } else {
      // Doesn't see a target
      return 0.0;
    }
  }

  public double getPhotonVisionArea() {
    var result = photonCamera.getLatestResult();
    SmartDashboard.putBoolean("hasTarget", result.hasTargets());
    if (result.hasTargets()) {
      // Sees a target
      target = result.getBestTarget();
      targetID = target.getFiducialId();
      SmartDashboard.putNumber("targetID", targetID);
      SmartDashboard.putNumber("photonArea", target.getArea());
      if ((targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22)) {
        photonArea = target.getArea();
        return photonArea - MiscMapping.PHOTON_AREA_GOAL;
      } else {
        return 0.0;
      }
    } else {
      // Doesn't see a target
      return 0.0;
    }
  }


  
}
