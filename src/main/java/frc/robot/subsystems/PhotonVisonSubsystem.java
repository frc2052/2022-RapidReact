// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;

public class PhotonVisonSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera(Constants.USB_CAMERA_NAME);
  PhotonPipelineResult result;
  boolean hasTarget;

  /** Creates a new PhotonVisonSubsystem. */
  public PhotonVisonSubsystem() {}

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
    if (hasTarget) {
        this.result = result;
    // This method will be called once per scheduler run
  }
}
}