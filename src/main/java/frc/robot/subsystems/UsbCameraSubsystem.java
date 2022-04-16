// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UsbCameraSubsystem extends SubsystemBase {
  public UsbCameraSubsystem() {

    
    // Creates UsbCamera and MjpegServer [1] and connects to it.
    UsbCamera camera = CameraServer.startAutomaticCapture();//"Intake", 0);
    CameraServer.getServer().setSource(camera);
    // camera.setResolution(320, 240);
    // Creates the CvSink and connects it to the UsbCamera
    // CameraServer.getVideo();
    // // Creates the CvSource and MjpegServer [2] and connects them
    // CameraServer.putVideo("Intake", 640, 480);
  }
}
