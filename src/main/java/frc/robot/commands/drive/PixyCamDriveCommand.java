// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PixyCamSubsystem;
import frc.robot.subsystems.PixyCamSubsystem.BallColor;

public class PixyCamDriveCommand extends DefaultDriveCommand {
  private final PixyCamSubsystem pixyCamSubsystem;
  private final BallColor ballColor;

  /** Creates a new PixyCamDriveCommand. */
  public PixyCamDriveCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    PixyCamSubsystem pixyCamSubsystem,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DashboardControlsSubsystem dashboard
  ) {
    super(
      drivetrainSubsystem, 
      translationXSupplier, 
      translationYSupplier, 
      () -> { return 0.0; }, //this value will not be used because getTurnWillBeOverriden
      dashboard
    );
    this.pixyCamSubsystem = pixyCamSubsystem;
    
    if (DriverStation.getAlliance() == Alliance.Blue) {
      ballColor = BallColor.BLUE;
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      ballColor = BallColor.RED;
    } else {
      // TODO: Do something else
      ballColor = null;
    }

    super.addRequirements(pixyCamSubsystem);
  }

  @Override
  public double getTurnValue() {
    return pixyCamSubsystem.angleToBall(ballColor).getRadians();
  }

  // Returns true when the angle to the ball is within 5 degrees
  @Override
  public boolean isFinished() {
    return pixyCamSubsystem.angleToBall(ballColor).getDegrees() <= 5.0;
  }
}
