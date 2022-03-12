// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.vision.VisionCalculator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;

public class MovingShootingCommand extends CommandBase {

  private final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer; 
  private final HopperSubsystem hopper;
  private final VisionSubsystem vision;
  private final VisionCalculator visionCalculator;
  protected final ChassisSpeeds chassisSpeeds;
  public double veloXMetersSecond;
  public double veloYMetersSecond;

  private Timer timer;

  public MovingShootingCommand(
    double vxMetersPerSecond, 
    double vyMetersPerSecond, 
    ShooterSubsystem shooter,
    IndexerSubsystem indexer,
    HopperSubsystem hopper,
    VisionSubsystem vision,
    VisionCalculator visionCalculator,
    ChassisSpeeds chassisSpeeds
    ) {
    this.veloXMetersSecond = vxMetersPerSecond;
    this.veloYMetersSecond = vyMetersPerSecond;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.vision = vision;
    this.visionCalculator = visionCalculator;
    this.chassisSpeeds = chassisSpeeds;
  }

  @Override
  public boolean isFinished() {
    if(!super.indexer.getCargoStagedDetected() && !super.indexer.getCargoPreStagedDetected()) {
      if (timer == null) {
        timer = new Timer(); 
        timer.start();
      }
      if (timer.get() >= 1) {
        return true;
      }
    } else { 
        clearTimer();
    }
    return false;
  }

  private void clearTimer() {
    if(timer != null) {
        timer.stop();
        timer = null;
    }
  }
}
