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
import edu.wpi.first.math.geometry.Rotation2d;

public class MovingShootingCommand extends CommandBase {

  private final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer; 
  private final HopperSubsystem hopper;
  private final VisionSubsystem vision;
  private final VisionCalculator visionCalculator;
  public double veloXMetersSecond;
  public double veloYMetersSecond;

  public MovingShootingCommand(
    double vxMetersPerSecond, 
    double vyMetersPerSecond, 
    ShooterSubsystem shooter,
    IndexerSubsystem indexer,
    HopperSubsystem hopper,
    VisionSubsystem vision,
    VisionCalculator visionCalculator
    ) {
    this.veloXMetersSecond = vxMetersPerSecond;
    this.veloYMetersSecond = vyMetersPerSecond;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.vision = vision;
    this.visionCalculator = visionCalculator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
