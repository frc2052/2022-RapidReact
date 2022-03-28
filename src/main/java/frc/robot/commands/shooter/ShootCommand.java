// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.ShooterDistanceConfig;
import frc.robot.util.vision.VisionCalculator;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer;
  private final HopperSubsystem hopper;
  private final VisionSubsystem vision;
  private final DrivetrainSubsystem drivetrain;
  // Determines whether all the balls should be shot and both feeder and preloader should be run (true)
  // or if only one ball should be shot and only the feeder should be run (false).
  // This is useful if you pick up the wrong color ball.
  private final ShootMode shootMode;
  private final BooleanSupplier isLinedUSupplier;

  private final VisionCalculator visionCalculator;

  public ShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision,
    BooleanSupplier isLinedUSupplier,
    DrivetrainSubsystem drivetrain
  ) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.vision = vision;
    this.shootMode = shootMode;
    this.isLinedUSupplier = isLinedUSupplier;
    this.drivetrain = drivetrain;

    visionCalculator = VisionCalculator.getInstance();

    addRequirements(this.shooter, this.indexer, this.hopper);
  }

  public ShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision,
    DrivetrainSubsystem drivetrain
  ) {
    this(
      shootMode, 
      shooter, 
      indexer, 
      hopper, 
      vision, 
      TargetingSubsystem.getInstance()::getIsLinedUpToShoot,
      drivetrain
    );
  }

  @Override
  public void initialize() {
      vision.setLEDMode(LEDMode.ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceInches = visionCalculator.getDistanceInches(vision.getTy()); // + drivetrain.getIntendedXVelocityMPS() * 0.1 + 0.1; // TEMP VALUES

    if(vision.getHasValidTarget()) {
        // Logic for switching the shoot angle depending on the ty of the target plus or minus a threshold (so we don't have any rapid movement) and if it's already at the right angle or not.
        if(vision.getTy() < Constants.Shooter.ANGLE_CHANGE_THRESHOLD_TY - Constants.Shooter.ANGLE_CHANGE_TOLERANCE_DEGREES && shooter.getShootAngleEnum() == FiringAngle.ANGLE_1) {
        shooter.setShootAngle2();
        } else if (vision.getTy() > Constants.Shooter.ANGLE_CHANGE_THRESHOLD_TY + Constants.Shooter.ANGLE_CHANGE_TOLERANCE_DEGREES && shooter.getShootAngleEnum() == FiringAngle.ANGLE_2) {
        shooter.setShootAngle1();
        }
    }

    ShooterDistanceConfig shooterConfig = visionCalculator.getShooterConfig((int) distanceInches, shooter.getShootAngleEnum());

    SmartDashboard.putNumber("Distance Inches", distanceInches);

    shooter.shootAtSpeed(
      shooterConfig.getTopMotorVelocityTicksPerSecond(), // Boosts or lowers the shooter velocity by a max of 110% and min of 90%
      shooterConfig.getBottomMotorVelocityTicksPerSecond()
    );

    if (shooter.isAtSpeed() && isLinedUSupplier.getAsBoolean()) {
      indexer.runFeeder();
      hopper.run();
      if (shootMode == ShootMode.SHOOT_ALL) {
        indexer.runPreload();
      }
    } else {
      hopper.stop();
      indexer.stopFeeder();
      indexer.stopPreload();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter, feeder, and preloader.
    shooter.stop();
    hopper.stop();
    indexer.stopFeeder();
    indexer.stopPreload();
    vision.setLEDMode(LEDMode.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum ShootMode {
    SHOOT_SINGLE,
    SHOOT_ALL
  }
}
