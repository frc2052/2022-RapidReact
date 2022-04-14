// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
import frc.robot.util.vision.ShooterDistanceConfig;
import frc.robot.util.vision.VisionCalculator;

public class ShootCommand extends ShooterIndexingCommand {
  private final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer;
  private final HopperSubsystem hopper;
  private final VisionSubsystem vision;
  // Determines whether all the balls should be shot and both feeder and preloader should be run (true)
  // or if only one ball should be shot and only the feeder should be run (false).
  // This is useful if you pick up the wrong color ball.
  private final BooleanSupplier isLinedUSupplier;
  private final VisionCalculator visionCalculator;

  private double distanceInches;
  private ShooterDistanceConfig shooterConfig;
  // private boolean lastSawStaged;

  public ShootCommand(
    ShootMode shootMode, 
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    HopperSubsystem hopper, 
    VisionSubsystem vision,
    BooleanSupplier isLinedUSupplier,
    DrivetrainSubsystem drivetrain
  ) {
    super (
      shootMode,
      indexer,
      hopper,
      shooter
    );


    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.vision = vision;
    this.isLinedUSupplier = isLinedUSupplier;

    visionCalculator = VisionCalculator.getInstance();
    shooterConfig = new ShooterDistanceConfig(0, 0, 0); // Initial speed (0) for shooter that'll make it just not run if no vision target.
    // Doesn't fix case where limelight breaks and leaves hasValidTarget true, to which the Limelight Is Dead Smart Dashboard button will need to be pressed.

    addRequirements(this.shooter);
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
      vision::getIsLinedUp,//TargetingSubsystem.getInstance()::getIsLinedUpToShoot,
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

    // boolean stagedCargoDetected = indexer.getCargoStagedDetected();

    if (vision.getHasValidTarget()) {
      distanceInches = visionCalculator.getDistanceInches(vision.getTy()); // + drivetrain.getIntendedXVelocityMPS() * 0.1 + 0.1; // TEMP VALUES

      // Logic for switching the shoot angle depending on the ty of the target plus or minus a threshold (so we don't have any rapid movement) and if it's already at the right angle or not.
      if (distanceInches < Constants.Shooter.ANGLE_CHANGE_THRESHOLD_DISTANCE_INCHES - Constants.Shooter.ANGLE_CHANGE_TOLERANCE_DISTANCE_INCHES && shooter.getShootAngleEnum() == FiringAngle.ANGLE_2) {
        shooter.setShootAngle1();
      } else if (distanceInches > Constants.Shooter.ANGLE_CHANGE_THRESHOLD_DISTANCE_INCHES + Constants.Shooter.ANGLE_CHANGE_TOLERANCE_DISTANCE_INCHES && shooter.getShootAngleEnum() == FiringAngle.ANGLE_1) {
        shooter.setShootAngle2();
      }

      shooterConfig = visionCalculator.getShooterConfig((int) distanceInches, shooter.getShootAngleEnum());
    }

    // SmartDashboard.putNumber("Distance Inches", distanceInches);

    if (shooterConfig.getDistanceInches() > 0) { // Makes sure the wheels are running so that we're not going to jam it by pushing a ball into a wheel that isn't running
      shooter.runAtSpeed(
        shooterConfig.getTopMotorVelocityTicksPerSecond(), // Boosts or lowers the shooter velocity by a max of 110% and min of 90%
        shooterConfig.getBottomMotorVelocityTicksPerSecond()
      );

      super.execute();

      // if (!stagedCargoDetected && lastSawStaged) {
      //     System.err.println("Shot with: " + "Target Top: " + shooter.getTargetTopWheelVelocity() + " Target Bottom: " + shooter.getTargetBottomWheelVelocity() 
      //     + " Top Velocity: " + shooter.getTopWheelVelocity() + " Bottom Velocity: " + shooter.getBottomWheelVelocity());
      // }
      // lastSawStaged = stagedCargoDetected;
    } else {
        shooter.stop();
    }
  }

  @Override
  public boolean isReadyToShoot() {
      return shooter.isAtSpeed() && vision.getIsLinedUp();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter, feeder, and preloader, hopper, and turns off Limelight LEDs.
    super.end(interrupted);
    shooter.stop();
    vision.setLEDMode(LEDMode.OFF);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
