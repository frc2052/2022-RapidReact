// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.util.vision.ShooterDistanceConfig;

public class NonVisionShootCommand extends ShooterIndexingCommand {
  private final ShooterSubsystem shooter;

  // Determines whether all the balls should be shot and both feeder and preloader should be run (true)
  // or if only one ball should be shot and only the feeder should be run (false).
  // This is useful if you pick up the wrong color ball.
  private FiringAngle firingAngleMode;
  private double topWheelVelocity;
  private double bottomWheelVelocity;

  public NonVisionShootCommand(ShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, FiringAngle firingAngleMode, double topWheelVelocity, double bottomWheelVelocity) {
    super(
      shootMode, 
      indexer, 
      hopper, 
      shooter
    );

    this.shooter = shooter;
    this.firingAngleMode = firingAngleMode;
    this.topWheelVelocity = topWheelVelocity;
    this.bottomWheelVelocity = bottomWheelVelocity;

    super.isLinedUp = true;

    addRequirements(shooter, indexer);
  }

  public NonVisionShootCommand(ShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, FiringAngle firingAngleMode, ShooterDistanceConfig shooterDistanceConfig) {
    this(shootMode, shooter, indexer, hopper, firingAngleMode, shooterDistanceConfig.getTopMotorVelocityTicksPerSecond(), shooterDistanceConfig.getBottomMotorVelocityTicksPerSecond());
  }

  public NonVisionShootCommand(ShootMode shootMode, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsystem hopper, double topWheelVelocity, double bottomWheelVelocity) { // TODO Probably remove this constructor and add firing angle specification to all other NonVisionShootCommands, was only added to not break the functionality of already existing ones.
    this(shootMode, shooter, indexer, hopper, FiringAngle.ANGLE_1, topWheelVelocity, bottomWheelVelocity);
  }

  @Override
  public void initialize() {
      if (firingAngleMode == null) {
        // Do nothing, keep same firing angle already set somewhere else.
      } else if (firingAngleMode == FiringAngle.ANGLE_1) {
        shooter.setShootAngle1();
      } else {
        shooter.setShootAngle2();
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.shootAtSpeed(topWheelVelocity, bottomWheelVelocity);
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.stop();
  }

  public void overrideDelay() {
    super.delayOverride = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
