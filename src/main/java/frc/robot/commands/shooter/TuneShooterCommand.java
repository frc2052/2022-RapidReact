// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TuneShooterCommand extends ShooterIndexingCommand {
  private ShooterSubsystem shooter;

  public TuneShooterCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, HopperSubsystem hopper) {
    super(
      ShootMode.SHOOT_ALL, 
      indexer, 
      hopper, 
      shooter
    );
    
    this.shooter = shooter;

    SmartDashboard.putNumber("Top shooter wheel speed TP100MS", shooter.getTargetTopWheelVelocity());
    SmartDashboard.putNumber("Bottom shooter wheel speed TP100MS", shooter.getTargetBottomWheelVelocity());
    SmartDashboard.putBoolean("Is Angle 2", false);

    addRequirements(shooter, indexer, intake, hopper);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topSpeedTP100MS = SmartDashboard.getNumber("Top shooter wheel speed TP100MS", shooter.getTargetTopWheelVelocity());
    double bottomSpeedTP100MS = SmartDashboard.getNumber("Bottom shooter wheel speed TP100MS", shooter.getTargetBottomWheelVelocity());

    if (SmartDashboard.getBoolean("Is Angle 2", false)) {
        shooter.setShootAngle2();
    } else {
        shooter.setShootAngle1();
    } 
    shooter.runAtSpeed(topSpeedTP100MS, bottomSpeedTP100MS);

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
