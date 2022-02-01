// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.TwoWheelFly;

public class ShooterCommand extends CommandBase {

  private final boolean wantShoot = false;

  public ShooterCommand() {
  }

  public static void executeShooter() {
    if (linedUp == true) { 
    double limelight.distance;
    x = Calculator.findAngle(); 
    TwoWheelFly.runAtShootSpeed(x); 
    wheelOne = Calculator.findTopSpeed(x); 
    wheelTwo = Calculator.findBottomSpeed(x); 
    Shooter.setSpeed(wheelOne, wheelTwo); 
  } 

    if (Shooter.Angle == x && Shooter.Speeds == x) { 
      Indexer.forward(); 
    } 
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
