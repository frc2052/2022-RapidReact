// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TwoWheelFly;

public class WheelVelocity extends CommandBase {
  
  private TwoWheelFly twoWheelFly;

  public WheelVelocity(TwoWheelFly twoWheelFly) { 
    this.twoWheelFly = twoWheelFly;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    twoWheelFly.runAtShootSpeed(Constants.ShooterSub.TOPWHEELSPEED, Constants.ShooterSub.BOTTOMWHEELSPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
