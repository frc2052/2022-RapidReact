// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperTest extends CommandBase {
    private HopperSubsystem hopper;

  /** Creates a new HopperTest. */
  public HopperTest(HopperSubsystem hopper) {
      this.hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      hopper.run();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
