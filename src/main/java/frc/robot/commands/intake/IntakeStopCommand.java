// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final HopperSubsystem hopper;

    public IntakeStopCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
      this.intake = intake;
      this.hopper = hopper;

      addRequirements(this.intake, this.hopper);
    }
  
    @Override
    public void initialize() {
        // Stops the intake and hopper.
        intake.stop();
        hopper.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
