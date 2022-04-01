// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in auto to raise the intake.
 */
public class IntakeArmOutCommand extends CommandBase {
    private final IntakeSubsystem intake;
    
    public IntakeArmOutCommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.armIn();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
