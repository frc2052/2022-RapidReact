// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command used in auto to raise the intake.
 */
public class IntakeArmInCommand extends HopperBaseCommand {
    private final IntakeSubsystem intake;
    
    public IntakeArmInCommand(IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper) {
        super(indexer, hopper);
        this.intake = intake;

        addRequirements(this.intake);
    }

    // Overriden to stop the hopper from excecuting before the command is ended.
    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Retracts intake arm and stops the intake wheels.
        intake.armIn();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
