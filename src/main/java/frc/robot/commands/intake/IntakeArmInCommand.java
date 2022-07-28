// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmInCommand extends CommandBase {
    private final IntakeSubsystem intake;
    
    /**
     * Command to simply raise the intake.
     * @param intake
     */
    public IntakeArmInCommand(IntakeSubsystem intake) {
        this.intake = intake;

        //addRequirements(this.intake); // Requirement commented out because unlikely this can conflict with anything and more likely the requirement itself would cause conflicts.
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
