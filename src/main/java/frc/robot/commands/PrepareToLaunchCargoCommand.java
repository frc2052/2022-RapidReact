// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PrepareToLaunchCargoCommand extends CommandBase {
  private final VisionSubsystem m_vision;
  private final IndexerSubsystem m_indexer;
  private final TwoWheelFlySubsystem m_twoWheelFly; 
  private final IntakeSubsystem m_intake;
  private final DigitalInput limitSwitch = new DigitalInput(Constants.LimitSwitch.INDEXER_PRELOAD);
  private final DigitalInput limitSwitchTwo = new DigitalInput(Constants.LimitSwitch.INDEXER_FEEDER);

  public PrepareToLaunchCargoCommand(VisionSubsystem vision, IndexerSubsystem indexer, TwoWheelFlySubsystem twoWheelFly, IntakeSubsystem intake) {
    m_vision = vision;
    m_indexer = indexer;
    m_twoWheelFly = twoWheelFly;
    m_intake = intake;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TESTING Mathmatical calculations for running shooter at required velocity with Limelight's calculated distance.
    if(m_vision.hasValidTarget()) {
      double distance = m_vision.getXDistanceToUpperHub();
      double reqVelocity = m_twoWheelFly.calculateReqProjectileVelocity(distance);
      double reqAngularVelocity = reqVelocity/Constants.ShooterSub.kFlywheelRadiusMeters;
      //double reqRPM = m_twoWheelFly.calculateReqShooterRPM(reqVelocity);

      m_twoWheelFly.setBothWheelVelocities(reqAngularVelocity);
    }

    m_twoWheelFly.runAtShootSpeed();
    if ( limitSwitch.get() == false ) {
      m_indexer.runPreload();
      m_intake.hopperGo();
    } else {
        m_indexer.stop();
      if (limitSwitchTwo.get() == false) {
        m_indexer.runPreload();
        m_intake.hopperGo();
      } else {
        m_indexer.stop();
    }
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
    m_twoWheelFly.stop();
    m_intake.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
