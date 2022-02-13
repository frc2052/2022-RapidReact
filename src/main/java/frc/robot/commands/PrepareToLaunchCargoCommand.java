// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TwoWheelFlySubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class PrepareToLaunchCargoCommand extends CommandBase {
  private final TwoWheelFlySubsystem m_twoWheelFly; 
  private final IndexerSubsystem m_indexer;
  private final VisionSubsystem m_vision;
  private final IntakeSubsystem m_intake;
  private final HopperSubsystem m_grassHopper;


  public PrepareToLaunchCargoCommand(TwoWheelFlySubsystem twoWheelFly, IndexerSubsystem indexer, VisionSubsystem vision, IntakeSubsystem intake, HopperSubsystem grassHopper) {
    this.m_twoWheelFly = twoWheelFly;
    this.m_indexer = indexer;
    this.m_vision = vision;
    this.m_intake = intake;
    this.m_grassHopper = grassHopper;
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
    if (m_indexer.getCargoPreStagedDetected() == false && m_indexer.getCargoStagedDetected() == false) {
      m_indexer.runPreload();
      m_grassHopper.hopperGo();
    } else if (m_indexer.getCargoPreStagedDetected() == false && m_indexer.getCargoStagedDetected() == true) {
        m_indexer.stopFeeder();
        m_indexer.runPreload();
    } else {
        m_indexer.stopFeeder();
        m_indexer.stopPreload();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopFeeder();
    m_indexer.stopPreload();
    m_twoWheelFly.stop();
    m_grassHopper.hopperStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
