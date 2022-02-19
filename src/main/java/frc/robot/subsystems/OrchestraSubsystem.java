// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {
  private final Orchestra orchestra;
  private SONGS song;

  /** Creates a new OrchestraSubsystem. */
  public OrchestraSubsystem() {
    orchestra = new Orchestra();
    song = SONGS.DEFINITELY_NOT_A_RICK_ROLL;
  }

  public void addInstrument(TalonFX instrument) {
    orchestra.addInstrument(instrument);
  }

  public void playSong() {
    
  }

  public void setSong(SONGS song) {
    this.song = song;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static enum SONGS {
    DEFINITELY_NOT_A_RICK_ROLL
  }
}
