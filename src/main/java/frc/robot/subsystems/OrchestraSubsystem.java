// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {
  private final Orchestra orchestra;
  private Songs currentSong;
  /** Creates a new OrchestraSubsystem. */
  public OrchestraSubsystem() {
    orchestra = new Orchestra();
    currentSong = Songs.DEFINITELY_NOT_A_RICK_ROLL;
  }

  public void addInstrument(TalonFX instrument) {
    orchestra.addInstrument(instrument);
  }

  public void playSong() {
    orchestra.loadMusic(currentSong.getFileName());
    if (orchestra.play() != ErrorCode.OK) {
      System.err.println("The orchestra didn't feel like playing today! :(");
    }
  }

  public void stopSong() {
    if (orchestra.stop() != ErrorCode.OK) {
      System.err.println("The orchestra has an encore!");
    }
  }

  public void setSong(Songs song) {
    this.currentSong = song;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static enum Songs {
    DEFINITELY_NOT_A_RICK_ROLL("RickRoll");

    private final String fileName;

    private Songs(String fileName) {
      this.fileName = fileName;
    }

    public String getFileName() {
      return fileName;
    }
  }
}
