// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

/**
 * Orchestra class passed into subsystems to register their instruments.
 * NOTE: This class will not work with the drivetrain :(
 */
public class OrchestraUtility {
    private final Orchestra orchestra;
    private Songs currentSong;

    private static OrchestraUtility INSTANCE;

    private OrchestraUtility() {
        orchestra = new Orchestra();
    }

    public void addInstrument(TalonFX instrument) {
        orchestra.addInstrument(instrument);
    }

    public void setSong(Songs song) {
        this.currentSong = song;
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

    public OrchestraUtility getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new OrchestraUtility();
        }
        return INSTANCE;
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
