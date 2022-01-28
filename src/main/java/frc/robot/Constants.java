// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
    
    public static final class Field {
        public static final double kUpperHubHeight = 2.64;   // In meters
    }

    public static final class Limelight {
        public static final double kMountHeight = 0.9525;
        public static final double kMountAngle = 28;

        public static final double kDistanceCalcOffset = 0;
    }
  
    public static final class Calculations {
        public static final double kParabolicPeakOffsetX = -0.5;
        public static final double kParabolicPeakOffsetY = 0.5;
        public static final double kGravitationalConstant = 9.8;
    }

    
    public static final class CopiedFromScorpion {

        public static final class Motors {

            public static final int kDriveRightMasterId = 1;
            public static final int kDriveRightFollower1Id = 2;
            public static final int kDriveRightFollower2Id = 3;
            public static final int kDriveLeftMasterId = 4;
            public static final int kDriveLeftFollower1Id = 5;
            public static final int kDriveLeftFollower2Id = 6;
            public static final int kOuterIntakeMotorID = 7;
            public static final int kClimberMotorID = 8;
            public static final int kShooterMasterMotorID = 9;
            public static final int kShooterFollowerMotorID = 10;
            public static final int kHoodMotorID = 11;
            public static final int kTurretMotorID = 12;
            public static final int kConveyorMotorBottemLeftID = 13;   
            public static final int kConveyorMotorBottemRightID = 14;   
            public static final int kLifterMotorID = 15;   
            public static final int kActiveBalanceMotorID = 16;
    
            public static final int kFalconShooterMotorId = 42;
        }
    
        public static final class Turret{
                public static final double kTurnLeftSpeed = -0.1;
                public static final double kTurnRightSpeed = 0.1;
    
                public static final int kTurretMinEncoderPos = -4000;
                public static final int kTurretMaxEncoderPos = 16000;
    
                public static final double kTicksPerDegree = 4096 / 90;
                public static final double kMaxAngle = 90;
                public static final double kMinAngle = -90;
    
                public static final double kMinTurretSpeed = .07;
                public static final double kMaxTurretSpeed = .75;
        }

        public static final class DriveTrain{
            public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(.6); //24" is .6 meters

            // See: http://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/characterization-routine.html
            public static final double ksVolts = 1.06;
            public static final double kvVoltSecondsPerMeter = 4.76;
            public static final double kaVoltSecondsSquaredPerMeter = 0.568;
            
            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;

            public static final int kVelocityControlSlot = 0;
            public static final int kCANBusConfigTimeoutMS = 10;
            public static final int kTicksPerRot = 1024;
            public static final double kEncoderGearRatio = (1.0/3)*(20.0/64);
            public static final double kDriveWheelCircumferenceMeters = Units.inchesToMeters(6.0) * Math.PI;

            public static final int kPDriveVel = 8;

            public static final double kTurnInPlaceSpeed = .75;
        }

        public static final class Solenoids {
            public static final int kShifterSolenoidID = 4;
            public static final int kUpIntakeSolenoidID = 0;
            public static final int kDownIntakeSolenoidID = 1;
            public static final int kElevatorLockSolenoidID = 2;
            public static final int kElevatorUnLockSolenoidID = 3;
        }
    }
}
