// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

/** Add your docs here. */
public class MK3StandardSwerveModule {
    private final String debugName;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANCoder encoder;

    private static final int CAN_TIMEOUT_MS = 250;

    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    public MK3StandardSwerveModule(
        String debugName,
        int driveMotorChannel,
        int steerMotorChannel,
        int encoderChannel,
        Rotation2d steerOffsetRadians
    ) {
        this.debugName = debugName;
        
        driveMotor = createDriveMotor(driveMotorChannel);
        steerMotor = createSteerMotor(steerMotorChannel);
        encoder = createCANCoder(encoderChannel, steerOffsetRadians);
    }

    private TalonFX createDriveMotor(int driveMotorChannel) {
        double sensorPositionCoefficient = Math.PI * 
            Constants.SwerveModule.WHEEL_DIAMETER_METERS * 
            Constants.SwerveModule.DRIVE_REDUCTION / 
            Constants.SwerveModule.TICKS_PER_ROTATION;
        double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        motorConfiguration.voltageCompSaturation = Constants.SwerveModule.MAX_VOLTAGE;
        motorConfiguration.supplyCurrLimit.currentLimit = driveCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;

        TalonFX returnDriveMotor = new TalonFX(driveMotorChannel);
        CtreUtils.checkCtreError(
            returnDriveMotor.configAllSettings(motorConfiguration),
            "Failed to configure drive motor on module: " + debugName
        );

        returnDriveMotor.enableVoltageCompensation(true);

        returnDriveMotor.setNeutralMode(NeutralMode.Brake);

        returnDriveMotor.setInverted(true);
        returnDriveMotor.setSensorPhase(true);

        // Reduce CAN status frame rates
        CtreUtils.checkCtreError(
            returnDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250, CAN_TIMEOUT_MS),
            "Failed to configure Falcon status frame period"
        );

        return returnDriveMotor;
    }
    
    private TalonFX createSteerMotor(int steerMotorChannel) {
        double sensorPositionCoefficient = 2.0 * Math.PI / 
            Constants.SwerveModule.TICKS_PER_ROTATION * 
            Constants.SwerveModule.STEER_REDUCTION;
        double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.slot0.kP = 0.2;
        motorConfiguration.slot0.kI = 0.0;
        motorConfiguration.slot0.kD = 0.1;
        
        motorConfiguration.voltageCompSaturation = Constants.SwerveModule.MAX_VOLTAGE;

        motorConfiguration.supplyCurrLimit.currentLimit = steerCurrentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;

        TalonFX returnSteerMotor = new TalonFX(steerMotorChannel);
        CtreUtils.checkCtreError(
            returnSteerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS), 
            "Failed to configure Falcon 500 settings"
        );

        returnSteerMotor.enableVoltageCompensation(true);
        CtreUtils.checkCtreError(
            returnSteerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS), 
            "Failed to set Falcon 500 feedback sensor"
        );
        returnSteerMotor.setSensorPhase(true);
        returnSteerMotor.setInverted(true);
        returnSteerMotor.setNeutralMode(NeutralMode.Brake);

        CtreUtils.checkCtreError(
            returnSteerMotor.setSelectedSensorPosition(
                absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient, 0, CAN_TIMEOUT_MS
            ),
            "Failed to set Falcon 500 encoder position"
        );

        // Reduce CAN status frame rates
        CtreUtils.checkCtreError(
                returnSteerMotor.setStatusFramePeriod(
                    StatusFrameEnhanced.Status_1_General,
                    250,
                    CAN_TIMEOUT_MS
                ),
                "Failed to configure Falcon status frame period"
        );

        return returnSteerMotor;
    }

    private CANCoder createCANCoder(int encoderChannel, Rotation2d steerOffset) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = steerOffset.getDegrees();
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        
        CANCoder returnEncoder = new CANCoder(encoderChannel);

        CtreUtils.checkCtreError(
            returnEncoder.configAllSettings(config, CAN_TIMEOUT_MS), 
            "Failed to configure CANCoder on module: " + debugName
        );
        // Reduce CAN status frame rates
        CtreUtils.checkCtreError(
            returnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, CAN_TIMEOUT_MS), 
            "Failed to configure CANCoder update rate!"
        );

        return returnEncoder;
    }

    public void setState(SwerveModuleState state) {
        double driveVoltage = state.speedMetersPerSecond / 
            Constants.SwerveModule.MAX_VELOCITY_METERS_PER_SECOND * 
            Constants.SwerveModule.MAX_VOLTAGE;
        double steerAngleRadians = state.angle.getRadians();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }
}
