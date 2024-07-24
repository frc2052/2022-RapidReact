package com.team2052.swervemodule;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule{
    protected static final int CAN_TIMEOUT_MS = 250;

    protected final String debugName;

    private final TalonFX angleMotor;
    private final TalonFX driveMotor;

    private final double anglePosConversionFactor;
    private final double drivePosConversionFactor;
    private final double driveVelocityConversionFactor;
    private final double maxVelocityMetersPerSecond;

    protected final CANCoder canCoder;

    public SwerveModule(String debugName, int driveMotorChannel, int angleMotorChannel, int cancoderChannel, Rotation2d angleMotorOffsetRadians) {

        //Initialize CANcoder
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.magnetOffsetDegrees = -angleMotorOffsetRadians.getDegrees();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        canCoder = new CANCoder(cancoderChannel);
        checkError(
            "Failed to configure CANCoder",
            canCoder.configAllSettings(
                canCoderConfiguration,
                CAN_TIMEOUT_MS
            )
        );

        checkError(
            "Failed to set CANCoder status frame period",
            canCoder.setStatusFramePeriod(
                CANCoderStatusFrame.SensorData,
                10,
                CAN_TIMEOUT_MS
            )
        );

        /*
         * Drive motor
         */
        driveMotor = new TalonFX(driveMotorChannel);

        checkError("Failed to restore drive motor factory defaults", driveMotor.configFactoryDefault());
        
        driveMotor.setSensorPhase(true);
        driveMotor.setInverted(SwerveConstants.SwerveModule.DRIVE_INVERTED);
        driveMotorNeutralModeBrake();


        drivePosConversionFactor = (Math.PI *  SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS /
        SwerveConstants.SwerveModule.TICKS_PER_ROTATION) * SwerveConstants.SwerveModule.DRIVE_REDUCTION;

        driveVelocityConversionFactor = drivePosConversionFactor / 60.0;

        checkError(
            "Failed to set drive motor status frame period",
            driveMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                250,
                CAN_TIMEOUT_MS
            )
        );

        /*
         * Angle motor
         */

        angleMotor = new TalonFX(angleMotorChannel);
        angleMotor.setInverted(SwerveConstants.SwerveModule.ANGLE_INVERTED);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        //PID configuration for the angle motor
        TalonFXConfiguration angleMotorConfiguration = new TalonFXConfiguration();
        angleMotorConfiguration.slot0.kP = SwerveConstants.SwerveModule.ANGLE_MOTOR_P;
        angleMotorConfiguration.slot0.kI = SwerveConstants.SwerveModule.ANGLE_MOTOR_I;
        angleMotorConfiguration.slot0.kD = SwerveConstants.SwerveModule.ANGLE_MOTOR_D;
        
        anglePosConversionFactor = (2.0 * Math.PI / SwerveConstants.SwerveModule.TICKS_PER_ROTATION) * 
        SwerveConstants.SwerveModule.ANGLE_REDUCTION;

        checkError("Failed to restore steer motor factory defaults", angleMotor.configFactoryDefault());
        checkError("Failed to configure steer motor", angleMotor.configAllSettings(angleMotorConfiguration));

        checkError(
            "Failed to set steer motor feedback sensor",
            angleMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                0,
                CAN_TIMEOUT_MS
            )
        );

        System.out.println(canCoder.getAbsolutePosition());

        checkError(
            "Failed to set steer motor encoder position",
            angleMotor.setSelectedSensorPosition(
                Math.toRadians(canCoder.getAbsolutePosition()) / anglePosConversionFactor,
                0,
                CAN_TIMEOUT_MS
            )
        );

        checkError(
            "Failed to set steer motor status frame period",
            angleMotor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                250,
                CAN_TIMEOUT_MS
            )
        );

        maxVelocityMetersPerSecond = SwerveConstants.SwerveModule.FALCON500_ROUNDS_PER_MINUTE / 60
         * SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS * Math.PI * SwerveConstants.SwerveModule.DRIVE_REDUCTION;
         
         this.debugName = debugName;
    }

    public void driveMotorNeutralModeBrake(){
        driveMotor.setNeutralMode(NeutralMode.Brake);
        System.out.println("Drive Motor set to BRAKE");
    }

    public void driveMotorNeutralModeCoast(){
        driveMotor.setNeutralMode(NeutralMode.Coast);
        System.out.println("Drive Motor set to COAST");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            // Convert from ticks to meters per second using the predefined conversion factor
            driveMotor.getSelectedSensorVelocity() * driveVelocityConversionFactor,
            new Rotation2d(
                // Convert from ticks to radians using the predefined conversion factor
                angleMotor.getSelectedSensorPosition() * anglePosConversionFactor
            )
        );
    }

    public void setState(double velocityMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getState().angle
        );

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(
            TalonFXControlMode.PercentOutput,
            desiredState.speedMetersPerSecond / maxVelocityMetersPerSecond
        );

        angleMotor.set(
            TalonFXControlMode.Position,
            desiredState.angle.getRadians() / anglePosConversionFactor
        );
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getSelectedSensorPosition() * drivePosConversionFactor,
            new Rotation2d(
                angleMotor.getSelectedSensorPosition() * anglePosConversionFactor
            )
        );
    }

    public CANCoder getCancoder() {
        return canCoder;
    }


    public static double getMaxVelocityMetersPerSecond() {
        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        return SwerveConstants.SwerveModule.FALCON500_ROUNDS_PER_MINUTE / 60 * SwerveConstants.SwerveModule.DRIVE_REDUCTION * 
            SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS * Math.PI;
    }

    public static double getMaxAngularVelocityRadiansPerSecond() {
        /*
         * Find the theoretical maximum angular velocity of the robot in radians per second 
         * (a measure of how fast the robot can rotate in place).
         */
        return getMaxVelocityMetersPerSecond() / Math.hypot(
            Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
            Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        );
    }

    public void debug(String debugName) {
        SmartDashboard.putNumber(debugName + " Offset Degrees", canCoder.getAbsolutePosition());
        SmartDashboard.putNumber(debugName + " Degrees", Math.toDegrees(angleMotor.getSelectedSensorPosition() * anglePosConversionFactor));
    }

    @SuppressWarnings("unchecked")
    protected <E> void checkError(String message, E... errors) {
        for (E error : errors) {
            if (error != REVLibError.kOk && error != ErrorCode.OK) {
                DriverStation.reportError(
                    message + " on [" + debugName + "] module: " + error.toString(),
                    false
                );
            }
        }
    }
}

