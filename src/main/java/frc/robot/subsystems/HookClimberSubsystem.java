package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class HookClimberSubsystem extends SubsystemBase{
    // DoubleSolenoid that controls both in and out solenoids for both climbing arms.
    private final DoubleSolenoid climberSolenoid;
    private ClimberSolenoidState currentSolenoidState;
    // Motor that controls the wenches for climbing.
    private final TalonFX climberMotor;
    private double desiredPositionTicks;

    private final DoubleSolenoid lockSolenoid;
    private boolean isLocked;

    private boolean isVertical;

    public HookClimberSubsystem() {
        climberSolenoid = new DoubleSolenoid(
            Constants.Solenoids.COMPRESSOR_MODULE_ID,
            PneumaticsModuleType.REVPH, 
            Constants.Solenoids.CLIMBER_FORWARD_SOLENOID,
            Constants.Solenoids.CLIMBER_BACKWARD_SOLENOID
        );
        currentSolenoidState = ClimberSolenoidState.BACKWARD;

        climberMotor = new TalonFX(Constants.MotorIDs.CLIMBER_MOTOR);
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.setInverted(true);
        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        climberMotor.setSelectedSensorPosition(0, 0, 10);

        lockSolenoid = new DoubleSolenoid(
            Constants.Solenoids.COMPRESSOR_MODULE_ID,
            PneumaticsModuleType.REVPH, 
            Constants.Solenoids.CLIMBER_LOCK_SOLENOID,
            Constants.Solenoids.CLIMBER_UNLOCK_SOLENOID
        );
        isLocked = lockSolenoid.get() == Value.kReverse;
        isVertical = true;
        unlock();
    }

    /**
     * Extends the climbing arm at a set speed
     */
    public void extend(double extendPctOutput, boolean override) {
        if (override) {
            climberMotor.set(ControlMode.PercentOutput, extendPctOutput * 0.5);
        } else if (getIsAtMaxHeight()) {
            climberMotor.set(ControlMode.PercentOutput, extendPctOutput);
        } else {
            System.err.println("Climber extended to (or past) the max height!");
            climberMotor.set(ControlMode.PercentOutput, 0);
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBER_MAX_EXTENSION);
        }
    }

    public void extend(boolean override) {
        extend(Constants.Climber.CLIMBER_EXTENSION_SPEED_PCT, override);
    }

    /**
     * Retracts the climbing arm at a set speed
     */
    public void retract(double retractPctOutput, boolean override) {
        if (override) {
            climberMotor.set(ControlMode.PercentOutput, retractPctOutput * .75); //slower climb speed when doing override
        } else if (getIsAtMinHeight()) {
            climberMotor.set(ControlMode.PercentOutput, retractPctOutput);
        } else {
            System.err.println("Climber retracted to (or past) the min height!");
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void retract(boolean override) {
        retract(Constants.Climber.CLIMBER_RETRACT_SPEED_PCT, override);
    }

    /**
     * Stops all climber motor activity.
     */
    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Shifts climber arms into climbing ready position (verticle)
     */
    public void shiftForward(){
        climberSolenoid.set(Value.kForward);
        currentSolenoidState = ClimberSolenoidState.FORWARD;
        isVertical = true;
    }

    /**
     * Shifts climber arms into the relaxed or reaching position (angled)
     */
    public void shiftBackward(){
        climberSolenoid.set(Value.kReverse);
        currentSolenoidState = ClimberSolenoidState.BACKWARD;
        isVertical = false;
    }

    public ClimberSolenoidState getClimberSolenoidState() {
        return currentSolenoidState;
    }

    public void lock() {
        System.err.println("************************ CLIMBER LOCKED");
        lockSolenoid.set(Value.kReverse);
        isLocked = true;
        LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CLIMBING_LOCK_ENGAGED);
    }

    public void unlock() {
        System.err.println("************************ CLIMBER UNLOCKED");
        lockSolenoid.set(Value.kForward);
        isLocked = false;
    }

    public void zeroEncoder() {
        climberMotor.setSelectedSensorPosition(0);
    }

    public boolean getIsLocked() {
        return isLocked;
    }

    public boolean getIsAtMaxHeight() {
        return climberMotor.getSelectedSensorPosition() <= (isVertical ? Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_VERTICAL : Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS_TILTED);
    }

    public boolean getIsAtMinHeight() {
        return climberMotor.getSelectedSensorPosition() >= Constants.Climber.MIN_CLIMBER_HEIGHT_TICKS;
    }

    public double getEncoderPosition() {
        return climberMotor.getSelectedSensorPosition();
    }

    public void setArmPostionInches(double inches){
        double ticks = heightInchesToTicks(inches);
        climberMotor.set(ControlMode.Position, ticks);
        desiredPositionTicks = ticks;
    }

    public boolean isAtDesiredPosition(){
        return climberMotor.getSelectedSensorPosition() == desiredPositionTicks;
    }

    private double heightInchesToTicks(double inches) {
        double numberOfRotaions = inches / Constants.Climber.WINCH_CIRCUMFERENCE_INCHES;
        return numberOfRotaions * Constants.Climber.TICKS_PER_WINCH_ROTATION;
    }

@Override
public void periodic() {
    SmartDashboard.putNumber("Climber Height Ticks", climberMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Climber Locked", isLocked);
}

    public static enum ClimberSolenoidState {
        FORWARD,
        BACKWARD;
    }
}
