package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HookClimberSubsystem extends SubsystemBase{
    // DoubleSolenoid that controls both in and out solenoids for both climbing arms.
    private final DoubleSolenoid climberSolenoid;
    // Motor that controls the wenches for climbing.
    private final TalonFX climberMotor;
    private double desiredPositionTicks;

    public HookClimberSubsystem() {
        climberSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            Constants.Solenoids.CLIMBER_FORWARD_SOLENOID,
            Constants.Solenoids.CLIMBER_BACKWARD_SOLENOID
        );

        climberMotor = new TalonFX(Constants.MotorIDs.CLIMBER_MOTOR);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.setInverted(false);
        climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        climberMotor.setSelectedSensorPosition(0, 0, 10);
    }

    /**
     * Extends the climbing arm at a set speed
     */
    public void manualExtendArm() {
        if (climberMotor.getSelectedSensorPosition() <= Constants.Climber.MAX_CLIMBER_HEIGHT_TICKS) {
            climberMotor.set(ControlMode.PercentOutput, Constants.Climber.CLIMBER_EXTENSION_SPEED);
        } else {
            System.out.println("Climber extended to (or past) the max height!");
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * Retracts the climbing arm at a set speed
     */
    public void manualRetractArm() {
        if (climberMotor.getSelectedSensorPosition() >= Constants.Climber.MIN_CLIMBER_HEIGHT_TICKS) {
            climberMotor.set(ControlMode.PercentOutput, -Constants.Climber.CLIMBER_EXTENSION_SPEED);
        } else {
            System.out.println("Climber retracted to (or past) the min height!");
            climberMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * Shifts climber arms into climbing ready position (verticle)
     */
    public void shiftClimberForward(){
        climberSolenoid.set(Value.kForward);
    }

    /**
     * Shifts climber arms into the relaxed or reaching position (angled)
     */
    public void shiftClimberBackward(){
        climberSolenoid.set(Value.kReverse);
    }

    /**
     * Stops all climber activity including solenoids and motors.
     */
    public void manualStop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
        climberSolenoid.set(Value.kOff);
    }

    public void setArmPostionInches(double inches){
        double ticks = heightInInchesToTicks(inches);
        climberMotor.set(ControlMode.Position, ticks);
        desiredPositionTicks = ticks;
    }

    public boolean isAtDesiredPosition(){
        return climberMotor.getSelectedSensorPosition() == desiredPositionTicks;
    }

    private double heightInInchesToTicks(double inches) {
        double numberOfRotaions = inches / Constants.Climber.WINCH_CIRCUMFERENCE_INCHES;
        return numberOfRotaions * Constants.Climber.TICKS_PER_WINCH_ROTATION;
    }
}
