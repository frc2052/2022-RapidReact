package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodOneWheel extends SubsystemBase {
    
    private TalonSRX angleMotor = null;
    private boolean runOpenLoop = true;

    public void HoodSubsystem() {

        angleMotor = new TalonSRX(Constants.Motors.kHoodMotorID);
        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        angleMotor.config_kF(0, 0.00001, 10);
        angleMotor.config_kP(0, 0, 10);
        angleMotor.config_kI(0, 0, 10);
        angleMotor.config_kD(0, 0, 10);

    }

// To Do: Calculate Ticks by Distance method

// to Do: Zeroing hood sensor

// To Do: Max and Minimums for the Hood

// To Do: Return current angle of hood method 

}
