package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  private Compressor compressor;

  public PneumaticsSubsystem() {
    compressor = new Compressor(Constants.Solenoids.COMPRESSOR_MODULE_ID, PneumaticsModuleType.REVPH);
    compressor.enableAnalog(100, 115);
  }

  @Override
  public void periodic() {
    double currentPressure = compressor.getPressure();
    SmartDashboard.putNumber("currentPressure", currentPressure);
  }
}
