package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  private final PneumaticHub pneumaticHub;

  public PneumaticsSubsystem() {
    pneumaticHub = new PneumaticHub(Constants.Solenoids.COMPRESSOR_MODULE_ID);
    pneumaticHub.clearStickyFaults();
    pneumaticHub.enableCompressorAnalog(100, 115);
  }

  @Override
  public void periodic() {
    // Gets the current compressor pressure from channel 0.
    double currentPressure = pneumaticHub.getPressure(0);
    SmartDashboard.putNumber("currentPressure", currentPressure);
  }
}
