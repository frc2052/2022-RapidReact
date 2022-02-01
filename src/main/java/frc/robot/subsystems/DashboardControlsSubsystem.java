package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class DashboardControlsSubsystem {

    private VisionSubsystem m_vision;

    private boolean limelightLEDsEnabled;
    private boolean lastLEDState;

    private static SendableChooser<driveMode> sendableChooserDriveMode;

    public DashboardControlsSubsystem(VisionSubsystem vision) {
        m_vision = vision;
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        lastLEDState = limelightLEDsEnabled;

        sendableChooserDriveMode = new SendableChooser<driveMode>();
    
        sendableChooserDriveMode.setDefaultOption("Field Centric Drive", driveMode.FIELDCENTRIC);
        sendableChooserDriveMode.addOption("Robot Centric Drive", driveMode.ROBOTCENTRIC);
    }

    public void addSelectorsToSmartDashboard() {
        SmartDashboard.putBoolean("Enable Limelight LEDs", limelightLEDsEnabled);
        SmartDashboard.putData("Drive Modes", sendableChooserDriveMode);
    }

    public void checkSmartDashboardControls() {
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);

        if(limelightLEDsEnabled && !lastLEDState) {
            System.out.println("*********Turning on**********");
            m_vision.setLED(LEDMode.ON);
        } else if (!limelightLEDsEnabled && lastLEDState) {
            System.out.println("*********Turning off***********");

            m_vision.setLED(LEDMode.OFF);
        }
        lastLEDState = limelightLEDsEnabled;
    }

    public driveMode getSelectedDriveMode() {
        driveMode selectedDriveMode = sendableChooserDriveMode.getSelected();
        return selectedDriveMode;
    }

    public enum driveMode {
        FIELDCENTRIC,
        ROBOTCENTRIC,
    }
}
