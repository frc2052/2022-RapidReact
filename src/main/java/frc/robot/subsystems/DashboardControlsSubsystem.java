package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem.CamMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

/** Subsystem for sending and checking toggles and selectable lists on the SmartDashboard */
public class DashboardControlsSubsystem {

    private VisionSubsystem m_vision;

    private SendableChooser<Autos> autoSelector;
    private SendableChooser<DriveMode> driveModeSelector;
    private SendableChooser<CamMode> limelightCamModeSelector;

    private boolean limelightLEDsEnabled;
    private boolean lastLEDState;
    private boolean limelightCamModeToggle;
    private boolean lastCamState;

    public DashboardControlsSubsystem(VisionSubsystem vision) { // Adds values and items to selectors and toggles.
        m_vision = vision;
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        lastLEDState = limelightLEDsEnabled;
        limelightCamModeToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        lastCamState = limelightCamModeToggle;

        autoSelector = new SendableChooser<Autos>();
        driveModeSelector = new SendableChooser<DriveMode>();
        limelightCamModeSelector = new SendableChooser<CamMode>();

        autoSelector.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        for(int i = 1; i < Autos.values().length; i++) {
            autoSelector.setDefaultOption(Autos.values()[i].name, Autos.values()[i]);
        }
    
        driveModeSelector.setDefaultOption("Field Centric Drive", DriveMode.FIELD_CENTRIC);
        driveModeSelector.addOption("Robot Centric Drive", DriveMode.ROBOT_CENTRIC);

        limelightCamModeSelector.setDefaultOption("Vision", CamMode.VISION);
        limelightCamModeSelector.addOption("Driver", CamMode.DRIVER);
    }

    public void addSelectorsToSmartDashboard() {    // Method currently run in robotInit to add selectors to the SmartDashboard
        SmartDashboard.putBoolean("Enable Limelight LEDs", limelightLEDsEnabled);
        SmartDashboard.putData("Autos", autoSelector);
        SmartDashboard.putData("Drive Modes", driveModeSelector);
        SmartDashboard.putData("Limelight Cam Mode", limelightCamModeSelector);
        SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightCamModeToggle);
    }

    public void checkSmartDashboardControls() { // Method currently executed in robotPeriodic that checks toggles and performs subsystem methods as needed
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        limelightCamModeToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);

        if(limelightLEDsEnabled && !lastLEDState) {
            m_vision.setLED(LEDMode.ON);
        } else if (!limelightLEDsEnabled && lastLEDState) {
            m_vision.setLED(LEDMode.OFF);
        }
        lastLEDState = limelightLEDsEnabled;

        if(!limelightCamModeToggle && lastCamState) {
            m_vision.setCamMode(CamMode.VISION);
        } else if (limelightCamModeToggle && !lastCamState) {
            m_vision.setCamMode(CamMode.DRIVER);
        }
        lastCamState = limelightCamModeToggle;
    }

    public Autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public DriveMode getSelectedDriveMode() {
        return driveModeSelector.getSelected();
    }

    public enum Autos {
        TEST_AUTO_1("TestAuto1"),
        SIMPLE_1_BALL("Simple 1 Ball"),
        SIMPLE_3_BALL("Simple 3 Ball"),
        THREE_BALL_TERMINAL("3 Ball - Terminal Cargo"),
        FOUR_BALL("4 Ball Auto");

        public String name;

        Autos(String name) {
            this.name = name;
        }
    }

    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
    }
}
