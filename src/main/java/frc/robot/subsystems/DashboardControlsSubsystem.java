package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

/** Subsystem for sending and checking toggles and selectable lists on the SmartDashboard */
public class DashboardControlsSubsystem {

    private VisionSubsystem m_vision;

    private SendableChooser<autos> autoSelector;
    private SendableChooser<driveMode> driveModeSelector;

    private boolean limelightLEDsEnabled;
    private boolean lastLEDState;

    public DashboardControlsSubsystem(VisionSubsystem vision) { // Adds values and items to selectors and toggles.
        m_vision = vision;
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        lastLEDState = limelightLEDsEnabled;

        autoSelector = new SendableChooser<autos>();

        autoSelector.setDefaultOption(autos.values()[0].name, autos.values()[0]);
        for(int i = 1; i < autos.values().length; i++)
            autoSelector.setDefaultOption(autos.values()[i].name, autos.values()[i]);

        SmartDashboard.putData("Auto Options", autoSelector);

        driveModeSelector = new SendableChooser<driveMode>();
    
        driveModeSelector.setDefaultOption("Field Centric Drive", driveMode.FIELD_CENTRIC);
        driveModeSelector.addOption("Robot Centric Drive", driveMode.ROBOT_CENTRIC);
    }

    public void addSelectorsToSmartDashboard() {    // Method currently run in robotInit to add selectors to the SmartDashboard
        SmartDashboard.putBoolean("Enable Limelight LEDs", limelightLEDsEnabled);
        SmartDashboard.putData("Autos", autoSelector);
        SmartDashboard.putData("Drive Modes", driveModeSelector);
    }

    public void checkSmartDashboardControls() { // Method currently executed in robotPeriodic that checks toggles and performs subsystem methods as needed
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);

        if(limelightLEDsEnabled && !lastLEDState) {
            m_vision.setLED(LEDMode.ON);
        } else if (!limelightLEDsEnabled && lastLEDState) {
            m_vision.setLED(LEDMode.OFF);
        }
        lastLEDState = limelightLEDsEnabled;
    }

    public autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public driveMode getSelectedDriveMode() {
        return driveModeSelector.getSelected();
    }

    public enum autos {
        TEST_AUTO_1("TestAuto1"),
        SIMPLE_1_BALL("Simple 1 Ball"),
        SIMPLE_3_BALL("Simple 3 Ball"),
        THREE_BALL_TERMINAL("3 Ball - Terminal Cargo"),
        FOUR_BALL("4 Ball Auto");

        public String name;

        autos(String name) {
            this.name = name;
        }
    }

    public enum driveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
    }
}
