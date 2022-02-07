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
    private boolean limelightDriveCamToggle;
    private boolean lastCamState;
    private CamMode lastCamMode;

    public DashboardControlsSubsystem(VisionSubsystem vision) { // Adds values and items to selectors and toggles.
        m_vision = vision;
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        lastLEDState = limelightLEDsEnabled;
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        lastCamState = limelightDriveCamToggle;
        lastCamMode = CamMode.VISION;

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
        SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightDriveCamToggle);
    }

    public void checkSmartDashboardControls() { // Method currently executed in robotPeriodic that checks toggles and performs subsystem methods as needed
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);

        if(limelightLEDsEnabled && !lastLEDState) {
            m_vision.setLED(LEDMode.ON);
        } else if (!limelightLEDsEnabled && lastLEDState) {
            m_vision.setLED(LEDMode.OFF);
        }
        lastLEDState = limelightLEDsEnabled;

        if(!limelightDriveCamToggle && lastCamState) {
            System.out.println("**********Limelight Set to Vision Mode**********");
            m_vision.setCamMode(CamMode.VISION);
            lastCamMode = CamMode.VISION;
        } else if (limelightDriveCamToggle && !lastCamState) {
            System.out.println("**********Limelight Set to Driver Camera Mode**********");
            m_vision.setCamMode(CamMode.DRIVER);
            lastCamMode = CamMode.DRIVER;
        }
        lastCamState = limelightDriveCamToggle;

        //Logic for having a selection list for limelight modes, unworking and uneeded for now...
        
        /* 
        if(limelightCamModeSelector.getSelected() == CamMode.DRIVER && (lastCamMode == CamMode.VISION)) {
            m_vision.setCamMode(CamMode.DRIVER);
            lastCamMode = CamMode.DRIVER;
            limelightDriveCamToggle = true;
            lastCamState = true;
            SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightDriveCamToggle);
        } else if (limelightCamModeSelector.getSelected() == CamMode.VISION && (lastCamMode == CamMode.DRIVER)) {
            m_vision.setCamMode(CamMode.VISION);
            lastCamMode = CamMode.VISION;
            limelightDriveCamToggle = false;
            lastCamState = false;
            SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightDriveCamToggle);
        }
        */
    }

    public Autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public DriveMode getSelectedDriveMode() {
        return driveModeSelector.getSelected();
    }

    public void setLastLEDState(boolean state) {
        lastLEDState = state;
    }

    public enum Autos {
        TEST_AUTO_1("TestAuto1"),
        SIMPLE_1_BALL("Simple 1 Ball"),
        SIMPLE_3_BALL("Simple 3 Ball"),
        SIMPLE_3_BALL_TESTING("Simple 3 Ball using front intake"),
        THREE_BALL_DRIVE_AND_SHOOT("3 Ball Drive and Shoot"),
        LEFT_TERMINAL("3 Ball - Terminal Cargo"),
        MIDDLE_DEFENSE("center start, with defense only"),
        MIDDLE_TERMINAL_3("center start, ball, then terminal ball"),
        MIDDLE_TERMINAL_DEFENSE("center start, terminal cargo and defense"),
        MIDDLE_TWO_DEFENSE("Center start, two cargo and defense"),
        FIVE_BALL("Five ball dream auto, rightmost start");

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