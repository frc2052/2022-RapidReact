package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.CamMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

/** Subsystem for sending and checking toggles and selectable lists on the SmartDashboard */
public class DashboardControlsSubsystem extends SubsystemBase {

    private VisionSubsystem visionSubsystem;

    private SendableChooser<Autos> autoSelector;
    private SendableChooser<DriveMode> driveModeSelector;
    private SendableChooser<CamMode> limelightCamModeSelector;
    private SendableChooser<LEDStatusMode> ledStatusModeSelector;

    private double ledBrightness;
    private double lastLEDBrightness;

    private boolean limelightLEDsEnabled;
    private boolean lastLEDState;
    private boolean limelightDriveCamToggle;
    private boolean lastCamState;
    private CamMode lastCamMode;

    private LEDStatusMode lastLEDStatusMode;

    public DashboardControlsSubsystem(VisionSubsystem vision) { // Adds values and items to selectors and toggles.
        this.visionSubsystem = vision;

        ledBrightness = SmartDashboard.getNumber("LED Brightness", 0.0);
        lastLEDBrightness = ledBrightness;

        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        lastLEDState = limelightLEDsEnabled;
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        lastCamState = limelightDriveCamToggle;
        lastCamMode = CamMode.VISION;
        lastLEDStatusMode = LEDStatusMode.RAINBOW;

        autoSelector = new SendableChooser<Autos>();
        driveModeSelector = new SendableChooser<DriveMode>();
        limelightCamModeSelector = new SendableChooser<CamMode>();
        ledStatusModeSelector = new SendableChooser<LEDStatusMode>();

        autoSelector.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        for(int i = 1; i < Autos.values().length; i++) {
            autoSelector.addOption(Autos.values()[i].name, Autos.values()[i]);
        }

        ledStatusModeSelector.setDefaultOption(LEDStatusMode.values()[0].name, LEDStatusMode.RAINBOW);
        for (int i = 1; i < LEDStatusMode.values().length; i++) {
            ledStatusModeSelector.addOption(LEDStatusMode.values()[i].name, LEDStatusMode.values()[i]);
        }
    
        driveModeSelector.setDefaultOption("Field Centric Drive", DriveMode.FIELD_CENTRIC);
        driveModeSelector.addOption("Robot Centric Drive", DriveMode.ROBOT_CENTRIC);

        limelightCamModeSelector.setDefaultOption("Vision", CamMode.VISION);
        limelightCamModeSelector.addOption("Driver", CamMode.DRIVER);
    }

    public void addSelectorsToSmartDashboard() {    // Method currently run in robotInit to add selectors to the SmartDashboard
        SmartDashboard.putData("Autos", autoSelector);
        SmartDashboard.putData("Drive Modes", driveModeSelector);
        SmartDashboard.putData("Limelight Cam Mode", limelightCamModeSelector);
        SmartDashboard.putData("LED Status Modes", ledStatusModeSelector);

        SmartDashboard.putNumber("LED Brightness", ledBrightness);

        SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightDriveCamToggle);
        SmartDashboard.putBoolean("Enable Limelight LEDs", limelightLEDsEnabled);
    }

    @Override
    public void periodic() {    // Periodic function to check for SmartDashboard changes in parallel with other loops.
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        LEDStatusMode selectedLEDStatusMode = getSelectedLEDStatusMode();
        ledBrightness = SmartDashboard.getNumber("LED Brightness", 0.0);

        if(limelightLEDsEnabled && !lastLEDState) {
            visionSubsystem.setLED(LEDMode.ON);
        } else if (!limelightLEDsEnabled && lastLEDState) {
            visionSubsystem.setLED(LEDMode.OFF);
        }
        lastLEDState = limelightLEDsEnabled;

        if(!limelightDriveCamToggle && lastCamState) {
            visionSubsystem.setCamMode(CamMode.VISION);
            lastCamMode = CamMode.VISION;
        } else if (limelightDriveCamToggle && !lastCamState) {
            visionSubsystem.setCamMode(CamMode.DRIVER);
            lastCamMode = CamMode.DRIVER;
        }
        lastCamState = limelightDriveCamToggle;

        if(selectedLEDStatusMode != lastLEDStatusMode) {
            LEDSubsystem.getInstance().setLEDStatusMode(selectedLEDStatusMode);
            lastLEDStatusMode = selectedLEDStatusMode;
        }

        if(ledBrightness != lastLEDBrightness) {
            LEDSubsystem.getInstance().setBrightness(ledBrightness);
            lastLEDBrightness = ledBrightness;
        }

        //Logic for having a selection list for limelight modes, unworking and uneeded for now...
        
        /* 
        if(limelightCamModeSelector.getSelected() == CamMode.DRIVER && (lastCamMode == CamMode.VISION)) {
            vision.setCamMode(CamMode.DRIVER);
            lastCamMode = CamMode.DRIVER;
            limelightDriveCamToggle = true;
            lastCamState = true;
            SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightDriveCamToggle);
        } else if (limelightCamModeSelector.getSelected() == CamMode.VISION && (lastCamMode == CamMode.DRIVER)) {
            vision.setCamMode(CamMode.VISION);
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

    public LEDStatusMode getSelectedLEDStatusMode() {
        return ledStatusModeSelector.getSelected();
    }

    public void setLastLEDState(boolean state) {
        lastLEDState = state;
    }

    public enum Autos {
        AUTO_TESTING("(A) Class for Testing Auto Actions and Paths"),
        TEST_AUTO_1("TestAuto1"),
        SIMPLE_1_BALL("Any Position Start, Simple 1 Ball"),
        SIMPLE_3_BALL("Far Right Start (A), Simple 3 Ball"),
        THREE_BALL_DRIVE_AND_SHOOT("Far Right Start (A), 3 Ball Drive and Shoot"),
        LEFT_TERMINAL_3_BALL("Far Left Start (D), 3 Ball Including Terminal Cargo"),
        LEFT_2_BALL_1_DEFENSE("Far Left Start (D), Shoot Two and Move Opponent Ball"),
        MIDDLE_TERMINAL_3_BALL("Center Start, ball, then terminal ball"),
        MIDDLE_TERMINAL_DEFENSE("center start, terminal cargo and defense"),
        FIVE_BALL("Far Right Start (A), Five Ball Auto"),
        RIGHT_MIDDLE_5_BALL_1_DEFENSE("Right Middle Start (B), Five Balls and 1 Ball Defense");

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
