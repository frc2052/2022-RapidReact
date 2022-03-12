package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.CamMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

/** Subsystem for sending and checking toggles and selectable lists on the SmartDashboard */
public class DashboardControlsSubsystem extends SubsystemBase {

    private VisionSubsystem vision;
    private RobotContainer robotContainer;

    private SendableChooser<Autos> autoSelector;
    private SendableChooser<DriveMode> driveModeSelector;
    private SendableChooser<CamMode> limelightCamModeSelector;
    private SendableChooser<LEDStatusMode> ledStatusModeSelector;
    private SendableChooser<ButtonBindingsProfile> buttonBindingsProfileSelector;

    private int ledBrightness;
    private int lastLEDBrightness;

    private boolean limelightLEDsEnabled;
    private boolean limelightDriveCamToggle;
    // private boolean limelightPowerRelayToggle;
    private boolean limelightLEDOverride;
    private boolean isLimelightDead;

    private boolean lastLimelightLEDsEnabled;
    private boolean lastIsDriverCamera;
    // private boolean lastLimelightPowerRelayState;
    private boolean lastLimelightLEDOverride;
    private boolean lastIsLimelightDead;

    private LEDStatusMode lastLEDStatusMode;
    private Autos selectedAuto;
    private Autos lastSelectedAuto;

    private ButtonBindingsProfile lastSelectedButtonBindingsProfile;

    public DashboardControlsSubsystem(VisionSubsystem vision, RobotContainer robotContainer) { // Adds values and items to selectors and toggles. Currently don't like passing robot container but might need to...
        this.vision = vision;
        this.robotContainer = robotContainer;
        
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        // limelightPowerRelayToggle = vision.getRelayState();
        limelightLEDOverride = false;
        selectedAuto = Autos.NONE_SELECTED;
        isLimelightDead = false;

        lastLimelightLEDsEnabled = limelightLEDsEnabled;
        lastLEDBrightness = ledBrightness;
        lastIsDriverCamera = limelightDriveCamToggle;
        lastLEDStatusMode = LEDStatusMode.RAINBOW;
        // lastLimelightPowerRelayState = limelightPowerRelayToggle;
        lastLimelightLEDOverride = limelightLEDOverride;
        lastSelectedAuto = selectedAuto;
        lastIsLimelightDead = isLimelightDead;

        autoSelector = new SendableChooser<Autos>();
        driveModeSelector = new SendableChooser<DriveMode>();
        limelightCamModeSelector = new SendableChooser<CamMode>();
        ledStatusModeSelector = new SendableChooser<LEDStatusMode>();
        buttonBindingsProfileSelector = new SendableChooser<ButtonBindingsProfile>();

        autoSelector.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        for (int i = 1; i < Autos.values().length; i++) {
            autoSelector.addOption(Autos.values()[i].name, Autos.values()[i]);
        }

        ledStatusModeSelector.setDefaultOption(LEDStatusMode.values()[0].name, LEDStatusMode.RAINBOW);
        for (int i = 1; i < LEDStatusMode.values().length; i++) {
            ledStatusModeSelector.addOption(LEDStatusMode.values()[i].name, LEDStatusMode.values()[i]);
        }
    
        driveModeSelector.setDefaultOption("Robot Centric Drive", DriveMode.ROBOT_CENTRIC);
        driveModeSelector.addOption("Field Centric Drive", DriveMode.FIELD_CENTRIC);

        buttonBindingsProfileSelector.setDefaultOption(ButtonBindingsProfile.values()[0].name, ButtonBindingsProfile.values()[0]);
        for (int i = 1; i < ButtonBindingsProfile.values().length; i++) {
            buttonBindingsProfileSelector.addOption(ButtonBindingsProfile.values()[i].name, ButtonBindingsProfile.values()[i]);
        }

        lastSelectedButtonBindingsProfile = ButtonBindingsProfile.DEFAULT;

        // limelightCamModeSelector.setDefaultOption("Vision", CamMode.VISION);
        // limelightCamModeSelector.addOption("Driver", CamMode.DRIVER);
    }

    public void addSelectorsToSmartDashboard() {    // Method currently run in robotInit to add selectors to the SmartDashboard
        SmartDashboard.putData("Autos", autoSelector);
        SmartDashboard.putData("Drive Modes", driveModeSelector);
        SmartDashboard.putData("Limelight Cam Mode", limelightCamModeSelector);
        SmartDashboard.putData("LED Status Modes", ledStatusModeSelector);
        SmartDashboard.putData("Button Bindings Profiles", buttonBindingsProfileSelector);

        SmartDashboard.putNumber("LED Brightness", ledBrightness);
        SmartDashboard.putNumber("Stream Mode", vision.getStreamMode());

        SmartDashboard.putBoolean("Enable Limelight LEDs", limelightLEDsEnabled);
        SmartDashboard.putBoolean("Toggle Limelight Driver Camera", limelightDriveCamToggle);
        SmartDashboard.putBoolean("Enable Limelight LEDs", limelightLEDsEnabled);
        SmartDashboard.putBoolean("Is An Auto Selected?", false);
        // SmartDashboard.putBoolean("Limelight Power Relay", limelightPowerRelayToggle);
        SmartDashboard.putBoolean("Limelight LED Override", limelightLEDOverride);
        SmartDashboard.putBoolean("Limelight Is Dead Button", false);

        SmartDashboard.putString("Selected Auto Description", selectedAuto.description);
    }

    @Override
    public void periodic() {    // Periodic function to check for SmartDashboard changes in parallel with other loops. Intended to only do logic when somthing has changed.
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        LEDStatusMode selectedLEDStatusMode = getSelectedLEDStatusMode();
        ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);
        // limelightPowerRelayToggle = SmartDashboard.getBoolean("Limelight Power Relay", vision.getRelayState());
        limelightLEDOverride = SmartDashboard.getBoolean("Limelight LED Override", false);
        selectedAuto = getSelectedAuto();
        isLimelightDead = SmartDashboard.getBoolean("Limelight Is Dead Button", false);

        if (limelightLEDsEnabled != lastLimelightLEDsEnabled) {
            if(limelightLEDsEnabled) {
                vision.setLED(LEDMode.ON);
                lastLimelightLEDsEnabled = true;
            } else {
                vision.setLED(LEDMode.OFF);
                lastLimelightLEDsEnabled = false;
            }
        }

        if (limelightDriveCamToggle != lastIsDriverCamera) {
            if(limelightDriveCamToggle) {
                vision.setLED(LEDMode.OFF);
                vision.setCamMode(CamMode.DRIVER);
                lastIsDriverCamera = true;
            } else {
                vision.setLED(LEDMode.ON);
                vision.setCamMode(CamMode.VISION);
                lastIsDriverCamera = false;
            }
        }

        if (selectedLEDStatusMode != lastLEDStatusMode) {
            LEDSubsystem.getInstance().setLEDStatusMode(selectedLEDStatusMode);
            lastLEDStatusMode = selectedLEDStatusMode;
        }

        if (ledBrightness != lastLEDBrightness) {
            LEDSubsystem.getInstance().setBrightness(ledBrightness);
            lastLEDBrightness = ledBrightness;
        }

        if (selectedAuto == Autos.NONE_SELECTED) {
            SmartDashboard.putBoolean("Is An Auto Selected?", false);
        } else {
            SmartDashboard.putBoolean("Is An Auto Selected?", true);
        }

        // if (limelightPowerRelayToggle != lastLimelightPowerRelayState) {
        //     vision.togglePowerRelay();
        //     lastLimelightPowerRelayState = limelightPowerRelayToggle;
        // }

        if (limelightLEDOverride != lastLimelightLEDOverride) {
            if (limelightLEDOverride) {
                vision.setLED(LEDMode.ON);
                vision.setLEDOverride(true);
                lastLimelightLEDOverride = true;
            } else {
                vision.setLED(LEDMode.OFF);
                vision.setLEDOverride(false);
                lastLimelightLEDOverride = false;
            }
        }

        if (selectedAuto != lastSelectedAuto && selectedAuto != null) {
            SmartDashboard.putString("Selected Auto Description", selectedAuto.description);
            robotContainer.initializeAutonomousCommand();
            lastSelectedAuto = selectedAuto;
        }

        if (isLimelightDead != lastIsLimelightDead) {
            //System.err.println("SWITCHED !!!!!!!!!!!!!!!!!");
            
            lastIsLimelightDead = isLimelightDead;
        }

        if (getSelectButtonBindingsProfile() != lastSelectedButtonBindingsProfile) {
            robotContainer.assignButtonBindings(getSelectButtonBindingsProfile());
            lastSelectedButtonBindingsProfile = getSelectButtonBindingsProfile();
        }
    }

    // public BooleanSupplier getIsLimelightDead() {
    //     return () -> { return isLimelightDead; };
    // }

    public Autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public DriveMode getSelectedDriveMode() {
        return driveModeSelector.getSelected();
    }

    public LEDStatusMode getSelectedLEDStatusMode() {
        return ledStatusModeSelector.getSelected();
    }

    public ButtonBindingsProfile getSelectButtonBindingsProfile() {
        return buttonBindingsProfileSelector.getSelected();
    }

    public void setLastLimelightLEDsEnabled(boolean state) {
        lastLimelightLEDsEnabled = state;
    }

    public enum Autos {
        NONE_SELECTED("NO AUTO SELECTED", "NO AUTO SELECTED"),
        ONE_BALL("*TUNED* One Ball", "Any Starting Location Facing Towards Hub"),
        AUTO_TESTING("Auto Testing", "Auto Testing"),
        SIMPLE_3_BALL("*TUNED* Simple 3 Ball", "Far Right Start (A) Facing Towards Hub"),
        //THREE_BALL_DRIVE_AND_SHOOT("3 Ball Drive and Shoot - Far Right Start (A) Facing Away From Hub"),
        //LEFT_TERMINAL_3_BALL("3 Ball Including Terminal Cargo - Far Left Start (D) Facing Away Hub"),
        LEFT_2_BALL_1_DEFENSE("*TUNED* 2 Ball and 1 Defence", "Far Left Start (D) Facing Away From Hub"),
        //MIDDLE_RIGHT_TERMINAL_3_BALL("Terminal 3 Ball - Middle Right Start (B) Facing Away From Hub"),
        MIDDLE_RIGHT_TERMINAL_4_BALL("*TUNED* Terminal 4 Ball", "Middle Left Start (C) Facing Towards Hub"),
        //MIDDLE_LEFT_TERMINAL_DEFENSE("2 Ball Terminal And Defense - Middle Left Start (C) Facing Towards Hub"),
        MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE("*TUNED* Terminal 3 Ball", "Middle Left Start (C) Facing Towards Hub"),
        RIGHT_FIVE_BALL("*TUNED* Right Five Ball Auto", "Far Right Start (A2) Facing Towards Hub"),
        LEFT_2_BALL_2_DEFENSE("-INITIAL TESTING- Left 2 Ball 2 Defense", "Far Left Start (D) Facing Away From Hub"),
        RIGHT_MIDDLE_5_BALL_1_DEFENSE("_TUNING_ Right Middle 5 Ball 1 Defense", "Right Middle Start (B) Facing Away From Hub");

        public String name;
        public String description;

        Autos(String name, String description) {
            this.name = name;
            this.description = description;
        }
    }

    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
    }

    // This definately doesn't work but maybe it's steps in the rights direction
    public enum ButtonBindingsProfile {
        DEFAULT("Default"),
        SOLO_DRIVER("Solo Driver");

        public String name;
        // public int[] turnJoystickBindings;
        // public int[] driveJoystickBindings;
        // public int[] secondaryPannelBindings;

        ButtonBindingsProfile(String name) {
            this.name = name;
        } 
        // ButtonBindingsProfile(String name, int[] turnJoystickBindings, int[] driveJoystickBindings, int[] secondaryPannelBindings) {
        //     this.name = name;
        //     this.turnJoystickBindings = turnJoystickBindings;
        //     this.driveJoystickBindings = driveJoystickBindings;
        //     this.secondaryPannelBindings = secondaryPannelBindings;
        // }
    }
}
