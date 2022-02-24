package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber.ZeroClimberEncoderCommand;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.CamMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

/** Subsystem for sending and checking toggles and selectable lists on the SmartDashboard */
public class DashboardControlsSubsystem extends SubsystemBase {

    private VisionSubsystem vision;
    private HookClimberSubsystem climber;

    private SendableChooser<Autos> autoSelector;
    private SendableChooser<DriveMode> driveModeSelector;
    private SendableChooser<CamMode> limelightCamModeSelector;
    private SendableChooser<LEDStatusMode> ledStatusModeSelector;
    private SendableChooser<ButtonBindingsProfile> buttonBindingsProfileSelector;

    private int ledBrightness;
    private int lastLEDBrightness;

    private boolean limelightLEDsEnabled;
    private boolean lastLimelightLEDsEnabled;
    private boolean limelightDriveCamToggle;
    private boolean lastIsDriverCamera;
    private boolean isAutoSelected;

    private LEDStatusMode lastLEDStatusMode;

    public DashboardControlsSubsystem(VisionSubsystem vision, HookClimberSubsystem climber) { // Adds values and items to selectors and toggles.
        this.vision = vision;
        this.climber = climber;
        
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);

        lastLimelightLEDsEnabled = limelightLEDsEnabled;
        lastLEDBrightness = ledBrightness;
        lastIsDriverCamera = limelightDriveCamToggle;
        lastLEDStatusMode = LEDStatusMode.RAINBOW;

        autoSelector = new SendableChooser<Autos>();
        driveModeSelector = new SendableChooser<DriveMode>();
        limelightCamModeSelector = new SendableChooser<CamMode>();
        // ledStatusModeSelector = new SendableChooser<LEDStatusMode>();

        autoSelector.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        for (int i = 1; i < Autos.values().length; i++) {
            autoSelector.addOption(Autos.values()[i].name, Autos.values()[i]);
        }

        ledStatusModeSelector.setDefaultOption(LEDStatusMode.values()[0].name, LEDStatusMode.values()[0]);
        for (int i = 1; i < LEDStatusMode.values().length; i++) {
            ledStatusModeSelector.addOption(LEDStatusMode.values()[i].name, LEDStatusMode.values()[i]);
        }
    
        driveModeSelector.setDefaultOption("Field Centric Drive", DriveMode.FIELD_CENTRIC);
        driveModeSelector.addOption("Robot Centric Drive", DriveMode.ROBOT_CENTRIC);

        buttonBindingsProfileSelector.setDefaultOption(ButtonBindingsProfile.values()[0].name, ButtonBindingsProfile.values()[0]);
        for (int i = 1; i < ButtonBindingsProfile.values().length; i++) {
            buttonBindingsProfileSelector.addOption(ButtonBindingsProfile.values()[i].name, ButtonBindingsProfile.values()[i]);
        }

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
    }

    @Override
    public void periodic() {    // Periodic function to check for SmartDashboard changes in parallel with other loops.
        limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        LEDStatusMode selectedLEDStatusMode = getSelectedLEDStatusMode();
        ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);

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
                vision.setCamMode(CamMode.VISION);
                lastIsDriverCamera = limelightDriveCamToggle;
            } else {
                vision.setCamMode(CamMode.DRIVER);
                lastIsDriverCamera = limelightDriveCamToggle;
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

        if (this.getSelectedAuto() == Autos.ONE_BALL) {
            SmartDashboard.putBoolean("Is An Auto Selected?", false);
        } else {
            SmartDashboard.putBoolean("Is An Auto Selected?", true);
        }
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

    public ButtonBindingsProfile getSelectButtonBindingsProfile() {
        return buttonBindingsProfileSelector.getSelected();
    }

    public void setLastLimelightLEDsEnabled(boolean state) {
        lastLimelightLEDsEnabled = state;
    }

    public enum Autos {
        ONE_BALL("DEFAULT WHEN NO OTHER SELECTED - One Ball Auto, Any Starting Location Facing Towards Hub"),
        AUTO_TESTING("Auto Testing"),
        SIMPLE_3_BALL("Simple 3 Ball - Far Right Start (A) Facing Towards Hub"),
        THREE_BALL_DRIVE_AND_SHOOT("3 Ball Drive and Shoot - Far Right Start (A) Facing Away From Hub"),
        LEFT_TERMINAL_3_BALL("3 Ball Including Terminal Cargo - Far Left Start (D) Facing Away Hub"),
        LEFT_2_BALL_1_DEFENSE("2 Ball and 1 Defence - Far Left Start (D) Facing Away From Hub"),
        MIDDLE_RIGHT_TERMINAL_3_BALL("Terminal 3 Ball - Middle Right Start (B) Facing Away From Hub"),
        MIDDLE_RIGHT_TERMINAL_4_BALL("Terminal 4 Ball - Middle Right Start (B) Facing Away From Hub"),
        MIDDLE_LEFT_TERMINAL_DEFENSE("2 Ball Terminal And Defense - Middle Left Start (C) Facing Towards Hub"),
        MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE("Other version of middle left terminal defense"),
        RIGHT_FIVE_BALL("Right Five Ball Auto - Far Right Start (A) Facing Away From Hub"),
        RIGHT_FIVE_BALL_2("Right Five Ball Auto 2 - Far Right Start (A) Facing Away From Hub"),
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

    // This definately doesn't work but maybe it's steps in the rights direction
    public enum ButtonBindingsProfile {
        DEFAULT("Default", new int[] {11, 1, 7, 6, 1, 1, 5}, new int[] {12, 3, 5, 7}, new int[] {1, 9, 3}),
        SOLO_DRIVER("Solo Driver", new int[] {4, 6, 8, 10, 12}, new int[] {7, 2, 7, 9}, new int[] {4, 8, 11});

        public String name;
        public int[] turnJoystickBindings;
        public int[] driveJoystickBindings;
        public int[] secondaryPannelBindings;

        ButtonBindingsProfile(String name, int[] turnJoystickBindings, int[] driveJoystickBindings, int[] secondaryPannelBindings) {
            this.name = name;
            this.turnJoystickBindings = turnJoystickBindings;
            this.driveJoystickBindings = driveJoystickBindings;
            this.secondaryPannelBindings = secondaryPannelBindings;
        }
    }
}
