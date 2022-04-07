package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDAlertStatusMode;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.CamMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

/** Subsystem for sending and checking toggles and selectable lists on the SmartDashboard */
public class DashboardControlsSubsystem extends SubsystemBase {

    private final VisionSubsystem vision;
    private final RobotContainer robotContainer;
    private final ShooterSubsystem shooter;

    private SendableChooser<Autos> autoSelector;
    private SendableChooser<DriveMode> driveModeSelector;
    private SendableChooser<CamMode> limelightCamModeSelector;
    private SendableChooser<LEDStatusMode> ledStatusModeSelector;
    private SendableChooser<ButtonBindingsProfile> buttonBindingsProfileSelector;

    // private int ledBrightness;
    // private int lastLEDBrightness;
    private int limelightDeadSeconds;
    private double lastShooterBoostPct;

    // private boolean limelightLEDsEnabled;
    // private boolean limelightDriveCamToggle;
    // private boolean limelightPowerRelayToggle;
    // private boolean limelightLEDOverride;
    private boolean isLimelightDead;
    // private boolean isDrivetrainDead;
    // private boolean isPnuematicsDead;

    private boolean lastLimelightLEDsEnabled;
    private boolean lastIsDriverCamera;
    // private boolean lastLimelightPowerRelayState;
    private boolean lastLimelightLEDOverride;
    private boolean lastIsLimelightDead;
    // private boolean lastIsDrivetrainDead;
    private boolean lastIsPnuematicsDead;

    // private LEDStatusMode lastLEDStatusMode;
    private Autos selectedAuto;
    private Autos lastSelectedAuto;
    // private ButtonBindingsProfile lastSelectedButtonBindingsProfile;
    private Timer timer;

    private DashboardControlsSubsystem(VisionSubsystem vision, RobotContainer robotContainer, ShooterSubsystem shooter) { // Adds values and items to selectors and toggles. Currently don't like passing robot container but might need to...
        this.vision = vision;
        this.robotContainer = robotContainer;
        this.shooter = shooter;
        
        // ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);
        selectedAuto = Autos.NONE_SELECTED;
        isLimelightDead = false;
        // limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        // limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        // limelightPowerRelayToggle = vision.getRelayState();
        // limelightLEDOverride = false;
        // isDrivetrainDead = false;
        // isPnuematicsDead = false;

        // lastLEDBrightness = ledBrightness;
        // lastLEDStatusMode = LEDStatusMode.RAINBOW;
        lastSelectedAuto = selectedAuto;
        lastIsLimelightDead = isLimelightDead;
        // lastLimelightPowerRelayState = limelightPowerRelayToggle;

        // Unnecessary, but might be nice for understanding
        lastLimelightLEDsEnabled = false;
        lastIsDriverCamera = false;
        lastLimelightLEDOverride = false;
        // lastIsDrivetrainDead = false;
        lastIsPnuematicsDead = false;

        autoSelector = new SendableChooser<Autos>();
        driveModeSelector = new SendableChooser<DriveMode>();
        limelightCamModeSelector = new SendableChooser<CamMode>();
        ledStatusModeSelector = new SendableChooser<LEDStatusMode>();
        buttonBindingsProfileSelector = new SendableChooser<ButtonBindingsProfile>();

        autoSelector.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        for (int i = 1; i < Autos.values().length; i++) {
            autoSelector.addOption(Autos.values()[i].name, Autos.values()[i]);
        }

        // ledStatusModeSelector.setDefaultOption(LEDStatusMode.values()[0].toString(), LEDStatusMode.RAINBOW);
        // for (int i = 1; i < LEDStatusMode.values().length; i++) {
        //     ledStatusModeSelector.addOption(LEDStatusMode.values()[i].toString(), LEDStatusMode.values()[i]);
        // }
    
        driveModeSelector.setDefaultOption("Robot Centric Drive", DriveMode.ROBOT_CENTRIC);
        driveModeSelector.addOption("Field Centric Drive", DriveMode.FIELD_CENTRIC);

        // buttonBindingsProfileSelector.setDefaultOption(ButtonBindingsProfile.values()[0].name, ButtonBindingsProfile.values()[0]);
        // for (int i = 1; i < ButtonBindingsProfile.values().length; i++) {
        //     buttonBindingsProfileSelector.addOption(ButtonBindingsProfile.values()[i].name, ButtonBindingsProfile.values()[i]);
        // }

        // lastSelectedButtonBindingsProfile = ButtonBindingsProfile.DEFAULT;

        // limelightCamModeSelector.setDefaultOption("Vision", CamMode.VISION);
        // limelightCamModeSelector.addOption("Driver", CamMode.DRIVER);
    }

    // Singleton pattern to make sure only one instance of this subsystems exists, it can be called from anywhere, and has an init method to pass the pointers to other classes.
    private static DashboardControlsSubsystem instance;
    public static void init(VisionSubsystem vision, RobotContainer robotContainer, ShooterSubsystem shooter) {
        if (instance == null) {
            instance = new DashboardControlsSubsystem(vision, robotContainer, shooter);
        }
    }
    public static DashboardControlsSubsystem getInstance() {
        if (instance == null) {
            System.err.println("ATTEMPTED TO GET DASHBOARD CONTROLS INSTANCE WHEN NULL");
        }
        return instance;
    }

    public void addSelectorsToSmartDashboard() {    // Method currently run in robotInit to add selectors to the SmartDashboard
        SmartDashboard.putData("Autos", autoSelector);
        SmartDashboard.putData("Drive Modes", driveModeSelector);
        SmartDashboard.putData("Limelight Cam Mode", limelightCamModeSelector);
        // SmartDashboard.putData("LED Status Modes", ledStatusModeSelector);
        // SmartDashboard.putData("Button Bindings Profiles", buttonBindingsProfileSelector);
        SmartDashboard.putString("Selected Auto Description", selectedAuto.description);

        // SmartDashboard.putNumber("LED Brightness", ledBrightness);
        // SmartDashboard.putNumber("Stream Mode", vision.getStreamMode());
        SmartDashboard.putNumber("Shooter Velocity Boost Pct", 0);

        SmartDashboard.putBoolean("Enable Limelight LEDs", false);
        SmartDashboard.putBoolean("Toggle Limelight Driver Camera", false);
        SmartDashboard.putBoolean("Is An Auto Selected?", false);
        SmartDashboard.putBoolean("Limelight LED Override", false);
        SmartDashboard.putBoolean("Lost Limelight Comm Warning", false);

        // Dead subsystem buttons
        SmartDashboard.putBoolean("Limelight Is Dead", false);
        // SmartDashboard.putBoolean("Drivetrain Is Dead", false);
        SmartDashboard.putBoolean("Pnuematics Is Dead", false);
        // SmartDashboard.putBoolean("Limelight Power Relay", limelightPowerRelayToggle);
    }

    @Override
    public void periodic() {    // Periodic function to check for SmartDashboard changes in parallel with other loops. Intended to only do logic when somthing has changed.
        // LEDStatusMode selectedLEDStatusMode = getSelectedLEDStatusMode();
        Autos selectedAuto = getSelectedAuto();
        // ButtonBindingsProfile selectedButtonBindingsProfile = getSelectButtonBindingsProfile();
        // int ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);
        double shooterBoostPct = SmartDashboard.getNumber("Shooter Velocity Boost Pct", 0);
        Boolean limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        Boolean limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        boolean limelightLEDOverride = SmartDashboard.getBoolean("Limelight LED Override", false);
        boolean isLimelightDead = !vision.getIsUpdating(); //SmartDashboard.getBoolean("Limelight Is Dead", false);
        // boolean isDrivetrainDead = SmartDashboard.getBoolean("Drivetrain Is Dead", false);
        boolean isPnuematicsDead = SmartDashboard.getBoolean("Pnuematics Is Dead", false);
        // limelightPowerRelayToggle = SmartDashboard.getBoolean("Limelight Power Relay", vision.getRelayState());

        if (limelightLEDsEnabled != lastLimelightLEDsEnabled) {
            if(limelightLEDsEnabled) {
                vision.setLEDMode(LEDMode.ON);
                lastLimelightLEDsEnabled = true;
            } else {
                vision.setLEDMode(LEDMode.OFF);
                lastLimelightLEDsEnabled = false;
            }
        }

        if (limelightDriveCamToggle != lastIsDriverCamera) {
            if(limelightDriveCamToggle) {
                vision.setLEDMode(LEDMode.OFF);
                vision.setCamMode(CamMode.DRIVER);
                lastIsDriverCamera = true;
            } else {
                vision.setLEDMode(LEDMode.ON);
                vision.setCamMode(CamMode.VISION);
                lastIsDriverCamera = false;
            }
        }

        // if (selectedLEDStatusMode != lastLEDStatusMode) {
        //     LEDSubsystem.getInstance().setLEDStatusMode(selectedLEDStatusMode);
        //     lastLEDStatusMode = selectedLEDStatusMode;
        // }

        // if (ledBrightness != lastLEDBrightness) {
        //     LEDSubsystem.setBothChannelBrightnesses(ledBrightness);
        //     lastLEDBrightness = ledBrightness;
        // }

        if (shooterBoostPct != lastShooterBoostPct) {
            shooter.setShooterVelocityBoost(shooterBoostPct);
            lastShooterBoostPct = shooterBoostPct;
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
                vision.setLEDMode(LEDMode.ON);
                vision.setLEDOverride(true);
                lastLimelightLEDOverride = true;
            } else {
                vision.setLEDOverride(false);
                vision.setLEDMode(LEDMode.OFF);
                lastLimelightLEDOverride = false;
            }
        }

        if (selectedAuto != lastSelectedAuto && selectedAuto != null) {
            SmartDashboard.putString("Selected Auto Description", selectedAuto.description);
            robotContainer.initializeAutonomousCommand();
            lastSelectedAuto = selectedAuto;
        }

        // if (selectedButtonBindingsProfile != lastSelectedButtonBindingsProfile && selectedButtonBindingsProfile != null) {
        //     robotContainer.assignButtonBindings(selectedButtonBindingsProfile);
        //     lastSelectedButtonBindingsProfile = selectedButtonBindingsProfile;
        // }

        if (isLimelightDead) {
            flashLostLimelightCommunicationWarning();
            // this.isLimelightDead = isLimelightDead;
            // lastIsLimelightDead = isLimelightDead;
        } else {
            SmartDashboard.putBoolean("Lost Limelight Comm Warning", false);
            limelightDeadSeconds = 0;
            clearTimer();
        }

        // if (isDrivetrainDead != lastIsDrivetrainDead) {
        //     if (isDrivetrainDead) {
        //         LEDSubsystem.getInstance().setAlertLEDStatusMode(LEDAlertStatusMode.LIGHT_SHOW);
        //     } else {
        //         LEDSubsystem.getInstance().clearAlertStatusMode();
        //     }
        //     lastIsDrivetrainDead = isDrivetrainDead;
        // }

        if (isPnuematicsDead != lastIsPnuematicsDead) {
            if (isPnuematicsDead) {
                shooter.setShootAngle1();
                shooter.setShootAngle1Override(true);
            } else {
                shooter.setShootAngle1Override(false);
            }
            lastIsPnuematicsDead = isPnuematicsDead;
        }

        // SmartDashboard.putBoolean("Shooting", robotContainer.getIsShooting());
    }

    // Didn't work
    // public void reset SelectedAuto() {
    //     SendableChooser<Autos> autoSelector1 = new SendableChooser<Autos>();
    //     autoSelector1.setDefaultOption(Autos.NONE_SELECTED.name, Autos.NONE_SELECTED);
    //     SmartDashboard.putData("Autos", autoSelector1);
    //     SmartDashboard.putData("Autos", autoSelector);
    // }

    public boolean getIsLimelightDead() {
        return isLimelightDead;
    }

    public Autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public DriveMode getSelectedDriveMode() {
        return driveModeSelector.getSelected();
    }

    // public LEDStatusMode getSelectedLEDStatusMode() {
    //     return ledStatusModeSelector.getSelected();
    // }

    // public ButtonBindingsProfile getSelectButtonBindingsProfile() {
    //     return buttonBindingsProfileSelector.getSelected();
    // }

    public double getShooterVelocityBoost() {
        return SmartDashboard.getNumber("Shooter Velocity Boost Pct", 0);
    }

    public void setLastLimelightLEDsEnabled(boolean state) {
        lastLimelightLEDsEnabled = state;
    }

    public void flashLostLimelightCommunicationWarning() {
        if (timer == null) {
            timer = new Timer();
            timer.start();
        }

        if (timer.hasElapsed(0.5)) {
            limelightDeadSeconds += 0.5;
            System.err.println("*** LIMELIGHT HASN't UPDATED IN " + (int) limelightDeadSeconds + " SECONDS");
            timer.reset();
        } else if (timer.hasElapsed(0.25)) {
            SmartDashboard.putBoolean("Lost Limelight Comm Warning", false);
        } else {
            SmartDashboard.putBoolean("Lost Limelight Comm Warning", true);
        }
    }

    public enum Autos {
        NONE_SELECTED("NO AUTO SELECTED", "NO AUTO SELECTED"),
        ONE_BALL("*TUNED* One Ball", "Any Starting Location Facing Towards Hub"),
        // AUTO_TESTING("Auto Testing", "Auto Testing"),
        SIMPLE_3_BALL("*TUNED* Simple 3 Ball", "Far Right Start (A) Facing Towards Hub"),
        //THREE_BALL_DRIVE_AND_SHOOT("3 Ball Drive and Shoot - Far Right Start (A) Facing Away From Hub"),
        //LEFT_TERMINAL_3_BALL("3 Ball Including Terminal Cargo - Far Left Start (D) Facing Away Hub"),
        LEFT_2_BALL_1_DEFENSE("*TUNED* 2 Ball and 1 Defence", "Far Left Start (D) Facing Away From Hub"),
        //MIDDLE_RIGHT_TERMINAL_3_BALL("Terminal 3 Ball - Middle Right Start (B) Facing Away From Hub"),
        MIDDLE_RIGHT_TERMINAL_4_BALL("*TUNED* Terminal 4 Ball", "Middle Left Start (C) Facing Towards Hub"),
        //MIDDLE_LEFT_TERMINAL_DEFENSE("2 Ball Terminal And Defense - Middle Left Start (C) Facing Towards Hub"),
        MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE("*TUNED* Terminal 3 Ball", "Middle Left Start (C) Facing Towards Hub"),
        RIGHT_FIVE_BALL("*TUNED* Original Right Five Ball Auto", "Far Right Start (A2) Facing Towards Hub"),
        FAST_RIGHT_FIVE_BALL_3("*TUNED* Right Five Ball Auto 3 - Faster", "Far Right Start (A2) Facing Away From Hub");
        // LEFT_2_BALL_2_DEFENSE("-INITIAL TESTING- Left 2 Ball 2 Defense", "Far Left Start (D) Facing Away From Hub"),
        // RIGHT_MIDDLE_5_BALL_1_DEFENSE("_TUNING_ Right Middle 5 Ball 1 Defense", "Right Middle Start (B) Facing Away From Hub"),
        // RIGHT_FIVE_BALL_2("-INITIAL TESTING- Right Five Ball Auto 2 - Drive And Shoot", "Far Right Start (A2) Facing Away From Hub"),

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

    private void clearTimer() {
        if(timer != null) {
            timer.stop();
            timer = null;
        }
    }
}
