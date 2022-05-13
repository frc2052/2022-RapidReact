package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
    // private SendableChooser<ButtonBindingsProfile> buttonBindingsProfileSelector;

    private double limelightDeadSeconds;
    private double lastShooterBoostPct;
    private double lastXAimOffset;
    // private int ledBrightness;
    // private int lastLEDBrightness;

    private boolean isLimelightDead;
    // private boolean limelightLEDsEnabled;
    // private boolean limelightDriveCamToggle;
    // private boolean limelightPowerRelayToggle;
    // private boolean limelightLEDOverride;
    // private boolean isDrivetrainDead;
    // private boolean isPnuematicsDead;

    private boolean lastLimelightLEDsEnabled;
    private boolean lastLimelightLEDOverride;
    private boolean lastIsPnuematicsDead;
    private boolean lastDisableShooterIdle;
    // private boolean lastIsDriverCamera;
    // private boolean lastLimelightPowerRelayState;
    // private boolean lastIsLimelightDead;
    // private boolean lastIsDrivetrainDead;

    private Autos selectedAuto;
    private Autos lastSelectedAuto;
    private Timer timer;
    // private ButtonBindingsProfile lastSelectedButtonBindingsProfile;
    // private LEDStatusMode lastLEDStatusMode;

    private DashboardControlsSubsystem(VisionSubsystem vision, RobotContainer robotContainer, ShooterSubsystem shooter) { // Adds values and items to selectors and toggles. Currently don't like passing robot container but might need to...
        this.vision = vision;
        this.robotContainer = robotContainer;
        this.shooter = shooter;
        
        selectedAuto = Autos.NONE_SELECTED;
        isLimelightDead = false;
        // ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);
        // limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);   // Gets the previous state of the LEDs on the dashbaord if left open.
        // limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        // limelightPowerRelayToggle = vision.getRelayState();
        // limelightLEDOverride = false;
        // isDrivetrainDead = false;
        // isPnuematicsDead = false;

        lastSelectedAuto = selectedAuto;
        // lastLEDStatusMode = LEDStatusMode.DISABLED;
        // lastLEDBrightness = ledBrightness;
        // lastIsLimelightDead = isLimelightDead;
        // lastLimelightPowerRelayState = limelightPowerRelayToggle;

        // Unnecessary, but might be nice for understanding
        lastLimelightLEDsEnabled = false;
        lastLimelightLEDOverride = false;
        lastIsPnuematicsDead = false;
        // lastIsDriverCamera = false;
        // lastIsDrivetrainDead = false;

        autoSelector = new SendableChooser<Autos>();
        driveModeSelector = new SendableChooser<DriveMode>();
        limelightCamModeSelector = new SendableChooser<CamMode>();
        ledStatusModeSelector = new SendableChooser<LEDStatusMode>();
        // buttonBindingsProfileSelector = new SendableChooser<ButtonBindingsProfile>();

        autoSelector.setDefaultOption(Autos.values()[0].name, Autos.values()[0]);
        for (int i = 1; i < Autos.values().length; i++) {
            autoSelector.addOption(Autos.values()[i].name, Autos.values()[i]);
        }

        ledStatusModeSelector.setDefaultOption(LEDStatusMode.values()[0].toString(), LEDStatusMode.OFF);
        for (int i = 1; i < LEDStatusMode.values().length; i++) {
            ledStatusModeSelector.addOption(LEDStatusMode.values()[i].toString(), LEDStatusMode.values()[i]);
        }
    
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

    /** Method currently run in robotInit to add selectors to the SmartDashboard */
    public void addSelectorsToSmartDashboard() {
        SmartDashboard.putData("Autos", autoSelector);
        SmartDashboard.putData("Drive Modes", driveModeSelector);
        SmartDashboard.putData("Limelight Cam Mode", limelightCamModeSelector);
        SmartDashboard.putData("LED Status Modes", ledStatusModeSelector);
        SmartDashboard.putString("Selected Auto Description", selectedAuto.description);
        // SmartDashboard.putData("Button Bindings Profiles", buttonBindingsProfileSelector);

        SmartDashboard.putNumber("Shooter Velocity Boost Pct", 0);
        SmartDashboard.putNumber("One Ball Delay", 0);
        SmartDashboard.putNumber("Vision X Aim Offset", vision.getXAimOffset());
        // SmartDashboard.putNumber("LED Brightness", ledBrightness);
        // SmartDashboard.putNumber("Stream Mode", vision.getStreamMode());

        SmartDashboard.putBoolean("Enable Limelight LEDs", false);
        SmartDashboard.putBoolean("Is An Auto Selected?", false);
        SmartDashboard.putBoolean("Limelight LED Override", false);
        SmartDashboard.putBoolean("Lost Limelight Comm Warning", false);
        SmartDashboard.putBoolean("Disable Shooter Idle", false);
        // SmartDashboard.putBoolean("Toggle Limelight Driver Camera", false);

        // Dead subsystem buttons
        SmartDashboard.putBoolean("Limelight Is Dead", false);
        SmartDashboard.putBoolean("Pnuematics Is Dead", false);
        // SmartDashboard.putBoolean("Drivetrain Is Dead", false);
        // SmartDashboard.putBoolean("Limelight Power Relay", limelightPowerRelayToggle);
    }

    // Periodic function to check for SmartDashboard changes in parallel with other loops. Intended to only do logic when somthing has changed.
    @Override
    public void periodic() {
        Autos selectedAuto = getSelectedAuto();
        double shooterBoostPct = SmartDashboard.getNumber("Shooter Velocity Boost Pct", 0);
        double visionXAimOffset = SmartDashboard.getNumber("Vision X Aim Offset", vision.getXAimOffset());
        boolean limelightLEDsEnabled = SmartDashboard.getBoolean("Enable Limelight LEDs", false);
        boolean limelightLEDOverride = SmartDashboard.getBoolean("Limelight LED Override", false);
        boolean isLimelightDead = !vision.getIsUpdating();
        boolean isPnuematicsDead = SmartDashboard.getBoolean("Pnuematics Is Dead", false);
        boolean disableShooterIdle = SmartDashboard.getBoolean("Disable Shooter Idle", false);
        LEDStatusMode selectedLEDStatusMode = getSelectedLEDStatusMode();
        // ButtonBindingsProfile selectedButtonBindingsProfile = getSelectButtonBindingsProfile();
        // boolean isDrivetrainDead = SmartDashboard.getBoolean("Drivetrain Is Dead", false);
        // boolean limelightDriveCamToggle = SmartDashboard.getBoolean("Toggle Limelight Driver Camera", false);
        // limelightPowerRelayToggle = SmartDashboard.getBoolean("Limelight Power Relay", vision.getRelayState());
        // int ledBrightness = (int)SmartDashboard.getNumber("LED Brightness", 100);

        if (limelightLEDsEnabled != lastLimelightLEDsEnabled) {
            if(limelightLEDsEnabled) {
                vision.setLEDMode(LEDMode.ON);
                lastLimelightLEDsEnabled = true;
            } else {
                vision.setLEDMode(LEDMode.OFF);
                lastLimelightLEDsEnabled = false;
            }
        }

        if (shooterBoostPct != lastShooterBoostPct) {
            shooter.setShooterVelocityBoost(shooterBoostPct);
            lastShooterBoostPct = shooterBoostPct;
        }

        if (visionXAimOffset != lastXAimOffset) {
            vision.setXAimOffset(visionXAimOffset);
            lastXAimOffset = visionXAimOffset;
        }

        if (selectedAuto == Autos.NONE_SELECTED) {
            SmartDashboard.putBoolean("Is An Auto Selected?", false);
        } else {
            SmartDashboard.putBoolean("Is An Auto Selected?", true);
        }

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

        if (disableShooterIdle != lastDisableShooterIdle) {
            shooter.setIdleSpeedEnabled(!disableShooterIdle);
            shooter.stop();
            lastDisableShooterIdle = disableShooterIdle;
        }

        if (selectedAuto != lastSelectedAuto && selectedAuto != null) {
            SmartDashboard.putString("Selected Auto Description", selectedAuto.description);
            robotContainer.initializeAutonomousCommand();
            lastSelectedAuto = selectedAuto;
        }

        if (isLimelightDead) {
            flashLostLimelightCommunicationWarning();
            // this.isLimelightDead = isLimelightDead;
            // lastIsLimelightDead = isLimelightDead;
            LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.LIMELIGHT_DEAD);
        } else {
            SmartDashboard.putBoolean("Lost Limelight Comm Warning", false);
            limelightDeadSeconds = 0;
            clearTimer();
        }

        if (isPnuematicsDead != lastIsPnuematicsDead) {
            if (isPnuematicsDead) {
                shooter.setShootAngle1();
                shooter.setShootAngle1Override(true);
            } else {
                shooter.setShootAngle1Override(false);
            }
            lastIsPnuematicsDead = isPnuematicsDead;
        }

        if (selectedLEDStatusMode != LEDStatusMode.OFF) {
            LEDSubsystem.getInstance().setLEDStatusMode(selectedLEDStatusMode);
            //lastLEDStatusMode = selectedLEDStatusMode;
        }

        // if (limelightDriveCamToggle != lastIsDriverCamera) {
        //     if(limelightDriveCamToggle) {
        //         vision.setLEDMode(LEDMode.OFF);
        //         vision.setCamMode(CamMode.DRIVER);
        //         lastIsDriverCamera = true;
        //     } else {
        //         vision.setLEDMode(LEDMode.ON);
        //         vision.setCamMode(CamMode.VISION);
        //         lastIsDriverCamera = false;
        //     }
        // }

        // if (ledBrightness != lastLEDBrightness) {
        //     LEDSubsystem.setBothChannelBrightnesses(ledBrightness);
        //     lastLEDBrightness = ledBrightness;
        // }

        // if (limelightPowerRelayToggle != lastLimelightPowerRelayState) {
        //     vision.togglePowerRelay();
        //     lastLimelightPowerRelayState = limelightPowerRelayToggle;
        // }
        
        // if (selectedButtonBindingsProfile != lastSelectedButtonBindingsProfile && selectedButtonBindingsProfile != null) {
        //     robotContainer.assignButtonBindings(selectedButtonBindingsProfile);
        //     lastSelectedButtonBindingsProfile = selectedButtonBindingsProfile;
        // }

        // if (isDrivetrainDead != lastIsDrivetrainDead) {
        //     if (isDrivetrainDead) {
        //         LEDSubsystem.getInstance().setAlertLEDStatusMode(LEDAlertStatusMode.LIGHT_SHOW);
        //     } else {
        //         LEDSubsystem.getInstance().clearAlertStatusMode();
        //     }
        //     lastIsDrivetrainDead = isDrivetrainDead;
        // }
    }

    public boolean getIsLimelightDead() {
        return isLimelightDead;
    }

    public Autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public DriveMode getSelectedDriveMode() {
        return driveModeSelector.getSelected();
    }

    public double getShooterVelocityBoost() {
        return SmartDashboard.getNumber("Shooter Velocity Boost Pct", 0);
    }

    public double getOneBallDelay() {
        return SmartDashboard.getNumber("One Ball Delay", 0);
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

    public LEDStatusMode getSelectedLEDStatusMode() {
        return ledStatusModeSelector.getSelected();
    }

    // Didn't work
    // public void resetSelectedAuto() {
    //     SendableChooser<Autos> autoSelector1 = new SendableChooser<Autos>();
    //     autoSelector1.setDefaultOption(Autos.NONE_SELECTED.name, Autos.NONE_SELECTED);
    //     SmartDashboard.putData("Autos", autoSelector1);
    //     SmartDashboard.putData("Autos", autoSelector);
    // }

    // public ButtonBindingsProfile getSelectButtonBindingsProfile() {
    //     return buttonBindingsProfileSelector.getSelected();
    // }

    public enum Autos {
        NONE_SELECTED("NO AUTO SELECTED", "NO AUTO SELECTED"),
        ONE_BALL("*TUNED* One Ball", "Any Starting Location Facing Towards Hub"),
        ONE_BALL_CUSTOM_DELAY("*TUNED* One Ball with Custom Delay", "Any Starting Location Facing Towards Hub"),
        AUTO_TESTING("Auto Testing", "Auto Testing"),
        SIMPLE_3_BALL("*TUNED* Simple 3 Ball", "Far Right Start (A) Facing Towards Hub"),
        LEFT_2_BALL_1_DEFENSE("*TUNED* 2 Ball and 1 Defence", "Far Left Start (D) Facing Away From Hub"),
        MIDDLE_LEFT_HANGER_4_BALL("*TUNED* Middle Left Hanger 4 Ball", "Middle Left Start (C) Facing Towards Hub"),
        MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE("*TUNED* Terminal 3 Ball", "Middle Left Start (C) Facing Towards Hub"),
        RIGHT_FIVE_BALL("*TUNED* Original Right Five Ball Auto", "Far Right Start (A2) Facing Towards Hub"),
        FAST_RIGHT_5_BALL_3("*TUNED* 10k Right 5 Ball Auto 3", "Far Right Start (A2) Facing Towards From Hub"),
        FAST_RIGHT_FIVE_BALL_4("*TUNED* Right Five Ball Auto 4 - Faster", "Far Right Start (A2) Facing Towards From Hub"),
        MIDDLE_LEFT_4_BALL_DEFENSE("*TUNED* Middle Left 4 Ball", "Middle Left Start (C) Facing Towards Hub"),
        RIGHT_2_BALL_DEFENSE("*TUNED* Right 2 Ball", "Far Right Start (A2) Facing Away From Hub"),
        LEFT_2_BALL_2_DEFENSE_2("*TUNED* Left 2 Ball 2 Defense 2", "Far Left Start (D) Facing Away From Hub"),
        MIDDLE_RIGHT_4_BALL("_TESTING_ Middle Right 4 Ball", "Middle Right Start (B) Facing Away From Hub"),
        LEFT_2_BALL_2_DEFENSE("_TESTING_ Left 2 Ball 2 Defense", "Far Left Start (D) Facing Away From Hub"),
        //THREE_BALL_DRIVE_AND_SHOOT("3 Ball Drive and Shoot - Far Right Start (A) Facing Away From Hub"),
        //LEFT_TERMINAL_3_BALL("3 Ball Including Terminal Cargo - Far Left Start (D) Facing Away Hub"),
        //MIDDLE_LEFT_TERMINAL_DEFENSE("2 Ball Terminal And Defense - Middle Left Start (C) Facing Towards Hub"),
        // RIGHT_MIDDLE_5_BALL_1_DEFENSE("_TUNING_ Right Middle 5 Ball 1 Defense", "Right Middle Start (B) Facing Away From Hub"),
        // RIGHT_FIVE_BALL_2("-INITIAL TESTING- Right Five Ball Auto 2 - Drive And Shoot", "Far Right Start (A2) Facing Away From Hub"),
        ;

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

    private void clearTimer() {
        if(timer != null) {
            timer.stop();
            timer = null;
        }
    }
}
