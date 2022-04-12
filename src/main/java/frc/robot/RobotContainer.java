// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.testing.AutoTesting;
import frc.robot.auto.testing.Left2Ball2DefenseAuto;
import frc.robot.auto.tuned.LeftDefenseAuto;
import frc.robot.auto.testing.LeftTerminal3Cargo;
import frc.robot.auto.tuned.MiddleLeft3BallTerminalDefenseAuto;
import frc.robot.auto.tuned.MiddleLeft4BallTerminalDefenseAuto;
import frc.robot.auto.testing.MiddleRight5BallDefenseAuto;
import frc.robot.auto.testing.MiddleRightTerminal3CargoAuto;
import frc.robot.auto.testing.Right5BallAuto2;
import frc.robot.auto.tuned.OneBallAuto;
import frc.robot.auto.tuned.FastRight5BallAuto3;
import frc.robot.auto.tuned.RightFiveBallAuto;
import frc.robot.auto.tuned.Simple3BallAuto;
import frc.robot.auto.testing.ThreeballDriveAndShoot;

import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.PIDVisionDriveCommand;
import frc.robot.commands.drive.ProfiledPIDTurnInPlaceCommand;
import frc.robot.commands.drive.VisionDriveCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.ToggleClimberSolenoidCommand;
import frc.robot.commands.climber.ZeroClimberEncoderCommand;
import frc.robot.commands.climber.autoclimbs.MidToHighAutoClimbCommand;
import frc.robot.commands.climber.autoclimbs.HighToTraversalAutoClimbCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.intake.IntakeArmToggleCommand;
import frc.robot.commands.intake.IntakeHopperRunCommand;
import frc.robot.commands.intake.IntakeReverseCommand;
import frc.robot.commands.intake.OnlyIntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.TeleopIntakeCommand;
import frc.robot.commands.intake.OuttakeCommand.OuttakeMode;
import frc.robot.commands.shooter.TuneShooterCommand;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootLowCommand;
import frc.robot.subsystems.DashboardControlsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.UsbCameraSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DashboardControlsSubsystem.ButtonBindingsProfile;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.buttonbindings.profiles.Default;
import frc.robot.util.buttonbindings.profiles.SoloDriver;
import frc.robot.util.vision.VisionCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DrivetrainSubsystem drivetrain;
  private VisionSubsystem vision;
  private DashboardControlsSubsystem dashboardControls;
  private ShooterSubsystem shooter;
  private UsbCameraSubsystem intakeCamera;
  private TargetingSubsystem targeting;
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;
  private HopperSubsystem hopper;
  private PneumaticsSubsystem pneumatics;
  private HookClimberSubsystem climber;

  private final Joystick driveJoystick = new Joystick(0);
  private final Joystick turnJoystick = new Joystick(1);
  private final Joystick secondaryPannel = new Joystick(2);

  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(2);

  private boolean initComplete = false;
  // private boolean competitionMode = true;
  private boolean isShooting = false;
  private boolean lastIsShooting = false;

  private Command autonomousCommand;

  private JoystickButton tempFieldCentricButton;
  private JoystickButton climberOverrideButton;
  private JoystickButton shootLowGoalButton;
  private JoystickButton shootSingleButton;
  private JoystickButton shootAllButton;
  private JoystickButton tuneShooterButton;
  private JoystickButton nonVisionShootAllButton;
  private JoystickButton nonVisionShootAllButton2;
  private JoystickButton nonVisionShootAllButton3;
  private JoystickButton eject1Button;
  private JoystickButton ejectAllButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // pixySub.setDefaultCommand(new PixyCamManualDriveCommand(pixySub));

    vision = new VisionSubsystem();
    shooter = new ShooterSubsystem();

    DashboardControlsSubsystem.init(vision, this, shooter);
    dashboardControls = DashboardControlsSubsystem.getInstance();
    dashboardControls.addSelectorsToSmartDashboard();

    drivetrain = new DrivetrainSubsystem();

    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(
        drivetrain,
        () -> -modifyAxis(driveJoystick.getY(), xLimiter),
        () -> -modifyAxis(driveJoystick.getX(), yLimiter),
        () -> -modifyAxis(turnJoystick.getX(), turnLimiter),
        () -> { return tempFieldCentricButton.get(); }
		  )
    );

    TargetingSubsystem.init(vision, drivetrain, shooter);
    targeting = TargetingSubsystem.getInstance();

    init();
  }

  public void init() {
    if (!initComplete) {
        initComplete = true;

    // //The following subsystems have a dependency on CAN

        // shooter = new ShooterSubsystem();
       // intakeCamera = new UsbCameraSubsystem();  
        indexer = new IndexerSubsystem();
        intake = new IntakeSubsystem();
        hopper = new HopperSubsystem();
        pneumatics = new PneumaticsSubsystem();
        climber = new HookClimberSubsystem();
        //LEDSubsystem.getInstance();

        configureButtonBindings();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drivetrain Button Bindings
    JoystickButton resetGyroButton = new JoystickButton(driveJoystick, 11);
    tempFieldCentricButton = new JoystickButton(turnJoystick, 2);

    // Intake Buttons Bindings
    JoystickButton intakeArmToggleButton = new JoystickButton(secondaryPannel, 1);
    JoystickButton intakeInButton = new JoystickButton(secondaryPannel, 7);
    JoystickButton intakeInButton2 = new JoystickButton(secondaryPannel, 9);
    JoystickButton intakeReverseButton = new JoystickButton(secondaryPannel, 6);
    Button pannelOuttakePreloadedCargoButton = new Button(() -> (secondaryPannel.getY() >= 0.5)); // 'Fake' buttons because some of the secondary pannel buttons are connected to joystick x and y outputs
    Button pannelOuttakeAllCargoButton = new Button(() -> (secondaryPannel.getY() <= -0.5));
    Button intakeArmInButton = new Button(() -> (secondaryPannel.getX() >= 0.5));
    Button intakeArmOutButton = new Button(() -> (secondaryPannel.getX() <= -0.5));
    JoystickButton rejectBothCargoButton = new JoystickButton(turnJoystick, 9);
    JoystickButton driverIntakeButton = new JoystickButton(driveJoystick, 2);

    // Shooter Buttons Bindings
    shootSingleButton = new JoystickButton(turnJoystick, 1);
    shootAllButton = new JoystickButton(driveJoystick, 1);
    tuneShooterButton = new JoystickButton(driveJoystick, 8);
    shootLowGoalButton = new JoystickButton(turnJoystick, 4);
    nonVisionShootAllButton = new JoystickButton(driveJoystick, 4);
    nonVisionShootAllButton2 = new JoystickButton(driveJoystick, 3);
    nonVisionShootAllButton3 = new JoystickButton(driveJoystick, 5);
    eject1Button = new JoystickButton(turnJoystick, 5);
    ejectAllButton = new JoystickButton(turnJoystick, 3);
    
    // Climber Buttons Bindings
    JoystickButton extendClimberButton = new JoystickButton(secondaryPannel, 5);
    JoystickButton retractClimberButton = new JoystickButton(secondaryPannel, 3);
    JoystickButton climberSolenoidToggleButton = new JoystickButton(secondaryPannel, 4);
    JoystickButton climberUnlockButton = new JoystickButton(secondaryPannel, 11);
    JoystickButton climberLockButton = new JoystickButton(secondaryPannel, 12);
    climberOverrideButton = new JoystickButton(secondaryPannel, 10);
    // commandInterruptButton = new JoystickButton(secondaryPannel, 10); // TEMP BUTTON PORT
    JoystickButton traversalAutoClimbButton = new JoystickButton(secondaryPannel, 2);
    JoystickButton highAutoClimbButton = new JoystickButton(secondaryPannel, 8);

    // In Testing
    // JoystickButton testingButton = new JoystickButton(turnJoystick, 10);

    Command shootSingleCommand =
      new ParallelCommandGroup(
        new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, vision, drivetrain),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrain,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter),
          () -> -modifyAxis(driveJoystick.getX(), yLimiter),
          vision,
          shooter
        )
      );

    Command visionShootAllCommand =
      new ParallelCommandGroup(
        // new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision),
        //new ConditionalCommand(
        //  new NonVisionShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_1, VisionCalculator.getInstance().getShooterConfig(3 * 12, FiringAngle.ANGLE_1)), 
          new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision, drivetrain), 
        //  dashboardControls::getIsLimelightDead
       // ),
        new VisionDriveCommand( // Overrides the DefualtDriveCommand and uses VisionDriveCommand when the trigger on the turnJoystick is held.
          drivetrain,
          () -> -modifyAxis(driveJoystick.getY(), xLimiter),
          () -> -modifyAxis(driveJoystick.getX(), yLimiter),
          vision,
          shooter
        )
      );
    Command resetGyroCommand = new InstantCommand(() -> { this.resetGyro(); });
    Command intakeToggleCommand = new IntakeArmToggleCommand(intake);
    // Command intakeOnCommand =
    //   new ConditionalCommand( // Makes sure the intake doesn't stop the shooter because both of them require the hopper, by checking if shoot button is pressed and using the OnlyIntakeCommand instead if it is.
    //     new OnlyIntakeCommand(intake, indexer), 
    //     new IntakeHopperRunCommand(intake, indexer, hopper),
    //     () -> isShooting
    // );//.withInterrupt(() -> getIsShooting());
    Command newIntakeOnCommand = new TeleopIntakeCommand(intake, indexer, hopper).withInterrupt(() -> isShooting);
    Command intakeOnCommand = new IntakeHopperRunCommand(intake, indexer, hopper).withInterrupt(() -> { return isShooting || !intakeInButton.get() || driverIntakeButton.get(); } );
    Command intakeOnCommand2 = new IntakeHopperRunCommand(intake, indexer, hopper).withInterrupt(() -> { return isShooting || !intakeInButton2.get() || intakeInButton.get() || driverIntakeButton.get(); } );
    Command intakeReverseCommand = new IntakeReverseCommand(intake, hopper);
    Command tuneShooterCommand = new TuneShooterCommand(shooter, indexer, intake, hopper);
    Command nonVisionShootLowGoalCommand = new ShootLowCommand(shooter, indexer);
    // Command lineupShootCommand = new NonVisionShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_2, VisionCalculator.getInstance().getShooterConfig(7 * 12, FiringAngle.ANGLE_2));
    Command nonVisionShootAllCommand =
      new NonVisionShootCommand(
        ShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper,
        FiringAngle.ANGLE_1,
        VisionCalculator.getInstance().getShooterConfig(3*12, FiringAngle.ANGLE_1)
      );

    Command nonVisionShootAllCommand2 =
      new NonVisionShootCommand(
        ShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper,
        FiringAngle.ANGLE_2,
        VisionCalculator.getInstance().getShooterConfig(7*12, FiringAngle.ANGLE_2)
      );

    Command nonVisionShootAllCommand3 =
      new NonVisionShootCommand(
        ShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper,
        FiringAngle.ANGLE_2,
        VisionCalculator.getInstance().getShooterConfig(14*12, FiringAngle.ANGLE_2)
      );

    Command climberExtendCommand = new ExtendClimberCommand(climber, () -> climberOverrideButton.get());
    Command climberRetractCommand = new RetractClimberCommand(climber, () -> climberOverrideButton.get());
    Command toggleClimberAngleCommand = new ToggleClimberSolenoidCommand(climber);
    Command unlockClimberCommand = new InstantCommand(() -> { climber.unlock(); });
    Command lockClimberCommand = new InstantCommand(() -> { climber.lock(); });
    Command intakeArmInCommand = new IntakeArmInCommand(intake);
    Command intakeArmOutCommand = new IntakeArmOutCommand(intake);

    Command outtakeAllCargoCommand = new OuttakeCommand(OuttakeMode.ALL_BALLS, intake, hopper, indexer).withInterrupt(() -> !pannelOuttakeAllCargoButton.get());
    Command outtakePreloadedCargoCommand = new OuttakeCommand(OuttakeMode.ONE_BALL, intake, hopper, indexer).withInterrupt(() -> !pannelOuttakePreloadedCargoButton.get());
    Command traversalAutoClimbCommand = new HighToTraversalAutoClimbCommand(climber, drivetrain).withInterrupt(() -> !traversalAutoClimbButton.get());
    Command highAutoClimbCommand = new MidToHighAutoClimbCommand(climber, drivetrain).withInterrupt(() -> !highAutoClimbButton.get());
    Command eject1Command = new NonVisionShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, null, 4000, 4000);
    Command eject2Command = new NonVisionShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, null, 4000, 4000, true);

    // pixyDriveCommandSwitch.whenHeld(
    //   new PixyCamDriveCommand(
    //     drivetrain,
    //     pixyCamSubsystem,
    //     () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     dashboardControlsSubsystem
    //   )
    // );

    // Drivetrain Button Command Bindings
    resetGyroButton.whenPressed(resetGyroCommand);
    tempFieldCentricButton.whenPressed(() -> { this.resetGyro(); });
    
    // Intake Button Command Bindings
    intakeArmToggleButton.whenPressed(intakeToggleCommand);
    intakeInButton.whenPressed(intakeOnCommand);
    intakeInButton2.whenPressed(intakeOnCommand2);
    intakeReverseButton.whileHeld(intakeReverseCommand);
    intakeArmOutButton.whenPressed(intakeArmOutCommand);
    intakeArmInButton.whenPressed(intakeArmInCommand);
    pannelOuttakeAllCargoButton.whenPressed(outtakeAllCargoCommand);
    pannelOuttakePreloadedCargoButton.whenPressed(outtakePreloadedCargoCommand);
    driverIntakeButton.whenHeld(newIntakeOnCommand);

    // Shooter Button Command Bindings
    shootSingleButton.whileHeld(shootSingleCommand);
    // shootSingleButton.whenPressed(() -> isShooting = true);
    // shootAllButton.whenReleased(() -> isShooting = false);
    shootAllButton.whileHeld(visionShootAllCommand);
    // shootAllButton.whenPressed(() -> isShooting = true);
    // shootAllButton.whenReleased(() -> isShooting = false);
    tuneShooterButton.whileHeld(tuneShooterCommand);
    shootLowGoalButton.whileHeld(nonVisionShootLowGoalCommand);
    nonVisionShootAllButton.whileHeld(nonVisionShootAllCommand);
    // nonVisionShootAllButton.whenPressed(() -> isShooting = true);
    // nonVisionShootAllButton.whenReleased(() -> isShooting = false);
    nonVisionShootAllButton2.whileHeld(nonVisionShootAllCommand2);
    // nonVisionShootAllButton2.whenPressed(() -> isShooting = true);
    // nonVisionShootAllButton2.whenReleased(() -> isShooting = false);
    nonVisionShootAllButton3.whileHeld(nonVisionShootAllCommand3);
    // nonVisionShootAllButton3.whenPressed(() -> isShooting = true);
    // nonVisionShootAllButton3.whenReleased(() -> isShooting = false);
      
    // Climber Button Command Bindings
    extendClimberButton.whileHeld(climberExtendCommand);
    retractClimberButton.whileHeld(climberRetractCommand);
    climberSolenoidToggleButton.whenPressed(toggleClimberAngleCommand);
    climberUnlockButton.whenPressed(unlockClimberCommand);
    climberLockButton.whenPressed(lockClimberCommand);
    traversalAutoClimbButton.whenPressed(traversalAutoClimbCommand);
    highAutoClimbButton.whenPressed(highAutoClimbCommand);

    rejectBothCargoButton.whenHeld(outtakeAllCargoCommand);
    eject1Button.whenHeld(eject1Command);
    ejectAllButton.whenHeld(eject2Command);

    // SmartDashboard Command Buttons
    SmartDashboard.putData("Zero Climber Encoder", new ZeroClimberEncoderCommand(climber));
    SmartDashboard.putData("Zero Gyroscope", new InstantCommand(() -> this.resetGyro()));
    // SmartDashboard.putData("Reset selected Auto Test", new InstantCommand(() -> dashboardControls.resetSelectedAuto()));

    // Testing Zone
    // Command testCommand = new OnlyIntakeCommand(intake, indexer);
    //testingButton.whenPressed(new ProfiledPIDTurnInPlaceCommand(drivetrain, () -> { return Rotation2d.fromDegrees(180); }));
        //   Command testCommand = 
    //     new PIDVisionDriveCommand(
    //         drivetrain, 
    //         () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
    //         () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
    //         vision
    //     );
    //     Command testCommand =
    //   new ParallelCommandGroup(
    //     // new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision),
    //     new ConditionalCommand(
    //       new NonVisionShootCommand(NonVisionShootMode.SHOOT_ALL, shooter, indexer, hopper, FiringAngle.ANGLE_1, VisionCalculator.getInstance().getShooterConfig(3 * 12, FiringAngle.ANGLE_1)), 
    //       new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision, drivetrain), 
    //       dashboardControls::getIsLimelightDead
    //     ),
    //     new PIDVisionDriveCommand(
    //         drivetrain, 
    //         () -> -modifyAxis(driveJoystick.getY(), xLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
    //         () -> -modifyAxis(driveJoystick.getX(), yLimiter) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
    //         vision
    //   )
    //   );

    // testingButton.whenHeld(testCommand);

    // if (!competitionMode) { // Makes sure not to do any processing of this if we're at compeition because we don't need it
    //   ButtonCommands.VISION_SHOOT_ALL.setCommand(visionShootAllCommand);
    //   ButtonCommands.NONVISION_SHOOT_LOW_GOAL.setCommand(nonVisionShootLowGoalCommand);
    //   ButtonCommands.NONVISION_SHOOT_ALL.setCommand(nonVisionShootAllCommand);
    //   ButtonCommands.MANUAL_SHOOT.setCommand(tuneShooterCommand);
    //   // ButtonCommands.LINEUP_SHOOT.setCommand(lineupShootCommand);
    //   ButtonCommands.TUNE_SHOOTER.setCommand(tuneShooterCommand);
    //   ButtonCommands.RESET_GYRO.setCommand(resetGyroCommand);
    //   ButtonCommands.VISION_SHOOT_SINGLE.setCommand(shootSingleCommand);
    //   ButtonCommands.CLIMBER_RETRACT.setCommand(climberRetractCommand);
    //   ButtonCommands.TOGGLE_CLIMBER_ANGLE.setCommand(toggleClimberAngleCommand);
    //   ButtonCommands.CLIMBER_EXTEND.setCommand(climberExtendCommand);
    //   ButtonCommands.INTAKE_TOGGLE.setCommand(intakeToggleCommand);
    //   ButtonCommands.INTAKE_REVERSE.setCommand(intakeReverseCommand);
    //   ButtonCommands.INTAKE_ON.setCommand(intakeOnCommand);
    //   ButtonCommands.CLIMBER_LIMIT_OVERRIDE.setCommand(null);
    //   ButtonCommands.UNLOCK_CLIMBER.setCommand(unlockClimberCommand);
    //   ButtonCommands.LOCK_CLIMBER.setCommand(lockClimberCommand);
    // }
  }

  // public void assignButtonBindings(ButtonBindingsProfile buttonBindingsProfile) {
  //     if (!competitionMode) {
  //         CommandScheduler.getInstance().clearButtons();

  //         switch (buttonBindingsProfile) {
  //             case DEFAULT:
  //               //   Default.configureButtons(driveJoystick, turnJoystick, secondaryPannel);
  //               configureButtonBindings();
  //                 break;
  //             case SOLO_DRIVER:
  //                 SoloDriver.configureButtons(driveJoystick, turnJoystick, secondaryPannel);
  //                 break;
  //             default:
  //                 System.err.println("REASSIGN BUTTON BINDINGS FELL THROUGH");
  //                 configureButtonBindings();
  //                 break;
  //         }
  //     }
  // }

  public boolean getIsShooting() {
    return isShooting;
  }

  public void evaluateIsShooting() {
      isShooting = shootAllButton.get() || shootSingleButton.get() || shootLowGoalButton.get() || eject1Button.get() || ejectAllButton.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public void initializeAutonomousCommand() {
    switch(dashboardControls.getSelectedAuto()) {
      // case AUTO_TESTING:
      //   autonomousCommand = new AutoTesting(drivetrain, vision, shooter, intake, hopper, indexer, climber);
      //   break;
      case ONE_BALL:
        autonomousCommand = new OneBallAuto(drivetrain, shooter, indexer, hopper, climber);
        break;
      case SIMPLE_3_BALL:
        autonomousCommand = new Simple3BallAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case LEFT_2_BALL_1_DEFENSE:
        autonomousCommand = new LeftDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case MIDDLE_LEFT_3_BALL_TERMINAL_DEFENSE:
        autonomousCommand = new MiddleLeft3BallTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case MIDDLE_RIGHT_TERMINAL_4_BALL:
        autonomousCommand = new MiddleLeft4BallTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case RIGHT_FIVE_BALL:
        autonomousCommand = new RightFiveBallAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      case FAST_RIGHT_FIVE_BALL_3:
        autonomousCommand = new FastRight5BallAuto3(drivetrain, vision, shooter, intake, indexer, hopper, climber);
        break;
      // case LEFT_2_BALL_2_DEFENSE:
      //   autonomousCommand = new Left2Ball2DefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      //   break;
      // case RIGHT_MIDDLE_5_BALL_1_DEFENSE:
      //   autonomousCommand = new MiddleRight5BallDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      //   break;
      // case RIGHT_FIVE_BALL_2:
      //   autonomousCommand = new Right5BallAuto2(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      //   break;
    //   case THREE_BALL_DRIVE_AND_SHOOT:
    //     return new ThreeballDriveAndShoot(drivetrain, vision, shooter, intake, hopper, indexer, climber);
    //   case LEFT_TERMINAL_3_BALL: 
    //     return new LeftTerminal3Cargo(drivetrain, vision, shooter, intake, indexer, hopper, climber);
    //   case MIDDLE_RIGHT_TERMINAL_3_BALL:
    //     return new MiddleRightTerminal3CargoAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
    //   case MIDDLE_LEFT_TERMINAL_DEFENSE:
    //     return new MiddleLeftTerminalDefenseAuto(drivetrain, vision, shooter, intake, indexer, hopper, climber);
      default:
        System.err.println("NO VALID AUTO SELECTED");
        break;
    }
  }

  // This code borrowed from the SwerverDriveSpecialist Sample code
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

//   public void setShooterPIDF() {
//     double p = SmartDashboard.getNumber("Shooter P", 0.115);
//     double i = SmartDashboard.getNumber("Shooter I", 0);
//     double d = SmartDashboard.getNumber("Shooter D", 0);
//     double f = SmartDashboard.getNumber("Shooter F", 0.048);

//     shooter.updatePIDF(p, i, d, f);
//   }

  // This code borrowed from the SwerverDriveSpecialist Sample code
  private static double modifyAxis(double value, SlewRateLimiter limiter) {
    // Deadband
    value = deadband(value, 0.05);
    // Square the axis for finer control at lower values
    value = limiter.calculate(Math.copySign(value * value, value));
    
    return value;
  }

  public void resetGyro() {
    drivetrain.zeroGyroscope();
  }

  public void resetRobot() {
    drivetrain.stop();
  }

  public enum ButtonCommands {
    VISION_SHOOT_ALL(null),
    VISION_SHOOT_SINGLE(null),
    NONVISION_SHOOT_LOW_GOAL(null),
    NONVISION_SHOOT_ALL(null),
    MANUAL_SHOOT(null),
    // LINEUP_SHOOT(null),
    TUNE_SHOOTER(null),
    RESET_GYRO(null),
    CLIMBER_RETRACT(null),
    TOGGLE_CLIMBER_ANGLE(null),
    CLIMBER_EXTEND(null),
    INTAKE_TOGGLE(null),
    INTAKE_IN(null),
    INTAKE_OUT(null),
    INTAKE_REVERSE(null),
    INTAKE_ON(null),
    INTAKE_REJECT_BOTH(null),
    CLIMBER_LIMIT_OVERRIDE(null),
    TRAVERSAL_AUTO_CLIMB(null),
    UNLOCK_CLIMBER(null),
    LOCK_CLIMBER(null);

    public Command command;

    private ButtonCommands(Command command) {
        this.command = command;
    }

    public void setCommand(Command command) {
        this.command = command;
    }
  }
}
