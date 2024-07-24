// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.TeleopIntakeCommand;
import frc.robot.commands.intake.TeleopOnlyIntakeCommand;
import frc.robot.commands.intake.OuttakeCommand.OuttakeMode;
import frc.robot.commands.shooter.ShooterIndexingCommand.ShootMode;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ThrottleShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FiringAngle;

import frc.robot.util.vision.VisionCalculator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private boolean isShooting;

  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;
  private IntakeSubsystem intake;
  private HopperSubsystem hopper;
  public final DrivetrainSubsystem drivetrain;

  private final Joystick joystick = new Joystick(0);

  public RobotContainer() {
    shooter = new ShooterSubsystem();
    
    drivetrain = new DrivetrainSubsystem();
    drivetrain.setDefaultCommand(
      new DriveCommand(
          // Forward velocity supplier.
          joystick::getY,
          // Sideways velocity supplier.
          joystick::getX,
          // Rotation velocity supplier.
          joystick::getZ,
          drivetrain
      )
    );

    indexer = new IndexerSubsystem();
    intake = new IntakeSubsystem();
    hopper = new HopperSubsystem();
    new PneumaticsSubsystem();
    
    // Configure the trigger bindings
    configureButtonBindings();
  }

  JoystickButton intakeButton;
  JoystickButton outtakeOneButton;
  JoystickButton outtakeAllButton;
  JoystickButton shoot3ftButton;
  JoystickButton shoot7ftButton;
  JoystickButton adjustableShootButton;
  JoystickButton angle1Button;
  JoystickButton angle2Button;

  private void configureButtonBindings() {

    // --- Intake Buttons Bindings ---
    intakeButton = new JoystickButton(joystick, 1);
    outtakeOneButton = new JoystickButton(joystick, 7);
    outtakeAllButton = new JoystickButton(joystick, 8);

    // --- Shooter Buttons Bindings ---
    shoot3ftButton = new JoystickButton(joystick, 5);
    shoot7ftButton = new JoystickButton(joystick, 3);
    adjustableShootButton = new JoystickButton(joystick, 4);

    angle1Button = new JoystickButton(joystick, 9);
    angle2Button = new JoystickButton(joystick, 10);

    // --- Intake Commands ---
    Command teleopIntakeOnCommand =
      new ConditionalCommand( // Makes sure the intake doesn't stop the shooter because both of them require the hopper, by checking if shoot button is pressed and using the OnlyIntakeCommand instead if it is.
        new TeleopOnlyIntakeCommand(intake, indexer), 
        new TeleopIntakeCommand(intake, indexer, hopper),
        () -> isShooting
      );
    Command outtake1Command = new OuttakeCommand(OuttakeMode.ONE_BALL, intake, hopper, indexer);
    Command outtakeAllCommand = new OuttakeCommand(OuttakeMode.ALL_BALLS, intake, hopper, indexer);

    // --- Shooter Commands ---
    Command threeFootShootCommand =
      new ShootCommand(
        ShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper,
        FiringAngle.ANGLE_1,
        VisionCalculator.getInstance().getShooterConfig(3*12, FiringAngle.ANGLE_1)
      );
    Command sevenFootShootCommand =
      new ShootCommand(
        ShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper,
        FiringAngle.ANGLE_2,
        VisionCalculator.getInstance().getShooterConfig(7*12, FiringAngle.ANGLE_2)
      );

    Command adjustableShootCommand = 
      new ThrottleShootCommand(
        ShootMode.SHOOT_ALL, 
        shooter, 
        indexer, 
        hopper, 
        joystick::getThrottle
      );
    
    // --- Intake Button Command Bindings ---
    intakeButton.whileTrue(teleopIntakeOnCommand);
    outtakeOneButton.whileTrue(outtake1Command);
    outtakeAllButton.whileTrue(outtakeAllCommand);

    // --- Shooter Button Command Bindings ---
    shoot3ftButton.whileTrue(threeFootShootCommand);
    shoot7ftButton.whileTrue(sevenFootShootCommand);
    adjustableShootButton.whileTrue(adjustableShootCommand);

    angle1Button.onTrue(new InstantCommand(()-> shooter.setShootAngle1()));
    angle2Button.onTrue(new InstantCommand(()-> shooter.setShootAngle2()));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void evaluateIsShooting() {
    isShooting = shoot3ftButton.getAsBoolean() || shoot3ftButton.getAsBoolean();
  }

  public boolean getIsShooting() {
    return isShooting;
  }
}
