package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.commands.climber.ClimberArmsBackCommand;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HookClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class Simple3BallAuto extends AutoBase {

     /**
     * Position A Start (Far Right parallel with outer Tarmac line)
     * Simple auto to score 3 alliance cargo.
     * @param drivetrain
     * @param vision
     */
    public Simple3BallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem hopper, HookClimberSubsystem climber) {
        super(drivetrain, vision, shooter, intake, hopper, indexer, climber);
//         vision.setLED(LEDMode.OFF);
        
//         Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
//         Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(0));
        
//         Pose2d start2 = new Pose2d(Units.inchesToMeters(50), 0, Rotation2d.fromDegrees(-130));
//         Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-96), Rotation2d.fromDegrees(-130)); //wheels should be pointing 90 degrees from straight ahead at end of path
//         Supplier<Rotation2d> aimNeg130DegreesRight = () -> { return Rotation2d.fromDegrees(-130); };

//         Pose2d shootPos = new Pose2d(Units.inchesToMeters(24), Units.inchesToMeters(-60), Rotation2d.fromDegrees(-45)); //wheels should be pointing 90 degrees from straight ahead at end of path
//         Supplier<Rotation2d> aimNeg45DegreesRight = () -> { return Rotation2d.fromDegrees(-45); };

//         SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, ball1Pos);
//         TurnInPlaceCommand turnToBall1 = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(179));
//         SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, start2, ball2Pos, aimNeg130DegreesRight);
//         SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, aimNeg45DegreesRight);
//         VisionTurnInPlaceCommand autoAim = new VisionTurnInPlaceCommand(drivetrain, vision);

// ;

//         ShootCommand shoot1CargoCommand = new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, vision);
//         ShootCommand shoot2CargoCommand = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision);

//         ParallelDeadlineGroup intakeBall1 =  new ParallelDeadlineGroup(driveToBall1, super.newIntakeArmOutCommand());
//         ParallelCommandGroup driveAndIntakeUp = new ParallelCommandGroup(driveToShoot, super.newIntakeArmInCommand());

//         this.addCommands(shoot1CargoCommand);
//         this.addCommands(driveToBall1); // Drives to the closest ball to the robot
//         this.addCommands(turnToBall1);
//         this.addCommands(intakeBall1);
//         this.addCommands(driveToBall2); // Drives and rotates to the second ball near the Tarmac
//         this.addCommands(driveAndIntakeUp); // Drives and rotates to position to shoot ball into upper hub, and puts up intake
//         this.addCommands(autoAim);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target
//         this.addCommands(shoot2CargoCommand);

//         this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
//         this.andThen(() -> drivetrain.stop(), drivetrain);

        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(-175));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(-50), Units.inchesToMeters(-6), Rotation2d.fromDegrees(-175));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(-16), Units.inchesToMeters(98), Rotation2d.fromDegrees(70)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(6), Units.inchesToMeters(50), Rotation2d.fromDegrees(45)); //wheels should be pointing 90 degrees from straight ahead at end of path

        AutoTrajectoryConfig testingCongig = super.createTrajectoryConfig(2, 1.5, 1, 3, 2);
        AutoTrajectoryConfig path2Config = super.createTrajectoryConfig(4, 3, 1, 5, 2);

        TurnInPlaceCommand turnToBall1 = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(175));
        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(testingCongig, startPos, ball1Pos, super.createRotationAngle(-170));
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(path2Config, super.getLastEndingPosCreated(60), ball2Pos, super.createRotationAngle(60));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(path2Config, super.getLastEndingPosCreated(), shootPos, super.createHubTrackingSupplier(-45));
        VisionTurnInPlaceCommand autoAim = new VisionTurnInPlaceCommand(drivetrain, vision);
        VisionTurnInPlaceCommand autoAim2 = new VisionTurnInPlaceCommand(drivetrain, vision);

        ClimberArmsBackCommand climberBack = new ClimberArmsBackCommand(climber);
        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveToBall2, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup returnToShoot = new ParallelDeadlineGroup(driveToShoot, super.newIntakeArmOutCommand());

        this.addCommands(climberBack);
        this.addCommands(autoAim);
        this.addCommands(super.newShoot1Command().withTimeout(1.5));
        this.addCommands(turnToBall1);
    //    this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_INTAKE_ON));
        this.addCommands(intakeBall1); // Drives and intakes the closest ball to the robot
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
    //    this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_DEFAULT));
        this.addCommands(returnToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(super.newIntakeArmInCommand());
        this.addCommands(autoAim2);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target
        this.addCommands(super.newShootAllCommand().withTimeout(3));

    //    this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
