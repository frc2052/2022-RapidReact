package frc.robot.auto;

// A version of Simple3BallAuto to be used and abused for testing and learning !

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class AutoTesting extends AutoBase {

    /**
     * Position A Start (Far Right parallel with outer Tarmac line) Facing Inward Towards Hub.
     * Simple auto to score 3 alliance cargo.
     * @param drivetrain
     * @param vision
     */
    public AutoTesting(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem grassHopper, IndexerSubsystem indexer) {
        super(drivetrain, vision);
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

        IntakeArmOutCommand intakeArmOutCommand = new IntakeArmOutCommand(intake, indexer, grassHopper);
        IntakeArmOutCommand intakeArmOutCommand2 = new IntakeArmOutCommand(intake, indexer, grassHopper);
        IntakeArmOutCommand intakeArmOutCommand3 = new IntakeArmOutCommand(intake, indexer, grassHopper);
        IntakeArmInCommand intakeArmInCommand = new IntakeArmInCommand(intake, indexer, grassHopper);

        ShootCommand shoot1CargoCommand = new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, grassHopper, vision);
        ShootCommand shoot2CargoCommand = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, grassHopper, vision);

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, intakeArmOutCommand);
        ParallelDeadlineGroup intakeBall2 = new ParallelDeadlineGroup(driveToBall2, intakeArmOutCommand2);
        ParallelDeadlineGroup returnToShoot = new ParallelDeadlineGroup(driveToShoot, intakeArmOutCommand3);

        this.addCommands(autoAim);
        this.addCommands(shoot1CargoCommand.withTimeout(1.5));
        this.addCommands(turnToBall1);
        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_INTAKE_ON));
        this.addCommands(intakeBall1); // Drives to the closest ball to the robot
        this.addCommands(intakeBall2); // Drives and rotates to the second ball near the Tarmac
        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_DEFAULT));
        this.addCommands(returnToShoot); // Drives and rotates to position to shoot ball into upper hub
        this.addCommands(intakeArmInCommand);
        this.addCommands(autoAim2);      // Turns on an uses the Limelight to adjust it's aiming position to the center of the target
        this.addCommands(shoot2CargoCommand.withTimeout(3));

        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
