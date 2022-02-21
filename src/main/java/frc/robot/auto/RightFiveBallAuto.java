package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
public class RightFiveBallAuto extends AutoBase {
    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line) Facing Towards the Hub.
     * 1. Immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * 2. Drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * 3. Drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * 4. Then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * @param drivetrain
     * @param vision
     */
    public RightFiveBallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, HopperSubsystem grassHopper) {
        super(drivetrain, vision, shooter, intake, grassHopper, indexer);
        vision.setLED(LEDMode.ON);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(-68), 0, Rotation2d.fromDegrees(0));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(-12), Units.inchesToMeters(96), Rotation2d.fromDegrees(-50)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(-18), Units.inchesToMeters(40), Rotation2d.fromDegrees(45)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d terminalBallMidPointPos = new Pose2d(0, Units.inchesToMeters(210), Rotation2d.fromDegrees(120));
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(-30), Units.inchesToMeters(210), Rotation2d.fromDegrees(-110));

        AutoTrajectoryConfig realignTrajectoryConfig = super.createTrajectoryConfig(1, 0.5, 1, 3, 1);

        VisionTurnInPlaceCommand aimAtHub = new VisionTurnInPlaceCommand(drivetrain, vision);
        TurnInPlaceCommand turnAround = new TurnInPlaceCommand(drivetrain, Rotation2d.fromDegrees(170).minus(drivetrain.getPose().getRotation()));
        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, ball1Pos);
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(50), ball2Pos, super.createRotationAngle(50));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, super.createHubTrackingSupplier(-45));
        SwerveControllerCommand driveToTerminalMidPoint = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withEndVelocity(1), super.getLastEndingPosCreated(), terminalBallMidPointPos, super.createRotationAngle(120));
        SwerveControllerCommand driveToTerminalBalls = super.createSwerveTrajectoryCommand(realignTrajectoryConfig.withStartVelocity(1), super.getLastEndingPosCreated(-110), terminalBallPos, super.createRotationAngle(110));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(66), shootPos, createHubTrackingSupplier(-45));
        

        ShootCommand shoot1CargoCommand = new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, grassHopper, vision); // Adjust when ready to shoot either 1 or 2 cargo individually
        ShootCommand shoot2CargoCommand = new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, grassHopper, vision);

        ParallelDeadlineGroup intakeBall1 = new ParallelDeadlineGroup(driveToBall1, super.newIntakeArmOutCommand());
        ParallelDeadlineGroup shoot2Balls = new ParallelDeadlineGroup(driveToShoot, shoot2CargoCommand, super.newIntakeArmInCommand());
        ParallelDeadlineGroup intakeTerminalBall = new ParallelDeadlineGroup(driveToTerminalBalls, super.newIntakeArmOutCommand());
        ParallelCommandGroup driveBackAndShoot2 = new ParallelCommandGroup(driveBackToShoot, super.newIntakeArmInCommand());

        this.addCommands(aimAtHub);
        this.addCommands(shoot1CargoCommand.withTimeout(3));
        this.addCommands(turnAround);
        this.addCommands(intakeBall1); // Drives to the closest ball to the robot
        this.addCommands(driveToBall2); // Drives and rotates to the second ball near the Tarmac
        this.addCommands(shoot2Balls); // Drives and rotates to position to shoot ball into upper hub\
        this.addCommands(driveToTerminalMidPoint);
        this.addCommands(intakeTerminalBall);
        this.addCommands(driveBackAndShoot2);
        this.addCommands(shoot2CargoCommand.withTimeout(3));
        this.andThen(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.AUTONOMOUS_FINISHED));
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}