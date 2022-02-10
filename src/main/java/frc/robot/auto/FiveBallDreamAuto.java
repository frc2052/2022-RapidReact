package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class FiveBallDreamAuto extends AutoBase {

    /**
     * Position A Start (Far Right Parallel with Outer Tarmac Line).
     * Will immediatley shoot preloaded cargo, turn around, and intake closest alliance cargo.
     * Will then drive to and intake second closest alliance cargo, and return to a position to shoot both.
     * Will drive to terminal cargo and wait a second to try and make sure cargo rolled out of terminal is in'tooken'.
     * Will then drive back to location just outside tarmac but avoiding upper exit to score 1 or 2 cargo held.
     * @param drivetrain
     * @param vision
     */
    public FiveBallDreamAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);
        
        Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d ball1Pos = new Pose2d(Units.inchesToMeters(-68), 0, Rotation2d.fromDegrees(0));
        Pose2d ball2Pos = new Pose2d(Units.inchesToMeters(-12), Units.inchesToMeters(96), Rotation2d.fromDegrees(-50)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d shootPos = new Pose2d(Units.inchesToMeters(-18), Units.inchesToMeters(40), Rotation2d.fromDegrees(45)); //wheels should be pointing 90 degrees from straight ahead at end of path
        Pose2d terminalBallPos = new Pose2d(Units.inchesToMeters(-40), Units.inchesToMeters(230), Rotation2d.fromDegrees(-110));

        SwerveControllerCommand driveToBall1 = super.createSwerveTrajectoryCommand(super.fastTurnSlowDriveTrajectoryConfig, startPos, ball1Pos, super.createRotationAngle(170));
        SwerveControllerCommand driveToBall2 = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(-50), ball2Pos, super.createRotationAngle(-50));
        SwerveControllerCommand driveToShoot = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(), shootPos, super.createHubTrackingSupplier(45));
        SwerveControllerCommand driveToTerminalBalls = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, super.getLastEndingPosCreated(-110), terminalBallPos, super.createRotationAngle(-110));
        SwerveControllerCommand driveBackToShoot = super.createSwerveTrajectoryCommand(super.speedDriveTrajectoryConfig, super.getLastEndingPosCreated(66), shootPos, createHubTrackingSupplier(45));

        //TODO: extend intake and turn on intake motors
        this.addCommands(driveToBall1); // Drives to the closest ball to the robot
        //TOOD: shoot 2 balls
        //TODO: is there  a way to force the wheels to point 90 immediately before it starts to drive? 
        this.addCommands(driveToBall2); // Drives and rotates to the second ball near the Tarmac
        this.addCommands(driveToShoot); // Drives and rotates to position to shoot ball into upper hub\
        this.addCommands(driveToTerminalBalls);

        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}
