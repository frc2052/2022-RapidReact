package frc.robot.auto;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.VisionTurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;
public class LeftDefenseAuto extends AutoBase {

    /**
     * Position D Start (Far Left Parallel with Outer Tarmac Line).
     * First intakes closest alliance ball, then turns and reapproaches tarmac to score 2.
     * Then drives to and intakes closest opponent cargo, and turns and fires it into the hanger.
     * (Potential for trying to get a ball that comes out of the exits with this one)
     * @param drivetrain
     * @param vision
     */
    public LeftDefenseAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);
        //Pose2d startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        //Supplier<Rotation2d> aimNeg90 = () -> { return Rotation2d.fromDegrees(-90);};
        //SwerveControllerCommand driveToBall1 = super.ceateSwerveTrajectoryCommand(super.m_slowTrajectoryConfig, startPos, ball1Pos);
        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(30));
        Pose2d firstBallPos = new Pose2d(Units.inchesToMeters(41), Units.inchesToMeters(35.5), Rotation2d.fromDegrees(30));
        Supplier<Rotation2d> turnToBallOne = () -> { return Rotation2d.fromDegrees(30);};
        //Pose2d shootPos = new Pose2d(Units.inchesToMeters(20),Units.inchesToMeters(35.5), Rotation2d.fromDegrees(160));
        /*Supplier<Rotation2d> turnToShoot = () -> {
            vision.updateLimelight();
            Rotation2d rotation;
            if(vision.hasValidTarget()) {
                rotation = drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx()));
            } else {
                rotation = Rotation2d.fromDegrees(170);
            }
            return rotation;};*/
        Pose2d opponentBallPos = new Pose2d(Units.inchesToMeters(70),Units.inchesToMeters(-10.5), Rotation2d.fromDegrees(-90));
        Supplier<Rotation2d> turnToOpponent = () -> { return Rotation2d.fromDegrees(-90);};
        SwerveControllerCommand driveToFirstBallPos = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, firstBallPos, turnToBallOne);
        VisionTurnInPlaceCommand turnToShoot = new VisionTurnInPlaceCommand(drivetrain, vision);
        //SwerveControllerCommand driveToShootPos = super.createSwerveTrajectoryCommand(super.m_fastTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(170)), shootPos, turnToShoot);
        SwerveControllerCommand driveToOpponentBallPos = super.createSwerveTrajectoryCommand(super.fastTurnTrajectoryConfig, super.getLastEndingPosCreated(Rotation2d.fromDegrees(-90)), opponentBallPos, turnToOpponent);
        //TODO: extend intake and turn on intake motors
        //TODO:Drive to first ball
        this.addCommands(driveToFirstBallPos);
        //TODO:Shoot both loaded balls
        this.addCommands(turnToShoot);
        //TODO: Drive to opponent ball
        this.addCommands(driveToOpponentBallPos);
        //TODO: Shoot opponent ball into hanger
        this.andThen(() -> drivetrain.stop(), drivetrain);
    }
}