package frc.robot.auto;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.LEDMode;


public class OneBallAuto extends AutoBase{
    //No starting position is needed, simply face the robot towards
    //target. This auto backs up to get the "off the trmac" point and shoots the preloaded ball
    //This auto is a last resort, in case of inability to use limelight
    //or other malfunctions. In normal or ideal cases, this auto should not be used

    public OneBallAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision, TwoWheelFlySubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
        super(drivetrain, vision);
        vision.setLED(LEDMode.ON);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        Pose2d backUpPos = new Pose2d(Units.inchesToMeters(-50),0, Rotation2d.fromDegrees(0));


        //SwerveControllerCommand driveToBackUpPos


    }



}