package frc.robot.auto.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.AutoBase;
import frc.robot.commands.drive.TurnInPlaceCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CISTest extends AutoBase {
    /**
     * Test auto for our dear friends CIS
     * @param drivetrain
     * @param shooter
     * @param indexer
     * @param hopper
     * @param climber
     */
    public CISTest(DrivetrainSubsystem drivetrain) {
        super(drivetrain, null, null, null, null, null, null);

        Pose2d startPos = new Pose2d(0,0, Rotation2d.fromDegrees(175));
        Pose2d backUpPos = new Pose2d(Units.inchesToMeters(-55), 0, Rotation2d.fromDegrees(175));

        SwerveControllerCommand backUp = super.createSwerveTrajectoryCommand(super.slowTrajectoryConfig, startPos, backUpPos);

        this.addCommands(backUp);
        this.addCommands(super.newTurnInPlaceCommand(175));
        this.addCommands(super.newStopDrivetrainCommand());

    }
}
