package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class MiddleTerminal3CargoAuto extends AutoBase {

    /**
     * To Start at Position B (UNWRITTEN)
     * 
     * @param drivetrain
     * @param vision
     */
    public MiddleTerminal3CargoAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain, vision);


    }
}
