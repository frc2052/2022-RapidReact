package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import frc.robot.subsystems.VisionSubsystem.LEDMode;

public class LeftTerminal3Cargo extends AutoBase {

    /**
     * Position D Start (Far Left Parallel With Outer Tarmac Edge) Facing Away From the Hub.
     * Auto for driving to terminal cargo from position D (not likely to be used)
     * (UNWRITTEN)
     * @param drivetrain
     * @param vision
     */
    public LeftTerminal3Cargo(DrivetrainSubsystem drivetrain, VisionSubsystem vision) {
        super(drivetrain, vision);

        
    }
}
