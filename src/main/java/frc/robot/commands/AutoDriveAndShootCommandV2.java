package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.auto.AutoTrajectoryConfig;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoDriveAndShootCommandV2 extends CommandBase {
    private AutoTrajectoryConfig m_slowTrajectoryConfig;

    private final Timer m_timer = new Timer();
    private Trajectory m_trajectory;
    private Supplier<Pose2d> m_pose;
    private SwerveDriveKinematics m_kinematics;
    private HolonomicDriveController m_controller;
    private Consumer<SwerveModuleState[]> m_outputModuleStates;
    private Supplier<Rotation2d> m_desiredRotation;

    private Supplier<Rotation2d> visionRotationSupplier;

    private DrivetrainSubsystem m_driveTrain;
    private VisionSubsystem m_vision;

    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Pose2d m_lastCreatedEndingPose;

    private boolean isLinedUp = false;
    
    public AutoDriveAndShootCommandV2(Pose2d startPose, Pose2d endPose, DrivetrainSubsystem driveTrain, VisionSubsystem vision) {
        m_driveTrain = driveTrain;
        m_vision = vision;
        m_swerveDriveKinematics = driveTrain.getKinematics();

        visionRotationSupplier = () -> { return Rotation2d.fromDegrees(m_vision.getRotationSpeedToTarget() + drivingHorizontalFiringOffsetAngle()); };

        m_slowTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(m_swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );

        m_trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            new ArrayList<Translation2d>(), //no midpoints in path (S curve)
            endPose,
            m_slowTrajectoryConfig.getTrajectoryConfig()
        );

        m_pose = m_driveTrain::getPose;
        m_kinematics = m_driveTrain.getKinematics();
        m_controller = new HolonomicDriveController(m_slowTrajectoryConfig.getXYController(),
                                                    m_slowTrajectoryConfig.getXYController(),
                                                    m_slowTrajectoryConfig.getThetaController()
                                                    );
        m_outputModuleStates = m_driveTrain::setModuleStates;
        m_desiredRotation = visionRotationSupplier;
        
    }

    

    @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    m_vision.updateLimelight();
    m_desiredRotation = () -> { return Rotation2d.fromDegrees(m_vision.getRotationSpeedToTarget() + drivingHorizontalFiringOffsetAngle()); };
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Temporarily using this from VisionDriveCommand and now also in VisionSubsystem for more adjustability as I test this and try to get it to work.
  public double getRotationSpeedToTarget() { // Returns a speed double in Omega Radians Per Second to be used for swerve chasis rotation 
    m_vision.updateLimelight(); // VisionSubsystem's method to update networktable values.
    double horizontalAngle = m_vision.getTx() + drivingHorizontalFiringOffsetAngle();      // Horizontal offset from the Limelight's crosshair to target.
    double visionRotation;

    if(m_vision.hasValidTarget()) { // Logic to set the chassis rotation speed based on horizontal offset.
        if(Math.abs(horizontalAngle) > 5) {
            visionRotation = -Math.copySign(Math.toRadians(horizontalAngle * 9) , horizontalAngle); // Dynamically changes rotation speed to be faster at a larger tx,
        } else if(Math.abs(horizontalAngle) > 2) {                                                   // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
            visionRotation = -Math.copySign(Math.PI /4, horizontalAngle);
        } else {
            visionRotation = 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
            isLinedUp = true;
        }
    } else {
        visionRotation = Math.PI / 2;
        visionRotation = 0;
    }
    return visionRotation;
  }

  private double drivingHorizontalFiringOffsetAngle() {
    if(m_driveTrain.getLastWheelVelocity() < 0.2) {    // Just avoids doing all the math if we're not or barely moving anyway
      return 0.0;
    }
    // TODO calculate horizontal firing angle offset using driveTrain.getVelocity() using theta = tan^-1(d*(velocity of the robot)/(x velocity of the ball leaving the shooter)/sqrt(height^2+distance^2))
    double firingVelocity = 20.0; // [TEMP VALUE] TODO make this get the value calculated for firing the shooter 
    double lineToHub = Math.sqrt(Math.pow(Constants.Field.kUpperHubHeightMeters, 2) + Math.pow(m_vision.xDistanceToUpperHub(), 2));
    return Math.atan(Math.toRadians(m_vision.xDistanceToUpperHub()*m_driveTrain.getLastWheelVelocity()/firingVelocity/lineToHub)) + Math.toRadians(Constants.DriveTrain.kDrivingAimAngleOffsetDegrees);
  }

  public boolean getIsLinedUp() {
    return isLinedUp;
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}