package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.VisionTurnInPlaceCommand;
import frc.robot.commands.intake.IntakeArmInCommand;
import frc.robot.commands.intake.IntakeArmOutCommand;
import frc.robot.commands.shooter.NonVisionShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.NonVisionShootCommand.NonVisionShootMode;
import frc.robot.commands.shooter.ShootCommand.ShootMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoBase  extends SequentialCommandGroup {
    private DrivetrainSubsystem drivetrain;
    private VisionSubsystem vision;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private HopperSubsystem hopper;
    private IndexerSubsystem indexer;

    protected SwerveDriveKinematics swerveDriveKinematics;
    private Pose2d lastCreatedEndingPose;

    // private TrajectoryConfig slowTrajectoryConfig;
    // private PIDController slowXYController;
    // private ProfiledPIDController slowThetaController;

    protected final AutoTrajectoryConfig slowTrajectoryConfig;
    protected final AutoTrajectoryConfig fastTurnTrajectoryConfig;
    protected final AutoTrajectoryConfig fastTurnSlowDriveTrajectoryConfig;
    protected final AutoTrajectoryConfig speedDriveTrajectoryConfig;

    public AutoBase(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem hopper, IndexerSubsystem indexer) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.indexer = indexer;

        swerveDriveKinematics = drivetrain.getKinematics();

        slowTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2.5, 1.5).setKinematics(swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI))
        );

        fastTurnTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(3, 1.5).setKinematics(swerveDriveKinematics), // Speed of actions, 1st TrajectoryFactory value is max velocity and 2nd is max accelaration.
            new PIDController(1, 0, 0),  // The XY controller PID value
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 4*Math.PI)) // Turning PID COntroller. Increasing 1st value increases speed of turning, and the TrapezoidalProfile is our contraints of these values.
        );

        fastTurnSlowDriveTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(2, 1.5).setKinematics(swerveDriveKinematics), 
            new PIDController(0.25, 0, 0),
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 3*Math.PI))
        );

        speedDriveTrajectoryConfig = new AutoTrajectoryConfig(
            new TrajectoryConfig(4.5, 3.5).setKinematics(swerveDriveKinematics), 
            new PIDController(1, 0, 0),
            new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4*Math.PI, 3*Math.PI))
        );
    }

    protected ShootCommand newShoot1Command() {
        return new ShootCommand(ShootMode.SHOOT_SINGLE, shooter, indexer, hopper, vision);
    }

    protected ShootCommand newShootAllCommand() {
        return new ShootCommand(ShootMode.SHOOT_ALL, shooter, indexer, hopper, vision);
    }

    protected NonVisionShootCommand newNonVisionShoot1Command(double topWheelVelocityTP100MS, double bottomWheelVelocityTP100MS) {
        return new NonVisionShootCommand(NonVisionShootMode.SHOOT_SINGLE, shooter, indexer, topWheelVelocityTP100MS, bottomWheelVelocityTP100MS);
    }

    protected NonVisionShootCommand newNonVisionShootAllCommand(double topWheelVelocityTP100MS, double bottomWheelVelocityTP100MS) {
        return new NonVisionShootCommand(NonVisionShootMode.SHOOT_ALL, shooter, indexer, topWheelVelocityTP100MS, bottomWheelVelocityTP100MS);
    }

    protected IntakeArmOutCommand newIntakeArmOutCommand() {
        return new IntakeArmOutCommand(intake, indexer, hopper);
    }

    protected IntakeArmInCommand newIntakeArmInCommand() {
        return new IntakeArmInCommand(intake, indexer, hopper);
    }

    protected VisionTurnInPlaceCommand newVisionTurnInPlaceCommand() {
        return new VisionTurnInPlaceCommand(drivetrain, vision);
    }

    /**
     * Creates a custom Trajectory Config from AutoTrajectoryConfig
     * @param maxXYVelocityMPS - Max driving velocity in meters per second
     * @param maxAccelarationMPS - Max driving accelaration in meters per second
     * @param xyP - Driving P (Proportional) value in PID. Increasing makes driving actions happen faster and decreasing makes them slower.
     * @param turnP - Driving P value. Increasing makes turning actions happen faster and decreasing makes them slower.
     * @param turnProfileContraintsMultiplier - Amount PI will be multiplied by in the TrapezoidalProfile.Constrains of the turning PID controller
     * @param startVelocityMPS - Tarjectory's starting velocity in meters per second
     * @param endVelocityMPS - Trajectory's ending velocity in meters per second
     * @return AutoTrajectoryConfig
     */
    protected AutoTrajectoryConfig createTrajectoryConfig(double maxXYVelocityMPS, double maxAccelarationMPS, double xyP, double turnP, double turnProfileContraintsMultiplier, double startVelocityMPS, double endVelocityMPS) {
        return new AutoTrajectoryConfig(
            new TrajectoryConfig(maxXYVelocityMPS, maxAccelarationMPS).setEndVelocity(endVelocityMPS).setStartVelocity(startVelocityMPS),
            new PIDController(xyP, 0, 0),
            new ProfiledPIDController(turnP, 0, 0, new TrapezoidProfile.Constraints(turnProfileContraintsMultiplier*Math.PI, turnProfileContraintsMultiplier*Math.PI))
            );
    }

    /**
     * Creates a custom Trajectory Config from AutoTrajectoryConfig without a specific start and end velocity.
     * @param maxXYVelocityMPS - Max driving velocity in meters per second
     * @param maxAccelarationMPS - Max driving accelaration in meters per second
     * @param xyP - Driving P (Proportional) value in PID. Increasing makes driving actions happen faster and decreasing makes them slower.
     * @param turnP - Driving P value. Increasing makes turning actions happen faster and decreasing makes them slower.
     * @param turnProfileContraintsMultiplier - Amount PI will be multiplied by in the TrapezoidalProfile.Constrains of the turning PID controller
     * @return AutoTrajectoryConfig
     */
    protected AutoTrajectoryConfig createTrajectoryConfig(double maxXYVelocityMPS, double maxAccelarationMPS, double xyP, double turnP, double turnProfileContraintsMultiplier) {
        return new AutoTrajectoryConfig(
            new TrajectoryConfig(maxXYVelocityMPS, maxAccelarationMPS),
            new PIDController(xyP, 0, 0),
            new ProfiledPIDController(turnP, 0, 0, new TrapezoidProfile.Constraints(turnProfileContraintsMultiplier*Math.PI, turnProfileContraintsMultiplier*Math.PI))
            );
    }

    // Most basic deafult swerve command, automatically using slowTrajectoryConfig.
    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            slowTrajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose,
        List<Translation2d> midpointList
    ) {
        return createSwerveTrajectoryCommand(
            trajectoryConfig,
            startPose,
            endPose,
            midpointList,
            () -> { return new Rotation2d(); }
        );
    }

    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        Supplier<Rotation2d> rotationSupplier
    ) {
        lastCreatedEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                new ArrayList<Translation2d>(), //no midpoints in path (S curve)
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            drivetrain::getPose,
            swerveDriveKinematics,
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            drivetrain::setModuleStates,
            drivetrain
        );
    }

    // Swerve controller command for adding an ArrayList of midpoints.
    protected SwerveControllerCommand createSwerveTrajectoryCommand(
        AutoTrajectoryConfig trajectoryConfig, 
        Pose2d startPose, 
        Pose2d endPose, 
        List<Translation2d> midpointList,
        Supplier<Rotation2d> rotationSupplier
    ) {
        lastCreatedEndingPose = endPose;

        return new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                startPose,
                midpointList,
                endPose,
                trajectoryConfig.getTrajectoryConfig()
            ),
            drivetrain::getPose,
            swerveDriveKinematics,
            trajectoryConfig.getXYController(),
            trajectoryConfig.getXYController(),
            trajectoryConfig.getThetaController(), 
            rotationSupplier,
            drivetrain::setModuleStates,
            drivetrain
        );
    }

    protected Supplier<Rotation2d> createRotationAngle(double angle) {
        return () -> { return Rotation2d.fromDegrees(angle); };
    }

    protected Supplier<Rotation2d> createHubTrackingSupplier(double noTargetAngle) {
        return () -> {
            Rotation2d rotation;
            vision.updateLimelight();
            if(vision.hasValidTarget()) {
                rotation = drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(vision.getTx()));
            } else {
                rotation = Rotation2d.fromDegrees(noTargetAngle);
            }
            return rotation;
        };
    }

    protected Pose2d getLastEndingPosCreated() {
        return lastCreatedEndingPose;
    }

    protected Pose2d getLastEndingPosCreated(double rotation) {
        return new Pose2d(lastCreatedEndingPose.getTranslation(), Rotation2d.fromDegrees(rotation));
    }

    protected Pose2d getLastEndingPosCreated(Rotation2d rotation) {
        return new Pose2d(lastCreatedEndingPose.getTranslation(), rotation);
    }
}
