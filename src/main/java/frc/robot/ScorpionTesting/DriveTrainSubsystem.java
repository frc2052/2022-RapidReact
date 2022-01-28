package frc.robot.ScorpionTesting;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Copied for 2020 scorpion for testing so I don't have to write an entire robot to do so...

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
//import frc.robot.commands.rotateWoFtoColCommand;
//import frc.robot.lib.CsvLogger;

public class DriveTrainSubsystem extends SubsystemBase {

  // private WPI_TalonFX leftMaster;
  // private WPI_TalonFX leftFollower1;   
  // private final WPI_TalonFX rightMaster;
  // private final WPI_TalonFX rightFollower1;
  
  private WPI_TalonSRX leftMaster;
  private VictorSPX leftFollower1; 
  private VictorSPX leftFollower2;
  
  private final WPI_TalonSRX rightMaster;
  private final VictorSPX rightFollower1;
  private final VictorSPX rightFollower2;

  private final Solenoid shifter;

  private boolean isHighGear;

  private final SpeedControllerGroup leftGroup;
  private final SpeedControllerGroup rightGroup;

  private AHRS navX = null;

  private DifferentialDrive drive;
  private DifferentialDriveOdometry odometry;

  public DriveTrainSubsystem() {
    // leftMaster = new WPI_TalonFX(Constants.Motors.kDriveLeftMasterId);
    // leftMaster.configFactoryDefault();
    // leftFollower1 = new WPI_TalonFX(Constants.Motors.kDriveLeftFollowerId);
    // leftFollower1.configFactoryDefault();
    // rightMaster = new WPI_TalonFX(Constants.Motors.kDriveRightMasterId);
    // rightMaster.configFactoryDefault();
    // rightFollower1= new WPI_TalonFX(Constants.Motors.kDriveRightFollowerId);
    // rightFollower1.configFactoryDefault();

    leftMaster = new WPI_TalonSRX(Constants.CopiedFromScorpion.Motors.kDriveLeftMasterId);
    leftMaster.configFactoryDefault();
    leftFollower1 = new WPI_VictorSPX(Constants.CopiedFromScorpion.Motors.kDriveLeftFollower1Id);
    leftFollower1.configFactoryDefault();
    leftFollower2 = new WPI_VictorSPX(Constants.CopiedFromScorpion.Motors.kDriveLeftFollower2Id);
    leftFollower2.configFactoryDefault();
    rightMaster = new WPI_TalonSRX(Constants.CopiedFromScorpion.Motors.kDriveRightMasterId);
    rightMaster.configFactoryDefault();
    rightFollower1= new WPI_VictorSPX(Constants.CopiedFromScorpion.Motors.kDriveRightFollower1Id);
    rightFollower1.configFactoryDefault();
    rightFollower2 = new WPI_VictorSPX(Constants.CopiedFromScorpion.Motors.kDriveRightFollower2Id);
    rightFollower2.configFactoryDefault();
    
    shifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CopiedFromScorpion.Solenoids.kShifterSolenoidID);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.CopiedFromScorpion.DriveTrain.kVelocityControlSlot, Constants.CopiedFromScorpion.DriveTrain.kCANBusConfigTimeoutMS);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.CopiedFromScorpion.DriveTrain.kVelocityControlSlot, Constants.CopiedFromScorpion.DriveTrain.kCANBusConfigTimeoutMS);
  
    rightMaster.setInverted(false);
    rightFollower1.setInverted(false);
    rightFollower2.setInverted(false);
    leftMaster.setInverted(false);
    leftFollower1.setInverted(false);
    leftFollower2.setInverted(false);

    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(false);

    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower1.setNeutralMode(NeutralMode.Brake);
    leftFollower2.setNeutralMode(NeutralMode.Brake);
    rightFollower1.setNeutralMode(NeutralMode.Brake);
    rightFollower2.setNeutralMode(NeutralMode.Brake);

    rightFollower1.follow(rightMaster);
    rightFollower2.follow(rightMaster);
    leftFollower1.follow(leftMaster);
    leftFollower2.follow(leftMaster);

    leftGroup = new SpeedControllerGroup(leftMaster);
    rightGroup  = new SpeedControllerGroup(rightMaster);    
    drive = new DifferentialDrive(leftGroup, rightGroup);

    try {
      navX = new AHRS(SPI.Port.kMXP);
      navX.enableLogging(true);
    } catch (Exception e) {
      DriverStation.reportError("Error instantiating navX: ", e.getStackTrace());
    }

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    //CsvLogger.addLoggingFieldBoolean("shifting", "", "getIsHighGear", this);
    //CsvLogger.addLoggingFieldDouble("odometry x", "", "getPoseX", this);
    //CsvLogger.addLoggingFieldDouble("odometry Y", "", "getPoseY", this);
  }

  

  public void setOdometry(double x, double y){
    Pose2d newPose = new Pose2d(x, y, getAngle());
    odometry.resetPosition(newPose, getAngle());
  }

  public double getPoseX() {
    return Units.metersToInches(odometry.getPoseMeters().getTranslation().getX());
  }

  public double getPoseY() {
    return Units.metersToInches(odometry.getPoseMeters().getTranslation().getY());
  }

  
  public void setHighGear(boolean highGear) {
    shifter.set(highGear);
    isHighGear = highGear;
  }

  public boolean getIsHighGear() {
    return isHighGear;
  }

  public void arcadeDrive(double tank, double turn) {
    System.out.println("Arcade Turn Value: " + turn);
    drive.arcadeDrive(tank, turn);
  }

  public void curvatureDrive(double tank, double turn, boolean quickTurn) {
    //System.out.println("Curvature Turn Value: " + turn);
    // if (quickTurn) {
    //   turn = turn * Constants.DriveTrain.kTurnInPlaceSpeed;
    // }
    drive.curvatureDrive(tank, turn, quickTurn);
  }

  public Rotation2d  getAngle() {
    if (navX == null) {
      return Rotation2d.fromDegrees(0.0);
    } else {
      return Rotation2d.fromDegrees(getHeading());
    }
  }

  public Pose2d getPose() {
    //System.out.println("------POSE: X: " + odometry.getPoseMeters().getTranslation().getX() + "   Y: " + odometry.getPoseMeters().getTranslation().getY() + "    ANGLE: " + odometry.getPoseMeters().getRotation().getDegrees());
    return odometry.getPoseMeters();
  }

  public void putToSmartDashboard() {
    SmartDashboard.putNumber("Right Encoder", rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Encoder", leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("x value meters", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("x value inches", Units.metersToInches(odometry.getPoseMeters().getTranslation().getX()));
    SmartDashboard.putNumber("y value meters", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("y value inches", Units.metersToInches(odometry.getPoseMeters().getTranslation().getY()));
    SmartDashboard.putNumber("NavX Angle", getHeading());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftSpeed = ((double)leftMaster.getSelectedSensorVelocity() / Constants.CopiedFromScorpion.DriveTrain.kTicksPerRot) * Constants.CopiedFromScorpion.DriveTrain.kDriveWheelCircumferenceMeters * Constants.CopiedFromScorpion.DriveTrain.kEncoderGearRatio;
    double rightSpeed = ((double)rightMaster.getSelectedSensorVelocity() / Constants.CopiedFromScorpion.DriveTrain.kTicksPerRot) * Constants.CopiedFromScorpion.DriveTrain.kDriveWheelCircumferenceMeters * Constants.CopiedFromScorpion.DriveTrain.kEncoderGearRatio;


    System.out.println("-----LEFT SPEED: " + leftSpeed + " RIGHT SPEED" + rightSpeed);

    return new DifferentialDriveWheelSpeeds(
      leftSpeed,
      rightSpeed
    );
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(-leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
  }
  

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
    setOdometry(0, 0);
    navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360) * (true ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    Rotation2d rot = Rotation2d.fromDegrees(getHeading());
    double left =  ((double)leftMaster.getSelectedSensorPosition() / Constants.CopiedFromScorpion.DriveTrain.kTicksPerRot) * Constants.CopiedFromScorpion.DriveTrain.kDriveWheelCircumferenceMeters * Constants.CopiedFromScorpion.DriveTrain.kEncoderGearRatio;
    double right = ((double)rightMaster.getSelectedSensorPosition() / Constants.CopiedFromScorpion.DriveTrain.kTicksPerRot) * Constants.CopiedFromScorpion.DriveTrain.kDriveWheelCircumferenceMeters * Constants.CopiedFromScorpion.DriveTrain.kEncoderGearRatio  ; 
    //System.out.println("----------------DEGREES" + rot.getDegrees() + " LEFT DIST: " + left + "  RIGHT DIST: " + right);

    odometry.update(
       rot,
       left,
       right  
    );
  }

 
}
