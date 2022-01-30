//Subsystem for accessing the Limelight's NetworkTables and methods for it

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, getpipe, camtran;
    private boolean hasValidTarget;

    private NetworkTableEntry ltv = table.getEntry("tv"); // if the Limelight has any targets               between 0 and 1
    private NetworkTableEntry ltx = table.getEntry("tx"); // horizontal offset from crosshair to target     between -29.8 and 29.8 (degrees) 
    private NetworkTableEntry lty = table.getEntry("ty"); // vertical offset from crosshair to target       between -24.85 and 24.85 (degrees)
    private NetworkTableEntry lta = table.getEntry("ta"); // target's area of the image                     between 0 and 100 (percent)  
    private NetworkTableEntry lts = table.getEntry("ts"); // the skew or rotation                           between -90 and 0 (degrees)
    private NetworkTableEntry ltl = table.getEntry("tl"); // the pipeline's latency contribution            at least 11ms

    private NetworkTableEntry ltshort = table.getEntry("tshort");   // length of shortest bounding box side (pixels)
    private NetworkTableEntry ltlong = table.getEntry("tlong");     // length of longest bounding box side  (pixels)
    private NetworkTableEntry lthor = table.getEntry("thor");       // horizontal sidelength of bound box   0 - 320 (pixels) 
    private NetworkTableEntry ltvert = table.getEntry("tvert");     // vertical sidelength of bound box     0 - 320 (pixels)

    private NetworkTableEntry lledMode = table.getEntry("ledMode");
    private NetworkTableEntry lcamMode = table.getEntry("camMode");
    private NetworkTableEntry lgetpipe = table.getEntry("getpipe"); // true active pipeline index of camera 0 ... 9
    private NetworkTableEntry lcamtran = table.getEntry("tshort");  // "Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)"

    public void updateLimelight() {
        this.tx = this.ltx.getDouble(0.0);
        this.ty = this.lty.getDouble(0.0);
        this.ta = this.lta.getDouble(0.0);
        this.ts = this.lts.getDouble(0.0);
        this.tl = this.ltl.getDouble(0.0);

        this.tshort = this.ltshort.getDouble(0.0);
        this.tlong = this.ltlong.getDouble(0.0);
        this.thor = this.lthor.getDouble(0.0);
        this.tvert = this.ltvert.getDouble(0.0);
        this.getpipe = this.lgetpipe.getDouble(0.0);
        this.camtran = this.lcamtran.getDouble(0.0);
        
        hasValidTarget = ltv.getDouble(0.0) == 1.0;
    }

    public double getTx() {return this.tx;}
    public double getTy() {return this.ty;}
    public double getTa() {return this.ta;}
    public double getTs() {return this.ts;}
    public double getTl() {return this.tl;}

    public double getTshort() {return this.tshort;}
    public double getTlong() {return this.tlong;}
    public double getThor() {return this.thor;}
    public double getTvert() {return this.tvert;}
    public double getGetpipe() {return this.getpipe;}
    public double getCamtran() {return this.camtran;}
    
    public boolean hasValidTarget() { // method for accessing tv to see if it has a target, which is when tv = 1.0.
      return hasValidTarget;
    }

    public void setPipeline(int pipeline) { // method to set pipeline (limelight 'profile')
        if(pipeline >= 0 && pipeline <= 9) // 10 availible pipelines
          lgetpipe.setDouble((double) pipeline);
        else
          System.err.println("SELECT A PIPLINE BETWEEN 0 AND 9!");
    }

    public void setLED(LEDMode mode) {
      lledMode.setNumber(mode.value);
    }

    public void setCamMode(CamMode mode) {
      switch(mode) {
        case VISION:
          setLED(LEDMode.ON);
          lcamMode.setNumber(0);  // Camera is used for vision processing
          break;
        case DRIVER:
          setLED(LEDMode.OFF);
          lcamMode.setNumber(1);  // Camera settings are adjusted by turning exposure back up to be used as a regular camera by the driver
          this.setPipeline(9);
          break;
      }
    }

    public double xDistanceToUpperHub() {
      updateLimelight();
      return (Constants.Field.kUpperHubHeight - Constants.Limelight.kMountHeight) / (Math.tan(Math.toRadians(Constants.Limelight.kMountAngle + ty))) + Constants.Limelight.kDistanceCalcOffset;
    }

    public double xDistanceToUpperHub(double angle) {
      return (Constants.Field.kUpperHubHeight - Constants.Limelight.kMountHeight) / (Math.tan(Math.toRadians(Constants.Limelight.kMountAngle + angle))) + Constants.Limelight.kDistanceCalcOffset;
    }

    public void putToSmartDashboard() { //sends all values to smart dashboad, primarily for learning purposes right now
        updateLimelight();
        SmartDashboard.putBoolean("Has target?", hasValidTarget);
        SmartDashboard.putString("Vertical Distance from Crosshair", ty + " degrees");
        SmartDashboard.putString("Horizontal Distance from Crosshair", tx + " degrees");
        SmartDashboard.putString("Target's area of the image", ta + "%");
        SmartDashboard.putNumber("Skew or Rotation", ts);
        SmartDashboard.putString("Pipeline latency contribution", tl + "ms");

        SmartDashboard.putString("Shortest Bounding Box Side", tshort + " pixels");
        SmartDashboard.putString("Longest Bounding Box Side", tlong + " pixels");
        SmartDashboard.putString("Horizontal Bounding Box Sidelength", thor + " pixels");
        SmartDashboard.putString("Vertical Bounding Box Sidelength", tvert + " pixels");
        SmartDashboard.putNumber("Pipeline", getpipe);
        SmartDashboard.putNumber("Camtran", camtran);

        SmartDashboard.putString("xDistance away (Meters)", xDistanceToUpperHub() + " meters");
        SmartDashboard.putString("xDistance away (Inches)", Units.metersToInches(xDistanceToUpperHub()) + " inches");
    }

    public enum LEDMode {
      PIPELINE(0),
      OFF(1), 
      BLINK(2), 
      ON(3);
    
      public int value;

      LEDMode(int value) {
        this.value = value;
      }
    }

    public enum CamMode {VISION, DRIVER}
}