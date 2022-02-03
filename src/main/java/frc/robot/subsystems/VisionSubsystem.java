package frc.robot.subsystems;

// Subsystem for accessing the Limelight's NetworkTable values and creating methods to control it.

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, camMode, getpipe;
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
    private NetworkTableEntry lcamMode = table.getEntry("camMode"); // can be 0, which is regular limelight vision processing, or 1, which turns up the exposure and disables vision processing, intended for driver camera use.
    private NetworkTableEntry lgetpipe = table.getEntry("getpipe"); // true active pipeline index of camera 0 through 9
    private NetworkTableEntry lpipeline = table.getEntry("pipeline");

    /* Unneeded other possible networktable entries
    private NetworkTableEntry lcamtran = table.getEntry("tshort");  // "Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)"
    private NetworkTableEntry lstream = table.getEntry("stream");   // 0 (Standard) sets side by side streams if a webcam is attached, 1 (PiP Main) sets secondary camera stream in lower right corner for primary stream, 2 (PiP Secondary) sets primary stream in lower right corner of secondary stream.
    private NetworkTableEntry ltcornxy = table.getEntry("tcornxy"); // gets a number array of bounding box corner coordiantes [x0,y0,x1,y1……]
    private NetworkTableEntry cx0 = table.getEntry("cx0");          // Crosshair A X in normalized screen space
    private NetworkTableEntry cy0 = table.getEntry("cy0");          // Crosshair A Y in normalized screen space
    private NetworkTableEntry cx1 = table.getEntry("cx1");          // Crosshair B X in normalized screen space
    private NetworkTableEntry cy1 = table.getEntry("cy1");          // Crosshair B Y in normalized screen space
    */

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
        this.camMode = this.lcamMode.getDouble(0.0);
        this.getpipe = this.lgetpipe.getDouble(0.0);
        
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
    public double getCamMode() {return this.camMode;}
    public double getGetpipe() {return this.getpipe;}
    
    public boolean hasValidTarget() { // Method for accessing tv to see if it has a target, which is when tv = 1.0.
      return hasValidTarget;
    }

    public void setPipeline(int pipeline) { // Method to set pipeline (Limelight 'profile')
        if(pipeline >= 0 && pipeline <= 9)  // 10 availible pipelines
          lpipeline.setDouble((double) pipeline);
        else
          System.err.println("SELECT A PIPLINE BETWEEN 0 AND 9!");
    }

    public void setLED(LEDMode mode) {
      switch(mode) {
        case PIPELINE:
          lledMode.setDouble(0.0);  // Whatever value is specified by the current pipeline
          break;
        case OFF:
          lledMode.setDouble(1.0);  // Turns the Limelight's LEDs off
          break;
        case BLINK:
          lledMode.setDouble(2.0);  // Makes the Limelight's LEDs blink
          break;
        case ON:
          lledMode.setDouble(3.0);  // Turns the Limelight's LEDs on
          break;
      }
    }

    public void setCamMode(CamMode mode) {
      switch(mode) {
        case VISION:
          setLED(LEDMode.ON);
          // TODO switch back to default pipeline with option to choose in SmartDashboard
          this.setPipeline(0);
          lcamMode.setDouble(0.0);  // Camera is used for vision processing
          setPipeline(0);
          break;
        case DRIVER:
          setLED(LEDMode.OFF);
          lcamMode.setDouble(1.0);  // Camera settings are adjusted by turning exposure back up to be used as a regular camera by the driver
          setPipeline(1);   // Change to pipeline 
          break;
      }
    }

    public double xDistanceToUpperHub() { // Calculates the distance from the Upper Hub using constants and ty. Make sure to first call updateLimelight() before using this.
      updateLimelight();
      return (Constants.Field.kUpperHubHeight - Constants.Limelight.kMountHeight) / (Math.tan(Math.toRadians(Constants.Limelight.kMountAngle + ty))) + Constants.Limelight.kDistanceCalcOffset;
    }

    public double xDistanceToUpperHub(double angle) { // Same calculation as the other but uses an angle argument.
      return (Constants.Field.kUpperHubHeight - Constants.Limelight.kMountHeight) / (Math.tan(Math.toRadians(Constants.Limelight.kMountAngle + angle))) + Constants.Limelight.kDistanceCalcOffset;
    }

    public double getRotationSpeedToTarget() { // Returns a speed double in Omega Radians Per Second to be used for swerve chasis rotation 
      if(hasValidTarget()) {
        // Logic to set the chassis rotation speed based on horizontal offset.
        if(Math.abs(this.tx) > 5) {
          return -Math.copySign(Math.toRadians(this.tx * 9) , this.tx); // Dynamically changes rotation speed to be faster at a larger tx,
        } else if(Math.abs(this.tx) > 2) {                              // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
          return -Math.copySign(Math.PI /4, this.tx);
        } else {
          return 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
        }
      } else {
        // No target found so don't turn.
        return 0;
      }
    }

    public void putToSmartDashboard() {
        updateLimelight();
        SmartDashboard.putBoolean("Has target?", hasValidTarget);
        SmartDashboard.putString("Vertical Distance from Crosshair: ", ty + " degrees");
        SmartDashboard.putString("Horizontal Distance from Crosshair: ", tx + " degrees");
        SmartDashboard.putString("Target's area of the image", ta + "%");
        SmartDashboard.putNumber("Skew or Rotation: ", ts);
        SmartDashboard.putString("Latency: ", tl + "ms");
        SmartDashboard.putNumber("Pipeline: ", getpipe);
        SmartDashboard.putString("Camera Mode: ", camMode == 0.0 ? "Vision" : "Driver"); // A Java 1 line if statement. If camMode == 0.0 is true it uses "Vision", else is uses "Driver".

        SmartDashboard.putNumber("xDistance away (Meters): ", xDistanceToUpperHub());
        SmartDashboard.putNumber("xDistance away (Inches)", Units.metersToInches(xDistanceToUpperHub()));

        //SmartDashboard.putString("Shortest Bounding Box Side", tshort + " pixels");
        //SmartDashboard.putString("Longest Bounding Box Side", tlong + " pixels");
        //SmartDashboard.putString("Horizontal Bounding Box Sidelength", thor + " pixels");
        //SmartDashboard.putString("Vertical Bounding Box Sidelength", tvert + " pixels");
        //SmartDashboard.putNumber("Camtran", camtran);
    }

    public enum LEDMode {PIPELINE, OFF, BLINK, ON}
    public enum CamMode {VISION, DRIVER}
}