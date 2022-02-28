package frc.robot.subsystems;

// Subsystem for accessing the Limelight's NetworkTable values and creating methods to control it.

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{

    // Relay powerRelay = new Relay(Constants.Limelight.RELAY_PORT); - Limelight VEX Power relay code, can't use because defaults to off when robot is disabled

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double tx, ty; //, ta, ts, tl, camMode, getpipe;
    private boolean hasValidTarget;
    
    private boolean ledOverride = false;

    private NetworkTableEntry ltv = table.getEntry("tv"); // If the Limelight has any targets.              Value between 0.0 and 1.0.
    private NetworkTableEntry ltx = table.getEntry("tx"); // Horizontal offset from crosshair to target.    Value between -29.8 and 29.8 (degrees).
    private NetworkTableEntry lty = table.getEntry("ty"); // Vertical offset from crosshair to target.      Value between -24.85 and 24.85 (degrees).
    private NetworkTableEntry lta = table.getEntry("ta"); // Target's area of the image.                    Value between 0 and 100 (percent).
    private NetworkTableEntry lts = table.getEntry("ts"); // The skew or rotation.                          Value between -90 and 0 (degrees).
    private NetworkTableEntry ltl = table.getEntry("tl"); // The pipeline's latency contribution.           Value greater than 0 (ms).

    private NetworkTableEntry ltshort = table.getEntry("tshort");   // Length of shortest bounding box side (pixels)
    private NetworkTableEntry ltlong = table.getEntry("tlong");     // Length of longest bounding box side  (pixels)
    private NetworkTableEntry lthor = table.getEntry("thor");       // Horizontal sidelength of bound box   0 - 320 (pixels) 
    private NetworkTableEntry ltvert = table.getEntry("tvert");     // Vertical sidelength of bound box     0 - 320 (pixels)

    private NetworkTableEntry lledMode = table.getEntry("ledMode"); // Current LED mode, value between 0.0 and 4.0, explained in setLED() method.
    private NetworkTableEntry lcamMode = table.getEntry("camMode"); // Can be 0, which is regular limelight vision processing, or 1, which turns up the exposure and disables vision processing, intended for driver camera use.
    private NetworkTableEntry lgetpipe = table.getEntry("getpipe"); // True active pipeline index of camera 0 through 9.
    private NetworkTableEntry lpipeline = table.getEntry("pipeline"); // NetworkTableEntry needed for setting pipeline value.
    private NetworkTableEntry lstream = table.getEntry("stream"); // 0 (Standard) sets side by side streams if a webcam is attached, 1 (PiP Main) sets secondary camera stream in lower right corner for primary stream, 2 (PiP Secondary) sets primary stream in lower right corner of secondary stream.

    /* Unneeded other possible networktable entries
    private NetworkTableEntry lcamtran = table.getEntry("tshort");  // "Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)"
    private NetworkTableEntry ltcornxy = table.getEntry("tcornxy"); // gets a number array of bounding box corner coordiantes [x0,y0,x1,y1……]
    private NetworkTableEntry cx0 = table.getEntry("cx0");          // Crosshair A X in normalized screen space
    private NetworkTableEntry cy0 = table.getEntry("cy0");          // Crosshair A Y in normalized screen space
    private NetworkTableEntry cx1 = table.getEntry("cx1");          // Crosshair B X in normalized screen space
    private NetworkTableEntry cy1 = table.getEntry("cy1");          // Crosshair B Y in normalized screen space
    */

    // public VisionSubsystem() {
    //   powerRelay.set(Relay.Value.kForward); // NEVER SET TO REVERSE YOU'LL BLOW UP THE LIMELIGHT
    // }

    @Override
    public void periodic() {
      hasValidTarget = ltv.getDouble(0.0) == 1.0;

      tx = ltx.getDouble(0.0);
      ty = lty.getDouble(0.0);

      // if (hasValidTarget) {
      //   if ((getPipeline() == 0 && ty >= Constants.Limelight.PIPELINE_SWITCH_TY_DEGREES + Constants.Limelight.PIPELINE_SWITCH_THRESHOLD)) { // Logic for switching between Limelight vision pipelines on distance (angle here) - written weirdly to be as efficient as possible
      //     if (getCamMode() != 1.0) {
      //       setPipeline(1);
      //     }
      //   } else if (getPipeline() == 1 && ty < Constants.Limelight.PIPELINE_SWITCH_TY_DEGREES - Constants.Limelight.PIPELINE_SWITCH_THRESHOLD) {
      //     if (getCamMode() != 1.0) {
      //       setPipeline(0);
      //     }
      //   }
      // } else if (getPipeline() != 0.0) {
      //   if (getCamMode() != 1.0) {
      //     setPipeline(0);
      //   }
      // }
    }

    // public void updateLimelight() { // Method for updating class doubles from their NetworkTable entries.
    //     tx = ltx.getDouble(0.0);
    //     ty = lty.getDouble(0.0);
    //     ta = lta.getDouble(0.0);
    //     ts = lts.getDouble(0.0);
    //     tl = ltl.getDouble(0.0);

    //     camMode = lcamMode.getDouble(0.0);
    //     getpipe = lgetpipe.getDouble(0.0);
        
    //     hasValidTarget = ltv.getDouble(0.0) == 1.0;
    // }

    public double getTx() {return this.tx;}
    public double getTy() {return this.ty;}
    public double getTa() {return this.lta.getDouble(0.0);}
    public double getTs() {return this.lts.getDouble(0.0);}
    public double getTl() {return this.ltl.getDouble(0.0);}

    public double getTshort() {return this.ltshort.getDouble(0.0);}
    public double getTlong() {return this.ltlong.getDouble(0.0);}
    public double getThor() {return this.lthor.getDouble(0.0);}
    public double getTvert() {return this.ltvert.getDouble(0.0);}

    public double getCamMode() {return this.lcamMode.getDouble(0.0);}
    public double getPipeline() {return this.lgetpipe.getDouble(0.0);}
    public double getLedMode() {return this.lledMode.getDouble(0.0);}
    public double getStreamMode() {return this.lstream.getDouble(0.0);}

    // Limelight VEX Power relay code, can't use because defaults to off when robot is disabled
    // public boolean getRelayState() {
    //   if(powerRelay.get() == Relay.Value.kOff) {
    //     return false;
    //   } else {
    //     return true;
    //   }
    // }
    
    public boolean isLinedUp() {
      return Math.abs(tx) < Constants.Limelight.LINED_UP_THRESHOLD;
    }

    public boolean hasValidTarget() { // Method for accessing tv to see if it has a target, which is when tv = 1.0.
      return this.hasValidTarget;
    }

    public void setPipeline(double pipeline) { // Method to set pipeline (Limelight 'profile').
      if(pipeline >= 0 && pipeline <= 9)  // 10 availible pipelines.
        lpipeline.setDouble(pipeline);
      else
        System.err.println("SELECT A PIPLINE BETWEEN 0 AND 9!");
    }

    public void setLEDOverride(boolean override) {
      ledOverride = override;
    }

    public void setLED(LEDMode mode) {
      if (!ledOverride) {
        switch(mode) {
          case PIPELINE:
            lledMode.setDouble(0.0);  // Whatever value is specified by the current pipeline.
            break;
          case OFF:
            lledMode.setDouble(1.0);  // Turns the Limelight's LEDs off.
            break;
          case BLINK:
            lledMode.setDouble(2.0);  // Makes the Limelight's LEDs blink.
            break;
          case ON:
            lledMode.setDouble(3.0);  // Turns the Limelight's LEDs on.
            break;
        }
      }
    }

    public void setCamMode(CamMode mode) {
      switch(mode) {
        case VISION:
          lcamMode.setDouble(0.0);  // Camera is used for vision processing.
          setPipeline(Constants.Limelight.DEFAULT_PIPELINE);      // Change to default pipeline for vision processing.
          break;
        case DRIVER:
          lcamMode.setDouble(9.0);  // Camera settings are adjusted by turning exposure back up to be used as a regular camera by the driver.
          setPipeline(9.0);      // Change to pipeline intended for driver cam.
          break;
      }
    }

    public LEDMode getLEDModeEnum() {
      switch((int)lledMode.getDouble(0.0)) {
        case 0:
          return LEDMode.PIPELINE;
        case 1:
          return LEDMode.OFF;
        case 2:
          return LEDMode.BLINK;
        case 3:
          return LEDMode.OFF;
        default:
          return LEDMode.PIPELINE;
      }
    }

    public double getEquationDistanceToUpperHubMeters() { // Calculates the distance from the Upper Hub using constants and ty. Make sure to first call updateLimelight() before using this.
      return (Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.Limelight.MOUNT_HEIGHT_METERS) / (Math.tan(Math.toRadians(Constants.Limelight.MOUNT_ANGLE_DEGREES + ty))) + Constants.Limelight.DISTANCE_CALCULATION_LINEAR_OFFSET;
    }

    public double getRotationSpeedToTarget() { // Returns a speed double in Omega Radians Per Second to be used for swerve chasis rotation.
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
        return Math.PI; // If no target found rotate half a rotation per second to find target.
      }
    }

    // Limelight VEX Power relay code, can't use because defaults to off when robot is disabled
    // public void togglePowerRelay() {
    //   if(powerRelay.get() == Relay.Value.kOff) {
    //     powerRelay.set(Relay.Value.kForward);
    //     // System.err.println("************ FORWARD");
    //   } else {
    //     powerRelay.set(Relay.Value.kOff); // NEVER SET TO REVERSE YOU'LL BLOW UP THE LIMELIGHT
    //     // System.err.println("************ OFF");
    //   }
    // }

    public void putToSmartDashboard() {
        SmartDashboard.putBoolean("Is lined up?", isLinedUp());
        SmartDashboard.putBoolean("Has target?", hasValidTarget);
        SmartDashboard.putNumber("Vertical Distance from Crosshair: ", ty);
        SmartDashboard.putNumber("Horizontal Distance from Crosshair: ", tx);
        SmartDashboard.putString("Target's area of the image", getTa() + "%");
        SmartDashboard.putString("Latency: ", getTl() + "ms");
        SmartDashboard.putNumber("Pipeline: ", getPipeline());
        SmartDashboard.putString("Camera Mode: ", getCamMode() == 0.0 ? "Vision" : "Driver"); // A Java 1 line if statement. If camMode == 0.0 is true it uses "Vision", else is uses "Driver".
        SmartDashboard.putBoolean("Enable Limelight LEDs", getLedMode() == 1.0 ? false : (getLedMode() == 3.0 ? true : false)); // To update toggle in case classes other than DashboardControlsSubsystem enable the LEDs.

      //SmartDashboard.putNumber("xDistance away (Meters): ", getEquationDistanceToUpperHubMeters());
        SmartDashboard.putNumber("xDistance away (Inches)", Units.metersToInches(getEquationDistanceToUpperHubMeters()));

        SmartDashboard.putBoolean("Is In Range?", ty > Constants.Limelight.FAR_RANGE_FROM_HUB_ANGLE_DEGREES ? (ty < Constants.Limelight.CLOSE_RANGE_FROM_HUB_ANGLE_DEGREES ? true : false) : false);

        //SmartDashboard.putString("Shortest Bounding Box Side", tshort + " pixels");
        //SmartDashboard.putString("Longest Bounding Box Side", tlong + " pixels");
        //SmartDashboard.putString("Horizontal Bounding Box Sidelength", thor + " pixels");
        //SmartDashboard.putString("Vertical Bounding Box Sidelength", tvert + " pixels");
        //SmartDashboard.putNumber("Camtran", camtran);
    }

    public enum LEDMode {PIPELINE, OFF, BLINK, ON}
    public enum CamMode {VISION, DRIVER}
}