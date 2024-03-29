package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;

public class VisionSubsystem extends SubsystemBase{

    // Relay powerRelay = new Relay(Constants.Limelight.RELAY_PORT); - Limelight VEX Power relay code, can't use because defaults to off when robot is disabled

    NetworkTable table;

    private double tx, ty, tl, pipeline, camMode; //, ta, ts, tl, camMode, getpipe;
    private boolean hasValidTarget;
    private double lastTl;
    private double xAimOffset = -3;
    
    private boolean ledOverride = false;
    private boolean isUpdating = true;
    private Timer timer;

    private NetworkTableEntry ltv; // If the Limelight has any targets.              Value between 0.0 and 1.0.
    private NetworkTableEntry ltx; // Horizontal offset from crosshair to target.    Value between -29.8 and 29.8 (degrees).
    private NetworkTableEntry lty; // Vertical offset from crosshair to target.      Value between -24.85 and 24.85 (degrees).
    private NetworkTableEntry ltl; // The pipeline's latency contribution.           Value greater than 0 (ms).

    // private NetworkTableEntry lta; // Target's area of the image.                    Value between 0 and 100 (percent).
    // private NetworkTableEntry lts; // The skew or rotation.                          Value between -90 and 0 (degrees).
    // private NetworkTableEntry ltshort = table.getEntry("tshort");   // Length of shortest bounding box side (pixels)
    // private NetworkTableEntry ltlong = table.getEntry("tlong");     // Length of longest bounding box side  (pixels)
    // private NetworkTableEntry lthor = table.getEntry("thor");       // Horizontal sidelength of bound box   0 - 320 (pixels) 
    // private NetworkTableEntry ltvert = table.getEntry("tvert");     // Vertical sidelength of bound box     0 - 320 (pixels)

    private NetworkTableEntry lledMode;  // Current LED mode, value between 0.0 and 4.0, explained in setLED() method.
    private NetworkTableEntry lcamMode;  // Can be 0, which is regular limelight vision processing, or 1, which turns up the exposure and disables vision processing, intended for driver camera use.
    private NetworkTableEntry lgetpipe;  // True active pipeline index of camera 0 through 9.
    private NetworkTableEntry lpipeline; // NetworkTableEntry needed for setting pipeline value.
    private NetworkTableEntry lstream;   // 0 (Standard) sets side by side streams if a webcam is attached, 1 (PiP Main) sets secondary camera stream in lower right corner for primary stream, 2 (PiP Secondary) sets primary stream in lower right corner of secondary stream.

    /* Unneeded other possible networktable entries
    private NetworkTableEntry lcamtran = table.getEntry("tshort");  // "Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)"
    private NetworkTableEntry ltcornxy = table.getEntry("tcornxy"); // gets a number array of bounding box corner coordiantes [x0,y0,x1,y1……]
    private NetworkTableEntry cx0 = table.getEntry("cx0");          // Crosshair A X in normalized screen space
    private NetworkTableEntry cy0 = table.getEntry("cy0");          // Crosshair A Y in normalized screen space
    private NetworkTableEntry cx1 = table.getEntry("cx1");          // Crosshair B X in normalized screen space
    private NetworkTableEntry cy1 = table.getEntry("cy1");          // Crosshair B Y in normalized screen space
    */

    /**
     * Subsystem for accessing the Limelight's NetworkTable values and creating methods to control it.
     */
    public VisionSubsystem() {
      // Power relay setup code for experimenting with a relay connected to the Limelight to fix it if it stopped updating (common issue we experienced but got better by lowering stream rate so it wasn't worth looking farther into this)
      // powerRelay.set(Relay.Value.kForward); // NEVER SET TO REVERSE YOU'LL BLOW UP THE LIMELIGHT

      table = NetworkTableInstance.getDefault().getTable("limelight");

      ltv = table.getEntry("tv");
      ltx = table.getEntry("tx");
      lty = table.getEntry("ty");
      ltl = table.getEntry("tl");

      lledMode = table.getEntry("ledMode");
      lcamMode = table.getEntry("camMode");
      lgetpipe = table.getEntry("getpipe");
      lpipeline = table.getEntry("pipeline");
      lstream = table.getEntry("stream");

      lastTl = getTl();
    }

    @Override
    public void periodic() {
      hasValidTarget = ltv.getDouble(0.0) == 1.0;

      // Values here are always being calculated anyway (probably because of SmartDashboard), so they're done periodic.
      ty = lty.getDouble(0.0); // get ty before tx because method below requires updated ty
      tx = ltx.getDouble(0.0) + getLinearXAimOffset();

      tl = ltl.getDouble(0.0);
      pipeline = lgetpipe.getDouble(0.0);
      camMode = lcamMode.getDouble(0.0);

      // Limelight Is Dead Detection Code - Checks if the latency hasn't 
      // updated in a full second (which shouldn't happen unless limelight died)
        if (tl == lastTl) {
          if (timer == null) {
              timer = new Timer();
              timer.start();
          }
          if (timer.hasElapsed(1)) { // If the timer has reached a second without clearing, it's broken.
            isUpdating = false;
          }
        } else {
          isUpdating = true;
          clearTimer();
        }

        lastTl = tl;

      SmartDashboard.putBoolean("Has Target?", hasValidTarget);
      // SmartDashboard.putNumber("Pipeline", pipeline);
      // SmartDashboard.putString("Latency", tl + "ms");
      // SmartDashboard.putString("Camera Mode: ", camMode == 0.0 ? "Vision" : "Driver"); // A Java 1 line if statement. If camMode == 0.0 is true it uses "Vision", else is uses "Driver".

      if (hasValidTarget) {
        SmartDashboard.putNumber("Vertical Distance from Crosshair: ", ty);
        SmartDashboard.putNumber("Horizontal Distance from Crosshair: ", tx);
        SmartDashboard.putNumber("Limelight Tolerance", getTolerance());
        // SmartDashboard.putBoolean("Is lined up?", isLinedUp());
        // SmartDashboard.putNumber("Equation xDistance away (Inches)", Units.metersToInches(getEquationDistanceToUpperHubMeters()));
        // SmartDashboard.putNumber("Test Equation Distance", testEquationDistance());
        // double distanceInches = VisionCalculator.getInstance().getDistanceInches(ty);
        // SmartDashboard.putNumber("Calculator Distance Inches", distanceInches);
        // SmartDashboard.putNumber("Calculator Distance Feet", distanceInches / 12);
        // SmartDashboard.putBoolean("Is In Range?", ty > Constants.Limelight.FAR_RANGE_FROM_HUB_ANGLE_DEGREES ? (ty < Constants.Limelight.CLOSE_RANGE_FROM_HUB_ANGLE_DEGREES ? true : false) : false);
      }

    // Experimental distance determined pipeline switching (was uneccessary)
      // if (hasValidTarget) { // Logic for switching pipelines at different ranges, not neccessary
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

    public double getTx() {return this.tx;}
    public double getTy() {return this.ty;}
    public double getTl() {return this.tl;};

    public double getCamMode() {return this.camMode;}
    public double getPipeline() {return this.pipeline;}
    public double getLedMode() {return this.lledMode.getDouble(0.0);}
    public double getStreamMode() {return this.lstream.getDouble(0.0);}
    
    public boolean getIsLinedUp() {
      if (hasValidTarget) {
        if (Math.abs(tx) <= getTolerance()) {
          LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.VISION_TARGET_LINED_UP);
          return true;
        }
      }
      return false; // reaches this return false if above weren't true
      // return Math.abs(tx) < Constants.Limelight.LINED_UP_THRESHOLD; // Could simply be this but then we wouldn't be able to set the LEDs to target_lined_up
    }

    /** Method for accessing tv to see if it has a target, which is when tv = 1.0. */
    public boolean getHasValidTarget() {
      return this.hasValidTarget;
    }

    public Rotation2d getXRotation() {
      return Rotation2d.fromDegrees(tx);
    }

    public Rotation2d getYRotation() {
      return Rotation2d.fromDegrees(ty);
    }

    public boolean getIsUpdating() {
      return isUpdating;
    }

    public double getXAimOffset() {
      return xAimOffset;
    }

        /** Method for calculating how far we can be from aiming at the target to determine 
     * we're lined up to shoot, depending on our distance away with the ty. */
    public double getTolerance() {
      return 0.07 * ty + 3.0; // Intended to be linear tolerance curve for isLinedUp that is 5 degrees when ty = -20 and 1.8 when ty = 20
    }

    /** Linear equation for how far to the left we should aim from the center of the target based on the ty. 
     * Implemented because we were constantly firing to the right, 
     * and don't need to aim as far when we're far away from the target. */
    public double getLinearXAimOffset() {
      return -(0.03 * ty + 2.0); // 3 Degrees to the left when really close (ty of ~25 degrees) and 1 degree when really far (ty of ~-25 degrees).
    }

    /** Would calculate the distance from the Upper Hub using the given equation, constants, and the ty. 
     * flawed because of spherical uniform distance from a camera https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7?
     * as well as vulnerable to other types of distortion and such. Not used in 2022 season. */
    public double getEquationDistanceToUpperHubMeters() {
      return (Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.Limelight.MOUNT_HEIGHT_METERS) / (Math.tan(Math.toRadians(Constants.Limelight.MOUNT_ANGLE_DEGREES + ty))) + Constants.Limelight.DISTANCE_CALCULATION_LINEAR_OFFSET;
    }

    /** @return A speed double in Omega Radians Per Second to be used for swerve chasis rotation.
     * Uses relatively arbitrary values but it worked and did so better than what we could get with 
     * PID controlled rotation lol */
    public double getRotationSpeedToTarget() {
      if(hasValidTarget) {
        if(Math.abs(tx) > 5) {
          return -Math.copySign(Math.toRadians(tx * 9) , tx); // Dynamically changes rotation speed to be faster at a larger tx,
        } else if(Math.abs(tx) > 2) {                              // multiplying tx by 9 to have the lowest value at 5 degrees being PI/4.
          return -Math.copySign(Math.PI /4, tx);
        } else {
          return 0; // Must set rotation to 0 once it's lined up or loses a target, or the chassis will continue to spin.
        }
      } else {
        return 0; // If no target found don't rotate.
      }
    }

    public void setPipeline(double pipeline) { // Method to set pipeline (Limelight 'profile').
      if(pipeline >= 0 && pipeline <= 9)  // 10 availible pipelines.
        lpipeline.setDouble(pipeline);
      else
        System.err.println("SELECT A PIPLINE BETWEEN 0 AND 9!");
    }

    public void setXAimOffset(double xAimOffset) {
        this.xAimOffset = xAimOffset;
    }

    public void setLEDOverride(boolean override) {
      ledOverride = override;
    }

    public void setLEDMode(LEDMode mode) {
      if (!ledOverride) {
        switch(mode) {
          case PIPELINE:
            lledMode.setDouble(0.0);  // Whatever value is specified by the current pipeline.
            break;
          case OFF:
            lledMode.setDouble(1.0);  // Turns the Limelight's LEDs off.
            SmartDashboard.putBoolean("Enable Limelight LEDs", false);  // SmartDashboard boolean sends in the case other locations enable or disable the LEDs.
            break;
          case BLINK:
            lledMode.setDouble(2.0);  // Makes the Limelight's LEDs blink.
            break;
          case ON:
            lledMode.setDouble(3.0);  // Turns the Limelight's LEDs on.
            SmartDashboard.putBoolean("Enable Limelight LEDs", true);
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
          lcamMode.setDouble(1.0);  // Camera settings are adjusted by turning exposure back up to be used as a regular camera by the driver.
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

    private void clearTimer() {
      if(timer != null) {
          timer.stop();
          timer = null;
      }

    // Old method for updating class doubles from their NetworkTable entries, moved most logic to periodic
    // public void updateLimelight() {
    //     tx = ltx.getDouble(0.0);
    //     ty = lty.getDouble(0.0);
    //     ta = lta.getDouble(0.0);
    //     ts = lts.getDouble(0.0);
    //     tl = ltl.getDouble(0.0);

    //     camMode = lcamMode.getDouble(0.0);
    //     getpipe = lgetpipe.getDouble(0.0);
        
    //     hasValidTarget = ltv.getDouble(0.0) == 1.0;
    // }

    /* Experiments with these equations and method of equation calculating limelight distane: https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7?
    public double getNewEquationDistanceToUpperHubMeters() {
      Translation2d xyTranslation = new Translation2d(tx, ty);
      xyTranslation.rotateBy(Rotation2d.fromDegrees(Constants.Limelight.MOUNT_ANGLE_DEGREES));
      double x = xyTranslation.getX();
      double y = xyTranslation.getY();
      double cameraToTargetHeight = Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.Limelight.MOUNT_HEIGHT_METERS;
    }

    public double testEquationDistance() {
      double x = Math.tan(Math.toRadians(tx));
      double y = Math.tan(Math.toRadians(ty - Constants.Limelight.MOUNT_ANGLE_DEGREES));
      double z = 1.0;

      double length = Math.sqrt(x * x + y * y + z * z);
      x /= length;
      y /= length;
      z /= length;

      double cameraToTargetHeight = Constants.Field.UPPER_HUB_HEIGHT_METERS - Constants.Limelight.MOUNT_HEIGHT_METERS;
      double distance = Math.hypot(x, y) / (cameraToTargetHeight / -z);

      return Units.metersToInches(distance);
    } */

    // Limelight VEX Power relay code, couldn't use because defaults to off when robot is disabled

    // public boolean getRelayState() {
    //   if(powerRelay.get() == Relay.Value.kOff) {
    //     return false;
    //   } else {
    //     return true;
    //   }
    // }

    // public void togglePowerRelay() {
    //   if(powerRelay.get() == Relay.Value.kOff) {
    //     powerRelay.set(Relay.Value.kForward);
    //     // System.err.println("************ FORWARD");
    //   } else {
    //     powerRelay.set(Relay.Value.kOff); // NEVER SET TO REVERSE YOU'LL BLOW UP THE LIMELIGHT
    //     // System.err.println("************ OFF");
    //   }
    // }
  }

    public enum LEDMode {PIPELINE, OFF, BLINK, ON}
    public enum CamMode {VISION, DRIVER}
}