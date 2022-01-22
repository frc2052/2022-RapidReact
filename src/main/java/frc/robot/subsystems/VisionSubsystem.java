//Subsystem for accessing the Limelight's NetworkTables and methods for it

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class VisionSubsystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, getpipe, camtran;

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
    private NetworkTableEntry lgetpipe = table.getEntry("getpipe"); // true active pipeline index of camera 0 ... 9
    private NetworkTableEntry lcamtran = table.getEntry("tshort");  // "Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)"

    public void updateLimelight() {

        this.tv = this.ltv.getDouble(0.0);
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
    }

    /**
     * Returns a double between 0 and 1 for if the Limelight has any targets
     * 
     * @return tv
     */
    public double getTv() {return this.tv;}

    /**
     * Returns a double between between -29.8 and 29.8 (degrees) for the horizontal offset from crosshair to target
     * 
     * @return tx
     */
    public double getTx() {return this.tx;}
    
    /**
     * Returns a double between -24.85 and 24.85 (degrees) for the vertical offset from crosshair to target
     * 
     * @return ty
     */
    public double getTy() {return this.ty;}

     /**
     * Returns a double between 0 and 100 (percent) for the target's area of the image
     * 
     * @return ta
     */
    public double getTa() {return this.ta;}

     /**
     * Returns a double between -90 and 0 (degrees) for the skew or rotation
     * 
     * @return ts
     */
    public double getTs() {return this.ts;}

    /**
     * Returns a double of at least at least 11 (ms) for the Limelights's pipeline's latency contribution 
     * 
     * @return tl
     */
    public double getTl() {return this.tl;}

    /**
     * Returns a double for pixels of the length of shortest bounding box side
     * 
     * @return tshort
     */
    public double getTshort() {return this.tshort;}

    /**
     * Returns a double for pixels of the length of longest bounding box side
     * 
     * @return tlong
     */
    public double getTlong() {return this.tlong;}

    /**
     * Returns a double between 0 - 320 (pixels) for the horizontal sidelength of the bounding box
     * 
     * @return thor
     */
    public double getThor() {return this.thor;}
    
    /**
     * Returns a double between 0 - 320 (pixels) for the vertical sidelength of the bounding box
     * 
     * @return tvert
     */
    public double getTvert() {return this.tvert;}

    /**
     * Returns a double between 0 ... 9 for the true active pipeline index of camera
     * 
     * @return getpipe
     */
    public double getGetpipe() {return this.getpipe;}

    /**
     * Returns a for the "Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)"
     * 
     * @return camtran
     */
    public double getCamtran() {return this.camtran;}
    
}
