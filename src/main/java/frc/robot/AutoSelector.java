package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoSelector {
    
    private static SendableChooser<AutoPaths> sendableChooserAutos;


    public enum AutoPaths {
        DRIVE("drive"),
        DM("don't move"),
        ONE("One ball auto"),
        TWO("Two ball auto"),
        THREEA("Three ball auto simple"),
        THREEB("three ball auto complex"),
        FIvE("scott's five ball (possible?)");

        public String name;

        AutoPaths(String name) {
            this.name = name;
        }
    }
}
