package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;


public class ColorSensing extends SubsystemBase{

    private static ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget= new Color(0.14, 0.47, 0.37);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.132, 0.99, 0.32);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    public boolean wantColorSense = false;
    public Color detectedColor;
    public String colorString;
    public void initializeColor() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);

    }
    @Override
    public void periodic(){
        detectedColor = m_colorSensor.getColor();
        
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";   
        }else{
            colorString = "Nothing";
        }
        
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putString("Yes, the color is", colorString);
    SmartDashboard.putString("Version", "2");
    }
        

}