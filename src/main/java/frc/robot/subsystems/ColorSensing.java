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
    //this is where the rgb values of the cargo are defined.
    private final Color kBlueTarget= new Color(0.14, 0.47, 0.37);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.132, 0.99, 0.32);
    //this YELLOW color is for testing, it has no real value in the game.
    
    //here we declare the port that the color sensor uses
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    //creating the color sensor
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    //this variable is what we assign our raw incoming color values to
    public Color detectedColor;
    public String colorString;

    //this method makes it possible to match incoming colors with our detectedColor
    public void initializeColor() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);

    }


    //here we are re-defining what periodic is.
    @Override
    public void periodic(){
        //attaching our raw incoming values to a variable
        detectedColor = m_colorSensor.getColor();
        
        //telling the sensor to try and match our incoming values to that of one of the defined colors.
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
        //if a color matches, change the name of the string that we will print to shuffleboard.
        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";   
        }else{
            colorString = "NA";
        }
        
        //outputting the current values of red blue and green to shuffleboard
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Green", detectedColor.green);
        //having the colorString shown on shuffleboard
        SmartDashboard.putString("Color is", colorString);
        //string put to shuffleboard so we can see if our new changes have been pushed to shuffleboard.
        SmartDashboard.putString("Version", "2");
    }
        

}