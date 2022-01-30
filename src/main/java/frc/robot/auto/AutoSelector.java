package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    private static SendableChooser<autos> autoSelector;

    public static void addToSmartDashboard() {
        autoSelector = new SendableChooser<autos>();

        autoSelector.setDefaultOption(autos.values()[0].name, autos.values()[0]);
        for(int i = 1; i < autos.values().length; i++)
            autoSelector.setDefaultOption(autos.values()[i].name, autos.values()[i]);

        SmartDashboard.putData("Auto Options", autoSelector);
    }

    public static autos getSelectedAuto() {
        return autoSelector.getSelected();
    }

    public enum autos {
        SIMPLE3BALL("Simple 3 Ball"),
        TESTAUTO1("TestAuto1");

        public String name;

        autos(String name) {
            this.name = name;
        }
    }
}
