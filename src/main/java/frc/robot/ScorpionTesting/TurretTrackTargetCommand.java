package frc.robot.ScorpionTesting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

import frc.robot.ScorpionTesting.*;


public class TurretTrackTargetCommand extends CommandBase {

    private VisionSubsystem vision;
    private TurretSubsystem turret;
    
    public TurretTrackTargetCommand() {
        vision = new VisionSubsystem();
        turret = new TurretSubsystem();

    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.print("Turret Tracking Initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        vision.updateLimelight();
        double angle = vision.getTx();
        if(vision.hasValidTarget()) {
            if(vision.getTx() > 2 || angle < -2)
                turret.driveToPos(angle);
            else
                turret.turnTurret(0);
        } else {
          turret.turnTurret(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.turnTurret(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
