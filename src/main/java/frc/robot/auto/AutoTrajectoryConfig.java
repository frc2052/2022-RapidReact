// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/** Add your docs here. */
public class AutoTrajectoryConfig {
    private final TrajectoryConfig m_trajectoryConfig;
    private final PIDController m_XYController;
    private final ProfiledPIDController m_thetaController;

    public AutoTrajectoryConfig(TrajectoryConfig trajectoryConfig, PIDController XYController, ProfiledPIDController thetController) {
        this.m_trajectoryConfig = trajectoryConfig;
        this.m_XYController = XYController;
        this.m_thetaController = thetController;
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return m_trajectoryConfig;
    }

    public PIDController getXYController() {
        return m_XYController;
    }

    public ProfiledPIDController getThetaController() {
        return m_thetaController;
    }
}
