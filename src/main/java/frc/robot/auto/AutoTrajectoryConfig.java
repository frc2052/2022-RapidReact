// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/** 
 * Wrapper class for creating custom TrajectoryConfigs for max velocity, accelaration, PID control of XY and turn, as well as start and end velocity all in auto.
 */
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

    public AutoTrajectoryConfig withStartVelocity(double startVelocityMPS) {
        return new AutoTrajectoryConfig(m_trajectoryConfig.setStartVelocity(startVelocityMPS), m_XYController, m_thetaController);
    }

    public AutoTrajectoryConfig withEndVelocity(double endVelocityMPS) {
        return new AutoTrajectoryConfig(m_trajectoryConfig.setEndVelocity(endVelocityMPS), m_XYController, m_thetaController);
    }

    public AutoTrajectoryConfig withStartAndEndVelocity(double velocityMPS) {
        return new AutoTrajectoryConfig(m_trajectoryConfig.setStartVelocity(velocityMPS).setEndVelocity(velocityMPS), m_XYController, m_thetaController);
    }

    public AutoTrajectoryConfig withStartAndEndVelocity(double startVelocityMPS, double endVelocityMPS) {
        return new AutoTrajectoryConfig(m_trajectoryConfig.setStartVelocity(startVelocityMPS).setEndVelocity(endVelocityMPS), m_XYController, m_thetaController);
    }
}
