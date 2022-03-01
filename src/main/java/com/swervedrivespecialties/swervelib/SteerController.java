package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.sensors.CANCoder;

public interface SteerController {
    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    CANCoder getSteerCANCoder();

    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
}
