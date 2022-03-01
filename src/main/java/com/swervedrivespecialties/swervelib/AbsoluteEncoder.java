package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.sensors.CANCoder;

public interface AbsoluteEncoder {
    CANCoder getCANCoder();

    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    double getAbsoluteAngle();
}
