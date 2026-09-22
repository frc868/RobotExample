// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
    /**
     * An auto which will have the drivetrain move forward at half speed for 2s.
     * 
     * @param drivetrain the drivetrain subsystem
     * @return the auto command
     */
    public static Command moveOut(Drivetrain drivetrain) {
        return drivetrain.tankDriveCommand(() -> .5, () -> .5).withTimeout(Seconds.of(2.0));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
