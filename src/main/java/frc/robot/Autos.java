// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

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

    /**
     * Creates an auto which will drive forward for 2s while moving the elevator to
     * intake, and will then extend the elevator to the high position.
     * 
     * @param drivetrain the drivetrain subsystem
     * @param elevator   the elevator subsystem
     * @return the auto command
     */
    public static Command prepareScore(Drivetrain drivetrain, Elevator elevator) {
        return Commands.sequence(
                Commands.parallel(
                        drivetrain.tankDriveCommand(() -> .5, () -> .5).withTimeout(Seconds.of(2.0)),
                        elevator.moveToPositionCommand(() -> Elevator.Constants.POSITION.INTAKE)),
                elevator.moveToPositionCommand(() -> Elevator.Constants.POSITION.HIGH));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
