package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class Controls {
    /**
     * The method that defines the controls for your driver
     * 
     * @param drivetrain an instance of the Drivetrain class
     * @param port       the port your Xbox controller is connected to
     */
    public static void configureControls(Drivetrain drivetrain, Elevator elevator, int port) {
        /** The Xbox controller that your driver will use to control the robot */
        CommandXboxController controller = new CommandXboxController(port);

        // A default command is what the subsystem runs when it's not doing anything
        // else, and since your drivetrain will always be driving we make it the default
        drivetrain.setDefaultCommand(drivetrain.tankDriveCommand(
                () -> -controller.getLeftY(), () -> -controller.getRightY()));

        // Set the controls for the elevator's motion controlled functions
        controller.a().whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.POSITION.INTAKE));
        controller.x().whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.POSITION.MID));
        controller.y().whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.POSITION.HIGH));
        controller.back().onTrue(elevator.resetPositionCommand());

        // Put any additional controls your driver will use inside of this method
    }
}
