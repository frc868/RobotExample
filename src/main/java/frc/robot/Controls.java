package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Controls {
    // Put any and all controls your driver will use inside of this method
    public static void configureControls(Drivetrain drivetrain, int port) {
        // Construct the controller
        CommandXboxController controller = new CommandXboxController(port);

        // A default command is what the subsystem runs when it's not doing anything else,
        // and since your drivetrain will always be driving we make it the default
        drivetrain.setDefaultCommand(drivetrain.tankDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getRightY()
        ));
    }
}
