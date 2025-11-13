package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class Controls {
    /** The method that defines the controls for your driver 
     * @param drivetrain an instance of the Drivetrain class
     * @param port the port your Xbox controller is connected to
     */
    public static void configureControls(Drivetrain drivetrain, Shooter shooter, int port) {
        /** The Xbox controller that your driver will use to control the robot */
        CommandXboxController controller = new CommandXboxController(port);

        // A default command is what the subsystem runs when it's not doing anything else,
        // and since your drivetrain will always be driving we make it the default
        drivetrain.setDefaultCommand(drivetrain.tankDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getRightY()
        ));

        // Spin shooter when A is pressed
        controller.a().whileTrue(shooter.spinAtVelocityCommand(() -> 10));

        // Put any additional controls your driver will use inside of this method
    }
}
