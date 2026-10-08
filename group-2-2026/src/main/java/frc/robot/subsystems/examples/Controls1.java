package frc.robot;


// IGNORE ALL THE ERRORS HERE, it's because this is supposed to be in the robot folder



//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Constants.Voltage;

public class Controls1 {
     public static final class Constants {
        public static final class Teleop {
            //Constants go here
        }
    }

    public static void configureControls(Drivetrain drivetrain, Intake intake, Shooter shooter, int port) {
        /** The Xbox controller that your driver will use to control the robot */
        CommandXboxController controller = new CommandXboxController(port);
        
        controller.leftTrigger().whileTrue(shooter.shootCommand(()->Voltage.HIGH));
        controller.rightTrigger().whileTrue(intake.runRollersCommand());
        controller.leftBumper().whileTrue(shooter.shootCommand(()->Voltage.REVERSE));
        controller.rightBumper().whileTrue(intake.reverseRollersCommand());
        
        controller.a().whileTrue(shooter.shootCommand(()->Voltage.MEDIUM));
        //Controls for drive
        
        drivetrain.setDefaultCommand(drivetrain.tankDrive(
            () -> -controller.getRightY(),
            () -> -controller.getLeftY()
        ));
        
    }

    public static void configureTestingControls(Drivetrain drivetrain, Intake intake, Shooter shooter, int port) {
        /** The Xbox controller that your driver will use to control the robot */
        CommandXboxController controller = new CommandXboxController(port);

        
         controller.x().whileTrue(drivetrain.setTestingVoltageCommand());
         controller.y().onTrue(drivetrain.incrementVoltageCommand(0.01));
         controller.b().onTrue(drivetrain.incrementVoltageCommand(-0.01));

         
         controller.a().onTrue(
             intake.runRollersCommand().repeatedly().withTimeout(5)
             .andThen(shooter.shootCommand(() -> Voltage.MEDIUM).withTimeout(0.7))
             .andThen(shooter.shootCommand(() -> Voltage.REVERSE).withTimeout(0.7))
             .andThen(shooter.shootCommand(() -> Voltage.HIGH).withTimeout(1))
         );
        
        
        // intake.runRollersCommand().withTimeout(0.5),
        // shooter.shootCommand(()-> Voltage.MEDIUM).withTimeout(0.7),
        // shooter.shootCommand(()-> Voltage.REVERSE).withTimeout(0.7),
        // shooter.shootCommand(()-> Voltage.HIGH).withTimeout(1.0)
        

        //controller.a().onTrue(drivetrain.changeGainCommand("kS", 0.001));
        //controller.b().onTrue(drivetrain.changeGainCommand("kS", -0.001));
        //kV = (Voltage - kS) / velocity(m/s)

        //controller.a().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        //controller.b().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        //controller.x().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        //controller.y().whileTrue(shooter.sysIdDynamic(Direction.kReverse));

    }
}



        //Runs shooter on high voltage for 5 seconds
        //controller.b().whileTrue(shooter.shootCommand(()->Voltage.HIGH).repeatedly().withTimeout(5));