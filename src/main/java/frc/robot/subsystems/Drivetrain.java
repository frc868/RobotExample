package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Constants.LEFT_MOTOR;
import frc.robot.subsystems.Drivetrain.Constants.RIGHT_MOTOR;

/** An example of a tank drive robot subsystem controlled by an xbox controller */
public class Drivetrain extends SubsystemBase {
    /** 
     * The variables that will remain the same during the whole time your robot
     * is enabled. In TechHOUNDS, we put constants in SCREAMING_SNAKE_CASE
     * to differentiate them from other variables
     */
    public static final class Constants {
        /** Constants for the left motor of your tank drive robot */
        public static final class LEFT_MOTOR {
            /** 
             * A CAN ID is the identity of a device on the robot's
             * CAN Bus. Each integer can point to only one device,
             * and this must be configured using Tuner X software 
             * (for Krakens) 
             */
            public static final int CAN_ID = 1;
            /**
             * The current limit in amps, representing the maximum power the motor can output.
             * Do not increase this number above 40.
             */
            public static final double CURRENT_LIMIT = 30; // Amps
            /**
             * Whether this drive motor should rotate clockwise or counterclockwise (reversed), 
             * i.e. when you pass in +12V it is converted to -12V before being sent to the motor
             */
            public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
            // Other constants, such as motion control constants, go here

        }
        /** Constants for the right motor of your tank drive robot */
        public static final class RIGHT_MOTOR {
            /** 
             * A CAN ID is the identity of a device on the robot's
             * CAN Bus. Each integer can point to only one device,
             * and this must be configured using Tuner X software 
             * (for Krakens) 
             */
            public static final int CAN_ID = 2;
            /**
             * The current limit in amps, representing the maximum power the motor can output.
             * Do not increase this number above 40.
             */
            public static final double CURRENT_LIMIT = 30; // Amps
            /**
             * Whether this drive motor should rotate clockwise or counterclockwise (reversed), 
             * i.e. when you pass in +12V it is converted to -12V before being sent to the motor
             */
            public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
            // Other constants, such as motion control constants, go here
        }
    }

    /**
     * The object for the left motor's controller
     */
    private TalonFX leftMotor = new TalonFX(LEFT_MOTOR.CAN_ID);
    /**
     * The configuration object for the left motor
     */
    private TalonFXConfiguration leftConfig = new TalonFXConfiguration();
   /**
     * The object for the right motor's controller
     */
    private TalonFX rightMotor = new TalonFX(RIGHT_MOTOR.CAN_ID);
    /**
     * The configuration object for the right motor
     */
    private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    /**
     * The differential drive uses the difference inrotation speed between
     * two sides to rotate or run them at the same speed to translate
     */
    private DifferentialDrive drivetrain = new DifferentialDrive(
        x -> leftMotor.setVoltage(x * 12), // Set voltage instead of speed to prevent
        x -> rightMotor.setVoltage(x * 12) // issues when battery voltage drops
    );

    /**
     * The constructor for the drivetrain object, including this whole file.
     * Call this in RobotContainer.java to create an instance of this code!
     * @return an instance of the drivetrain object
     */
    public Drivetrain() {
        // Configure everything we set in the constants on the drive motors, including inversion, current limit, and neutral mode
        leftConfig.MotorOutput
            .withInverted(LEFT_MOTOR.INVERTED)
            .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel type system
        leftConfig.CurrentLimits
            .withStatorCurrentLimit(LEFT_MOTOR.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
        leftMotor.getConfigurator().apply(leftConfig);
        
        rightConfig.MotorOutput
            .withInverted(RIGHT_MOTOR.INVERTED)
            .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel type system
        rightConfig.CurrentLimits
            .withStatorCurrentLimit(RIGHT_MOTOR.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
        rightMotor.getConfigurator().apply(rightConfig);
        
        System.out.println("Drivetrain initialized!");
    }

    public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            drivetrain.tankDrive(left.getAsDouble(), right.getAsDouble());
        });
    }
}
