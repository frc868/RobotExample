package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.Drivetrain.Constants.*;

/**
 * An example of a basic tank drive robot subsystem controlled by an Xbox
 * controller.
 */
public class Drivetrain extends SubsystemBase {
    /**
     * The variables that will remain the same during the whole time your robot
     * is enabled. We put constants in SCREAMING_SNAKE_CASE to differentiate them
     * from other variables.
     */
    public static final class Constants {
        /**
         * The CAN ID of the left motor, representing the identity of the device on the
         * CAN bus. Each integer can point to only one device, and this must be
         * configured using Tuner X software.
         */
        public static final int CAN_ID_LEFT = 1;
        /**
         * The CAN ID of the right motor, representing the identity of the device on the
         * CAN bus. Each integer can point to only one device, and this must be
         * configured using Tuner X software.
         */
        public static final int CAN_ID_RIGHT = 2;
        /**
         * The CAN bus of the device, representing the physical device the daisy chain
         * originates from.
         */
        public static final String CAN_BUS = "rio";

        /**
         * The current limit in amps, representing the maximum torque the motor can
         * output. Do not increase this number above 40 for now.
         */
        public static final Current CURRENT_LIMIT = Amps.of(30);

        /** The direction the left drive motor should spin with positive voltage. */
        public static final InvertedValue INVERSION_LEFT = InvertedValue.CounterClockwise_Positive;
        /** The direction the right drive motor should spin with positive voltage. */
        public static final InvertedValue INVERSION_RIGHT = InvertedValue.Clockwise_Positive;
    }

    /**
     * The object representing the left motor's TalonFX controller, which is how we
     * interface with it.
     */
    private final TalonFX leftMotor;
    /**
     * The object representing the right motor's TalonFX controller, which is how we
     * interface with it.
     */
    private final TalonFX rightMotor;

    /** A control mode for setting a voltage [-12, 12] on the left motor. */
    private final VoltageOut voltageRequestLeft = new VoltageOut(0.0);
    /** A control mode for setting a voltage [-12, 12] on the right motor. */
    private final VoltageOut voltageRequestRight = new VoltageOut(0.0);

    /**
     * The differential drive uses the difference in rotation speed between
     * two sides to rotate or run them at the same speed to translate
     */
    private DifferentialDrive drivetrain;

    /**
     * The constructor for the drivetrain object, including this whole file.
     * Call this in RobotContainer.java to create an object of this class!
     */
    public Drivetrain() {
        // Assing our motors to their CAN IDs and buses
        leftMotor = new TalonFX(CAN_ID_LEFT, CAN_BUS);
        rightMotor = new TalonFX(CAN_ID_RIGHT, CAN_BUS);

        // The config objects for the motors.
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();

        // Configure everything we set in the constants on the drive motors, including
        // inversion, current limit, and neutral mode
        leftConfig.MotorOutput
                .withInverted(INVERSION_LEFT)
                .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel
        leftConfig.CurrentLimits
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true); // Don't change this
        leftMotor.getConfigurator().apply(leftConfig);

        rightConfig.MotorOutput
                .withInverted(INVERSION_RIGHT)
                .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel
        rightConfig.CurrentLimits
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true); // Don't change this
        rightMotor.getConfigurator().apply(rightConfig);

        // Set up our differential drive
        drivetrain = new DifferentialDrive(
                // Set voltage instead of speed to prevent issues as battery voltage drops
                speed -> leftMotor.setControl(voltageRequestLeft.withOutput(speed * 12)),
                speed -> rightMotor.setControl(voltageRequestRight.withOutput(speed * 12)));

        System.out.println("Drivetrain initialized!");
    }

    /**
     * A command to drive the robot tank, meaning one joystick is responsible
     * for left wheel rotation and the other for right wheel rotation
     * 
     * @param left  left side speed
     * @param right right side speed
     * @return the command to run
     */
    public Command tankDriveCommand(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            drivetrain.tankDrive(left.getAsDouble(), right.getAsDouble());
        }).withName("drivetrain.tankDrive");
    }

    /**
     * The periodic method for the drivetrain, which is called by the
     * {@link edu.wpi.first.wpilibj2.command.CommandScheduler CommandScheduler}
     * every interation (by default every .02s)
     */
    @Override
    public void periodic() {
        // Log current and voltage of both drive motors
        SmartDashboard.putNumber("drivetrain/leftMotor/voltage", leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/leftMotor/current", leftMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("drivetrain/rightMotor/voltage", rightMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/rightMotor/current", rightMotor.getStatorCurrent().getValueAsDouble());

        // DO NOT control any motors here
    }
}
