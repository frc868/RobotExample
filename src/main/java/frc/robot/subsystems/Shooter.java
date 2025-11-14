package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Constants.MOTOR;

import static frc.robot.subsystems.Shooter.Constants.*;

/** An example of a shooter robot subsystem with velocity motion control */
public class Shooter extends SubsystemBase {
    public static final class Constants {
         /** Constants for the singular motor of this shooter */
        public static final class MOTOR {
            /** 
             * A CAN ID is the identity of a device on the robot's
             * CAN Bus. Each integer can point to only one device,
             * and this must be configured using Tuner X software 
             * (for Krakens) 
             */
            public static final int CAN_ID = 3;
            /**
             * The current limit in amps, representing the maximum power the motor can output.
             * Do not increase this number above 40.
             */
            public static final double CURRENT_LIMIT = 15; // Amps
            /**
             * Whether this drive motor should rotate clockwise or counterclockwise (reversed), 
             * i.e. when you pass in +12V it is converted to -12V before being sent to the motor
             */
            public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
            // Other constants, such as motion control constants, go here
        }
        /** The number of volts needed to overcome the static friction of our mechanism */
        public static final double kS = 0.0; // TODO find good values using Reca.lc or manual tuning with robot
        /** The number of volts needed to maintain a constant velocity */
        public static final double kV = 0.0;
        /** The number of volts needed to induce an acceleration */
        public static final double kA = 0.0;
        /** How aggressively the mechanism will try to reach it's destination, basically */
        public static final double kP = 0.0;
        /** Not used in FRC */
        public static final double kI = 0.0;
        /** How much changes in motion are dampened (reduced), basically */
        public static final double kD = 0.0;
        /** The maximum acceleration (change in velocity) of the motion profile, in m/s/s */
        public static final double MAX_ACCELERATION = 0.0;
        /** The maximum jerk (change in acceleration) of the motion profile, in m/s/s/s */
        public static final double MAX_JERK = 0.0;

        // The gear ratio betwen the motor and the wheels; used to convert encoders to meaningful units
        public static final double GEAR_RATIO = 15; // TODO
    }
    /** The object for the motor's controller */
    private TalonFX motor = new TalonFX(MOTOR.CAN_ID);
    /** The configuration object for the motor */
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    /** 
     * The velocity motion magic controller, which handles feedback, feedforward, and motion 
     * profiling to achieve a desired velocity 
     */
    private MotionMagicVelocityVoltage mmRequest = new MotionMagicVelocityVoltage(0);

    /**
     * The constructor for the shooter object, including this whole file.
     * Call this in RobotContainer.java to create an instance of this code!
     * @return an instance of the shooter class
     */
    public Shooter() {
        // Configure everything we set in the constants on the motor, including inversion, 
        // current limit, neutral mode, and motion control gains
        motorConfig.MotorOutput
            .withInverted(MOTOR.INVERTED)
            .withNeutralMode(NeutralModeValue.Coast); // Coast so it doesn't slam to a stop after each launch
        motorConfig.CurrentLimits
            .withStatorCurrentLimit(MOTOR.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
        motorConfig.Feedback.withSensorToMechanismRatio(GEAR_RATIO);
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kA = kA;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = MAX_JERK;
        motor.getConfigurator().apply(motorConfig);

        System.out.println("Shooter initialized!");
    }

    /**
     * Spin the shooter at a given velocity
     * @param velocity in RPS
     * @return the command to run
     */
    public Command spinAtVelocityCommand(DoubleSupplier velocity) {
        return run(() -> {
            motor.setControl(mmRequest.withVelocity(velocity.getAsDouble()));
        }).withName("shooter.spinAtVelocityCommand");
    }

    /** 
     * The periodic method for the shooter, which is called by the
     * {@link edu.wpi.first.wpilibj2.command.CommandScheduler CommandScheduler} every interation (by default every .02s) 
     */
    @Override
    public void periodic() {
        // Log current and voltage of our shooter motor
        SmartDashboard.putNumber("shooter/motor/voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter/motor/current", motor.getStatorCurrent().getValueAsDouble());
        
        // DO NOT control any motors here
    }
}
