package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Constants.POSITION;

import static frc.robot.subsystems.Elevator.Constants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** An example of a motion-controlled elevator */
public class Elevator extends SubsystemBase {
    /**
     * The variables that will remain the same during the whole time your robot
     * is enabled. We put constants in SCREAMING_SNAKE_CASE to differentiate them
     * from other variables.
     */
    public static final class Constants {
        /** The common positions of the elevator in distance from the FLOOR */
        public static enum POSITION {
            MIN(Inches.of(6.0)),
            MAX(Feet.of(3.5)),
            INTAKE(Inches.of(8.0)),
            MID(Feet.of(1.0)),
            HIGH(Feet.of(2.75));

            public final Distance value;

            private POSITION(Distance value) {
                this.value = value;
            }
        }

        /**
         * The CAN ID of the motor, representing the identity of the device on the
         * CAN bus. Each integer can point to only one device, and this must be
         * configured using Tuner X software.
         */
        public static final int CAN_ID = 3;
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

        /** The direction the motor should spin with positive voltage. */
        public static final InvertedValue INVERSION = InvertedValue.CounterClockwise_Positive;

        /** The gear ratio between the motor and the drums. */
        public static final double GEAR_RATIO = 868.0 / 761.0;

        /** The diameter of the elevator drums. */
        public static final Distance DRUM_DIAMETER = Inches.of(3.0);
        /** The circumference of the drums, as calculated from the diameter */
        public static final Distance DRUM_CIRCUMFERENCE = DRUM_DIAMETER.times(Math.PI);

        /**
         * The number of volts needed to overcome the static friction of our mechanism.
         */
        public static final double kS = 0.0; // TODO find good values using Reca.lc or manual tuning with robot
        /** The number of volts needed to maintain a constant velocity. */
        public static final double kV = 0.0;
        /** The number of volts needed to overcome gravity. */
        public static final double kG = 0.0;
        /** The number of volts needed to induce an acceleration. */
        public static final double kA = 0.0;
        /**
         * How aggressively the mechanism will try to reach it's destination, basically.
         */
        public static final double kP = 0.0;
        /** Not used in FRC. */
        public static final double kI = 0.0;
        /** How much changes in motion are dampened (reduced), basically. */
        public static final double kD = 0.0;

        /** The acceptable linear distance from the goal position. */
        public static final Distance TOLERANCE = Inches.of(1.0);

        /** The maximum velocity of the motion profile */
        public static final LinearVelocity MAX_VELOCITY = FeetPerSecond.of(1.0);
    }

    /**
     * The object representing the motor's TalonFX controller, which is how we
     * interface with it.
     */
    private final TalonFX motor;

    /** The position for the intake to target. */
    private Distance goalPosition = POSITION.MIN.value;

    /** A control mode for setting a voltage [-12, 12] on the motor. */
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    /** The control mode for closed-loop motion control. */
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(
            Rotations.of(goalPosition.in(Meters) / DRUM_CIRCUMFERENCE.in(Meters)));

    /** The constructor for the elevator */
    public Elevator() {
        // Assigning our devices to their CAN IDs and buses
        motor = new TalonFX(CAN_ID, CAN_BUS);

        // The config objects for the motors.
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Configure everything we set in the constants on the motors, including
        // inversion, current limit, neutral mode, and motion control constants
        motorConfig.MotorOutput
                .withInverted(INVERSION)
                .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this
        motorConfig.CurrentLimits
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true); // Don't change this
        motorConfig.Feedback.withSensorToMechanismRatio(GEAR_RATIO);
        motorConfig.Slot0
                .withKS(kS)
                .withKV(kV)
                .withKG(kG)
                .withKA(kA)
                .withKP(kP)
                .withKI(kI)
                .withKD(kD);
        motorConfig.MotionMagic.withMotionMagicCruiseVelocity(
                RotationsPerSecond.of(MAX_VELOCITY.in(MetersPerSecond) / DRUM_CIRCUMFERENCE.in(Meters)));
        motorConfig.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(
                        Rotations.of(POSITION.MAX.value.in(Meters) / DRUM_CIRCUMFERENCE.in(Meters)))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(
                        Rotations.of(POSITION.MIN.value.in(Meters) / DRUM_CIRCUMFERENCE.in(Meters)))
                .withReverseSoftLimitEnable(true);
        motor.getConfigurator().apply(motorConfig);

        // Make the default command to continue moving to the goal so that even when a
        // command has finished the elevator stays where we leave it.
        setDefaultCommand(moveToGoalCommand());
    }

    /**
     * Get the current LINEAR position of the elevator from the FLOOR.
     * 
     * @return the position
     */
    public Distance getPosition() {
        return Meters.of(motor.getPosition().getValue().in(Rotations) * DRUM_CIRCUMFERENCE.in(Meters));
    }

    /**
     * Creates a command that allows commanding an open-loop speed, stopping on end.
     * 
     * @param speed the desired speed [-1.0, 1.0]
     * @return the command
     */
    public Command setSpeedCommand(DoubleSupplier speed) {
        return runEnd(
                () -> motor.setControl(voltageRequest.withOutput(speed.getAsDouble() * 12.0)),
                () -> motor.setControl(voltageRequest.withOutput(Volts.zero())))
                .withName("elevator.setSpeed");
    }

    /**
     * Creates a command that drives the elevator toward the current goal using
     * motion control.
     * 
     * @return the command
     */
    public Command moveToGoalCommand() {
        return run(() -> {
            motor.setControl(positionRequest
                    .withPosition(Rotations.of(goalPosition.in(Meters) / DRUM_CIRCUMFERENCE.in(Meters))));
        }).withName("elevator.moveToPosition");
    }

    /**
     * Creates a command that allows for moving the elevator to a predefined
     * constant {@link POSITION}.
     * 
     * @param positionSupplier the desired position
     * @return the command
     */
    public Command moveToPositionCommand(Supplier<POSITION> positionSupplier) {
        return Commands.runOnce(() -> {
            goalPosition = positionSupplier.get().value;
        }).andThen(moveToGoalCommand()).until(this::atGoal).withName("elevator.moveToPosition");
    }

    /**
     * Creates a command that allows for moving the elevator to any given position.
     * 
     * @param positionSupplier the desired position
     * @return the command
     */
    public Command moveToArbitraryPositionCommand(Supplier<Distance> positionSupplier) {
        return Commands.runOnce(() -> {
            goalPosition = positionSupplier.get();
        }).andThen(moveToGoalCommand()).until(this::atGoal).withName("elevator.moveToPosition");
    }

    /**
     * Creates a command that sets the elevator motor's relative encoder to the
     * minimum position, ensuring accurate measurements. YOU MUST RESET POSITION
     * AFTER EACH POWER CYCLE.
     * 
     * @return the command
     */
    public Command resetPositionCommand() {
        return runOnce(() -> {
            motor.setPosition(Rotations.of(POSITION.MIN.value.in(Meters) / DRUM_CIRCUMFERENCE.in(Meters)));
        }).withName("elevator.resetPositionCommand");
    }

    /**
     * Get whether the elevator is within tolerance of its goal position.
     * 
     * @return true if yes, false otherwise
     */
    public boolean atGoal() {
        return getPosition().isNear(goalPosition, TOLERANCE);
    }

    /**
     * The periodic method for the elevator, which is called by the
     * {@link edu.wpi.first.wpilibj2.command.CommandScheduler CommandScheduler}
     * every iteration (by default every .02s).
     */
    @Override
    public void periodic() {
        // Log current and voltage of the motor
        SmartDashboard.putNumber("elevator/motor/voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator/motor/current", motor.getStatorCurrent().getValueAsDouble());

        // Log subsystem data and motion control related info
        SmartDashboard.putBoolean("elevator/atGoal", atGoal());
        SmartDashboard.putNumber("elevator/positionMeters", getPosition().in(Meters));
    }
}
