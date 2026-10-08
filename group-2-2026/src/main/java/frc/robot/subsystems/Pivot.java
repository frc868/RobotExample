package frc.robot.subsystems;

// IMPORTANT: Importing units.Units statically with a star imports all units
import static edu.wpi.first.units.Units.*;
// Does the same thing for this file's OWN constants so we don't have to put Constants. all the time.
import static frc.robot.subsystems.Pivot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    public static class Constants {
        /** Where the motor is on the CAN bus */
        public static final int CAN_ID = 1;

        /** How much current the motor is allowed to draw */
        public static final Current CURRENT_LIMIT = Amps.of(10);

        /** WPIlib stuff that allows the motor to be inverted or not */
        public static final InvertedValue INVERSION = InvertedValue.Clockwise_Positive;
    }

    // Initializes motor objects to be used later
    // Initial values can also be set in constructor
    private final TalonFX motor = new TalonFX(CAN_ID);
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    
    // this doesn't really matter
    //private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public Pivot() {
        config.MotorOutput
            .withInverted(INVERSION)
            .withNeutralMode(NeutralModeValue.Brake); // What the motor should do when not moving
        config.CurrentLimits
            .withStatorCurrentLimit(CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // apply current limit, MUST be enabled
        
        // Applies all that stuff
        motor.getConfigurator().apply(config);
    }

    // Always include comments (javadocs) above commands!
    /**
     * This command runs the motor counterclockwise at 12 Volts
     * 
     * @return the command to run itself
     */
    public Command spinMotor() {
        return startEnd(() -> {
            motor.setVoltage(12);
        }, () -> {
            motor.setVoltage(0); // Usually, we would make another command called "stop()" to reuse elsewhere.
        }).withName("pivot.spinMotor");
    }

}
