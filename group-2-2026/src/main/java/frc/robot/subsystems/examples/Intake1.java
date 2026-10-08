package frc.robot.subsystems.examples;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.examples.Intake1.Constants.*;
import frc.robot.subsystems.examples.Intake1.Constants.Voltage;

//Subsystem base is a class and it gives Intake all the properties of that class, so it's like a template for a subsystem
public class Intake1 extends SubsystemBase{
    public final class Constants {

        public static enum Voltage {
            NORMAL(7.0),
            REVERSE(-5.0);

            public final double value;

            private Voltage(double value) {
                this.value = value;
            }
        }
        
        public static final class INTAKE_MOTOR {
             /**CAN ID for Intake Motor */
            private static final int CAN_ID = 3;

            /**Current Limit for Intake Motor, measured in amps */
            private static final double CURRENT_LIMIT = 20.0;

            /**Sets which way the motor will rotate*/
            private static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

            /**Voltage the intake will run at*/
            private static final double RUN_VOLTAGE = 12.0;

            /**Voltage to reverse the intake*/
            private static final double REVERSE_VOLTAGE = 5.0;
        }
    }

    /**
     * The object for the motor's controller
     * (Esentially meaning just the motor itself)
     */
    private TalonFX Motor = new TalonFX(INTAKE_MOTOR.CAN_ID);
    /**
     * The object for the configuration part of the motor
     * (The different things like inversions and current limits to be applied to the motor)
     */
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();


    /**Motor configs
     * This is where you put all your configurations for your motors
     */
    public Intake1() {
        /**Sets motors to be on brake so they're harder to move (Shocking right?) */
        Motor.setNeutralMode(NeutralModeValue.Brake);

        /**Sets the motor to spin clockwise/counterclockwise */
        motorConfig.MotorOutput.Inverted = INTAKE_MOTOR.INVERTED;
        /**Sets hard limit on motor current */
        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.INTAKE_MOTOR.CURRENT_LIMIT;
        /**Applies the limit to the stator which is some part of the motor*/
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /**Applies the configs to the actual motor */
        Motor.getConfigurator().apply(motorConfig);
    }
        /**Runs rollers forward at forward voltage variable in Intake
         * @param Voltage for shoot commmand
         * @return The command
        */
        public Command intakeCommand(Supplier<Voltage> voltageSupplier) {
        return startEnd (() ->{
            Motor.setVoltage(voltageSupplier.get().value);
        }, () -> {
            stop();
        }).withName("Intake.intakeCommand");
        }

    /**Runs rollers forward at forward voltage variable in Intake
     * @return The command
    */
        public Command runRollersCommand() {
            //Calls a function the states it has a start and end
        return startEnd (() ->{
            System.out.println("Running rollers");
            //When the starting action (pressing a button in our case) is completed,
            //it sets the voltage to the intake motor whatever the RUN_VOLTAGE is
            Motor.setVoltage(Constants.INTAKE_MOTOR.RUN_VOLTAGE);
        }, () -> {
            //When the ending action (unpressing the button in our case)
            //is completed, it runs the method stop() which is somewhere in code
            //and just sets the voltage of the motor to 0.0,
            //If you're on WPILIB you can hover over stop() and see the javadoc on it
            stop();
        }).withName("Intake.runRollersCommand");
    }



    /**Runs rollers in reverse at reverse voltage variable in Intake
     * @return The command
    */
    public Command reverseRollersCommand() {
        return startEnd (() ->{
            Motor.setVoltage(-Constants.INTAKE_MOTOR.REVERSE_VOLTAGE);
        }, () -> {
            stop();
        }).withName("Intake.reverseRollersCommand");
    }
    public Command runRollersSlowCommand() {
        return startEnd (() ->{
            Motor.setVoltage(7.0);
        }, () -> {
            stop();
        }).withName("Intake.reverseRollersCommand");
    }

    /**Sets the motor voltage to 0 
     * @return The command
    */
    public void stop(){
        Motor.setVoltage(0.0);
    }

    
        @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/intakeMotor/Voltage", Motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake/intakeMotor/Current", Motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("intake/intakeMotor/Velocity", Motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intake/intakeMotor/Acceleration", Motor.getAcceleration().getValueAsDouble());
    }

}