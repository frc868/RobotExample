package frc.robot.subsystems.examples;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter1 extends SubsystemBase{
    public static final class SHOOTER {
        public static final int CAN_ID = 4;
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
    // public static final class BLOCK_WHEEL {
    //     public static final int CAN_ID = 5;
    //     /**
    //      * The current limit in amps, representing the maximum power the motor can output.
    //      * Do not increase this number above 40.
    //      */
    //     public static final double CURRENT_LIMIT = 30; // Amps
    //     /**
    //      * Whether this drive motor should rotate clockwise or counterclockwise (reversed), 
    //      * i.e. when you pass in +12V it is converted to -12V before being sent to the motor
    //      */
    //     public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    //     // Other constants, such as motion control constants, go here
    // }
    public static enum shooterLevels {
        LOW(2.5),
        MEDIUM(3.5),
        HIGH(4.5),
        DEFAULT(0.5),
        SUPERHIGH(6.5),
        STOP(0.0);

        public final double value;
        
        private shooterLevels(double value){
            this.value = value;
        }
    }
    private TalonFX shooter = new TalonFX(SHOOTER.CAN_ID);
    // private TalonFX blockWheel = new TalonFX(BLOCK_WHEEL.CAN_ID);
    private TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
    // private TalonFXConfiguration blockWheelConfig = new TalonFXConfiguration();

    public Shooter1() {
        wheelConfig.MotorOutput
            .withInverted(SHOOTER.INVERTED)
            .withNeutralMode(NeutralModeValue.Coast); // Probably don't change this unless it's for a flywheel type system
        wheelConfig.CurrentLimits
            .withStatorCurrentLimit(SHOOTER.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
            shooter.getConfigurator().apply(wheelConfig);

        // blockWheelConfig.MotorOutput
        //     .withInverted(SHOOTER.INVERTED)
        //     .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel type system
        // blockWheelConfig.CurrentLimits
        //     .withStatorCurrentLimit(SHOOTER.CURRENT_LIMIT)
        //     .withStatorCurrentLimitEnable(true); // Don't change this
        //     blockWheel.getConfigurator().apply(wheelConfig);
        System.out.println("Motor Initialized");
    }
    
    /**Runs shooter motor with enum
     * 
     * @param shooterLevelSupplier LOW, MEDIUM, HIGH
     * @return the command
     */
/*
    public Command shooterMotor(Supplier<shooterLevels> shooterLevelSupplier){
            return run(
                () -> {
                    shooter.setVoltage(shooterLevelSupplier.get().value);       
            }).withName("shooter.shooterMotor");
    
    }
            */



    // public Command shooterMotor2(Supplier<shooterLevels> shooterLevelSupplier){
    //     return startEnd(()->{
    //         shooter.setVoltage(shooterLevelSupplier.get().value);       
    //     },()-> {
    //         stop();
    //     }).withName("shooter.stopShooterMotor");
    // }

    public Command shooterMotor3(Supplier<shooterLevels> shooterLevelSupplier){
        return startEnd(()->{
            shooter.setVoltage(shooterLevelSupplier.get().value);       
        },()-> {
            stop();
        }).withName("shooter.stopShooterMotor");
    }


    public void stop(){
        shooter.setVoltage(0.0);
        // blockWheel.setVoltage(0.0);
    }
    /*
        public Command blockWheelMotor(){
        return run(() -> {
            blockWheel.setVoltage(5.0);
        }).withName("shooter.blockWheelMotor");
    }
        */

    // public Command runBlockWheelCommand(){
    //     return startEnd(()->{
    //         blockWheel.setVoltage(5.0);
    //     },()-> {
    //         blockWheel.setVoltage(0.0);
    //     }).withName("shooter.runBeltCommand");
    // }

        @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/shooterMotor/Position",shooter.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("shooter/shooterMotor/Current",shooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooter/shooterMotor/Voltage",shooter.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter/shooterMotor/Velocity",shooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("shooter/shooterMotor/Acceleration",shooter.getAcceleration().getValueAsDouble());

    }


    //public Command driveBeltForward(){
    //    return run(() -> {
    //        blockWheel.setVoltage(5.0);
    //    }).withName("belt.driveBeltForward");
    //}
    //public Command driveBeltBackward(){
    //    return run(() -> {
    //        blockWheel.setVoltage(-5.0);
    //    }).withName("belt.driveBeltBackward");
    //}

    
    // public Command baseMotor(DoubleSupplier mainInput){
    //     return run(() -> {
    //         maindrive.tankDrive(mainInput.getAsDouble(), mainInput.getAsDouble());
    //     }).withName("drivetrain.baseMotor");
    // }

    }