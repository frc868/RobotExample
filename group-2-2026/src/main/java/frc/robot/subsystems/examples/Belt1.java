package frc.robot.subsystems.examples;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Belt1 extends SubsystemBase{
    public static final class BELT {
        public static final int CAN_ID = 3;
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
    private TalonFX belt = new TalonFX(BELT.CAN_ID);
    private TalonFXConfiguration finalConfig = new TalonFXConfiguration();

    public Belt1() {
        finalConfig.MotorOutput
            .withInverted(BELT.INVERTED)
            .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel type system
        finalConfig.CurrentLimits
            .withStatorCurrentLimit(BELT.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
            belt.getConfigurator().apply(finalConfig);

        System.out.println("Motor Initialized");
    }
    
    public Command driveBeltForward(){
        return run(() -> {
            belt.setVoltage(5.0);
        }).withName("belt.driveBeltForward");
    }

    public Command driveBeltBackward(){
        return run(() -> {
            belt.setVoltage(-5.0);
        }).withName("belt.driveBeltBackward");
    }
    /**
     * runs belt when holding button
     * @return the command
     */
    public Command runBeltCommand(){
        return startEnd(()->{
            belt.setVoltage(5.0);
        },()-> {
            beltStop();
        }).withName("shooter.runBeltCommand");
    }
    
    public Command runBeltReverseCommand(){
        return startEnd(()->{
            belt.setVoltage(-5.0);
        },()-> {
            beltStop();
        }).withName("shooter.runBeltCommand");
    }

    public void beltStop(){
        belt.setVoltage(0.0);
    }
        @Override
    public void periodic() {
 
        SmartDashboard.putNumber("belt/belt/Position",belt.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("belt/belt/Current",belt.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("belt/belt/Voltage",belt.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("belt/belt/Velocity",belt.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("belt/belt/Acceleration",belt.getAcceleration().getValueAsDouble());
    }

    
    // public Command baseMotor(DoubleSupplier mainInput){
    //     return run(() -> {
    //         maindrive.tankDrive(mainInput.getAsDouble(), mainInput.getAsDouble());
    //     }).withName("drivetrain.baseMotor");
    // }


}