package frc.robot.subsystems.examples;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain1 extends SubsystemBase{
    public static final class CONSTANT_RIGHT {
        //Make sure to ask what this should be and change it
        public static final int CAN_ID = 1;
        /**
         * The current limit in amps, representing the maximum power the motor can output.
         * Do not increase this number above 40.
         */
        public static final double CURRENT_LIMIT = 25; // Amps
        /**
         * Whether this drive motor should rotate clockwise or counterclockwise (reversed), 
         * i.e. when you pass in +12V it is converted to -12V before being sent to the motor
         */
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
        // Other constants, such as motion control constants, go here
    }
    public static final class CONSTANT_LEFT {
        //Make sure to ask what this should be and change it
        public static final int CAN_ID = 2;
        /**
         * The current limit in amps, representing the maximum power the motor can output.
         * Do not increase this number above 40.
         */
        public static final double CURRENT_LIMIT = 25; // Amps
        /**
         * Whether this drive motor should rotate clockwise or counterclockwise (reversed), 
         * i.e. when you pass in +12V it is converted to -12V before being sent to the motor
         */
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        // Other constants, such as motion control constants, go here
    }
    private TalonFX rightMotor = new TalonFX(CONSTANT_RIGHT.CAN_ID);
    private TalonFX leftMotor = new TalonFX(CONSTANT_LEFT.CAN_ID);

    private TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    public Drivetrain1() {
        // Configure everything we set in the constants on the drive motors, including inversion, current limit, and neutral mode
        leftConfig.MotorOutput
            .withInverted(CONSTANT_LEFT.INVERTED)
            .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel type system
        leftConfig.CurrentLimits
            .withStatorCurrentLimit(CONSTANT_LEFT.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
        leftMotor.getConfigurator().apply(leftConfig);
        
        rightConfig.MotorOutput
            .withInverted(CONSTANT_RIGHT.INVERTED)
            .withNeutralMode(NeutralModeValue.Brake); // Probably don't change this unless it's for a flywheel type system
        rightConfig.CurrentLimits
            .withStatorCurrentLimit(CONSTANT_RIGHT.CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true); // Don't change this
        rightMotor.getConfigurator().apply(rightConfig);
        
        System.out.println("Drivetrain initialized!");
    }
    
    private DifferentialDrive drivetrain = new DifferentialDrive(
        x -> leftMotor.setVoltage(x * 12), 
        x -> rightMotor.setVoltage(x * 12) 
    );

    public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            drivetrain.tankDrive(left.getAsDouble(), right.getAsDouble());
        }).withName("drivetrain.tankDrive");
    }
    // public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
    //     return startEnd(()->{
    //         drivetrain.tankDrive(left.getAsDouble(), right.getAsDouble());
    //     },()-> {
    //         tankDriveStop();
    //     }).withName("shooter.runBeltCommand");
    // }
    public Command tankDriveStop() {
        return run(() -> {
            drivetrain.tankDrive(0, 0);
        }).withName("drivetrain.tankDrive");
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("drivetrain/leftMotor/Position",leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/leftMotor/Current",leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/leftMotor/Voltage",leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/leftMotor/Velocity",leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/leftMotor/Acceleration",leftMotor.getAcceleration().getValueAsDouble());

        SmartDashboard.putNumber("drivetrain/rightMotor/Position",rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/rightMotor/Current",rightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/rightMotor/Voltage",rightMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/rightMotor/Velocity",rightMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("drivetrain/rightMotor/Acceleration",rightMotor.getAcceleration().getValueAsDouble());
    }
    
    // public Command baseMotor(DoubleSupplier mainInput){
    //     return run(() -> {
    //         maindrive.tankDrive(mainInput.getAsDouble(), mainInput.getAsDouble());
    //     }).withName("drivetrain.baseMotor");
    // }

    // public static final class Constants {
    //     public static final double kP = 2.0;
    //     public static final double kI = 0.0;
    //     public static final double kD = 0.0;
    //     public static final double kS = 0.15;
    //     public static final double kG = 2.28;
    //     public static final double kV = 3.07;
    //     public static final double kA = 0.41
    //     public static final double MAX_ACCELERATION = 10.0
    //     public static final double MAX_VELOCITY = 20.0;

    // private final ProfiledPIDController
    //     elevatorPidController = 
    //     new ProfiledPIDController(kP, kI, kD,
    //     new TrapezoidProfile.Constraints(
    //         MOTION_PROFILE.MAX_VELOCITY, 
    //         MOTION_PROFILE.MAX_ACCELERATION
    //     )
    // );

    // private final SimpleMotorFeedforward
    //     simpleMotorFeedforward = 
    //     new SimpleMotorFeedforward(kS, kV, kA
    // );

    // public Command moveToCurrentGoalCommand() {
    //     return run(()->{
    //         double feedforward = 
    //             simpleMotorFeedforward.calculate(
    //             elevatorPidController.getSetpoint()
    //             .velocity);
    //         double pid = 
    //             elevatorPidController.calculate(
    //             getPosition());
    //         elevatorVoltage = MathUtil.clamp(
    //             feedforward + pid, -12, 12);
    //         setVoltage(elevatorVoltage);
    //         }).withName("elevator.moveToCurrentGoal");


}