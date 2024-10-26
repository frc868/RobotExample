package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain {
    private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushless);

    private DifferentialDrive drivetrain = new DifferentialDrive(leftMotor, rightMotor);

    public Drivetrain() {
        rightMotor.setInverted(true);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }
}
