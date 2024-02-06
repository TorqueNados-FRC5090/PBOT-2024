package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private GenericPID shooterPID;

    public Shooter(int rightMotorID, int leftMotorID) {
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.follow(rightMotor, true);

        shooterPID = new GenericPID(rightMotor, ControlType.kVelocity, .00022, .0000005, 0);
    }

    /** @return The RPM of the left motor */
    public double getLeftMotorRPM() {return leftMotor.getEncoder().getVelocity(); }
    /** @return The RPM of the right motor */
    public double getRightMotorRPM() {return rightMotor.getEncoder().getVelocity(); }
    /** @return The left motor*/
    public CANSparkMax getLeftMotor(){return leftMotor; }
    /** @return The right motor */
    public CANSparkMax getRightMotor(){return rightMotor; }

    /** Shoots out a piece
     * @param speed speed of motors by %
     */
    public void shoot(double speed) {
        rightMotor.set(speed);
    }

    /** Stops the shooter */
    public void stop() {
        shooterPID.pause();
        rightMotor.stopMotor();
    }
    /** Activates the shooter PID
     * @param RPM speed of motors by RPM
     */
    public void shootRPM(double RPM){
        shooterPID.activate(RPM); 
    }

    @Override // Called every 20ms
    public void periodic() {
        // Prints the left motor RPM
        SmartDashboard.putNumber("Left Motor RPM", getLeftMotorRPM());
        // Prints the right motor RPM
        SmartDashboard.putNumber("Right Motor RPM", getRightMotorRPM());
    }
}
