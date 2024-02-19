package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    private CANSparkFlex leaderMotor;
    private CANSparkFlex followerMotor;
    private GenericPID shooterPID;

    /** Constructs a shooter
     *  @param rightMotorID The ID of the right motor from the perspective of the robot
     *  @param leftMotorID The ID of the left motor from the perspective of the robot
     */
    public Shooter(int rightMotorID, int leftMotorID) {
        leaderMotor = new CANSparkFlex(rightMotorID, MotorType.kBrushless);
        leaderMotor.restoreFactoryDefaults();
        
        followerMotor = new CANSparkFlex(leftMotorID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor, true);

        shooterPID = new GenericPID(leaderMotor, ControlType.kVelocity, .00022, .0000005, 0);
    }

    /** @return Whether the shooter has reached its target speed */
    public boolean atTargetSpeed() { return shooterPID.atSetpoint(10); }
    /** @return The RPM of the leader motor */
    public double getRPM() { return shooterPID.getMeasurement(); }

    /** Activates the shooter PID
     *  @param RPM The speed of the shooter in % power
     */
    public void setSpeed(double RPM){
        shooterPID.activate(RPM); 
    }
    /** Stops the shooter */
    public void stop() {
        shooterPID.pause();
        leaderMotor.stopMotor();
    }

    @Override // Called every 20ms
    public void periodic() {
        // Prints the current speed of the shooter in RPM
        SmartDashboard.putNumber("Shooter RPM", getRPM());
    }
}
