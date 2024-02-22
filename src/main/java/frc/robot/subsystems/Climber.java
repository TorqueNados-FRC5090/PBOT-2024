package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.CLIMBER_RATIO;
import frc.robot.wrappers.GenericPID;

public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private GenericPID climberPID;

    /** Constructs a climber
     *  @param rightMotorID The ID of the right climbing motor from the perspective of the robot
     *  @param rightMotorID The ID of the left climbing motor from the perspective of the robot */
    public Climber(int rightMotorID, int leftMotorID) {
        leaderMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        leaderMotor.restoreFactoryDefaults();

        followerMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor, true);

        climberPID = new GenericPID(leaderMotor, ControlType.kPosition, 0, 0, 0); // TODO: TUNE PID FOR USE
        climberPID.setRatio(CLIMBER_RATIO);
    }

    /** @return How far the climber is extended in inches */
    public double getHeight() { return climberPID.getMeasurement(); }
    /** @return Whet */
    public boolean atSetpoint() { return climberPID.atSetpoint(.5); } 

    /** Moves the climber to a given height in inches
     *  @param inches The desired height of the climber in inches */
    public void goToPosition(double inches) {
        climberPID.activate(inches);
    }
    /** Stops the climber from moving */
    public void stop() { 
        climberPID.pause(); 
        leaderMotor.stopMotor(); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Height", getHeight());
        SmartDashboard.putBoolean("Climber at Setpoint", atSetpoint());
    }
}
