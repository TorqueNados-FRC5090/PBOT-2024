package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_PIVOT_RATIO;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

public class Shooter extends SubsystemBase{
    private CANSparkFlex shooterLeader;
    private CANSparkFlex shooterFollower;
    private GenericPID shooterPID;

    private CANSparkMax pivotLeader;
    private CANSparkMax pivotFollower;
    private GenericPID pivotPID;

    /** Constructs a shooter
     *  @param shooterRightID The ID of the right shooter motor from the perspective of the robot
     *  @param shooterLeftID The ID of the left shooter motor from the perspective of the robot
     *  @param pivotRightID The ID of the right shooter pivoting motor from the perspective of the robot
     *  @param pivotLeftID The ID of the left shooter pivoting motor from the perspective of the robot
     */
    public Shooter(int shooterRightID, int shooterLeftID, int pivotRightID, int pivotLeftID) {
        shooterLeader = new CANSparkFlex(shooterRightID, MotorType.kBrushless);
        shooterLeader.restoreFactoryDefaults();
        shooterFollower = new CANSparkFlex(shooterLeftID, MotorType.kBrushless);
        shooterFollower.restoreFactoryDefaults();
        shooterFollower.follow(shooterLeader, true);

        shooterPID = new GenericPID(shooterLeader, ControlType.kVelocity, .00022, .0000005, 0);

        pivotLeader = new CANSparkMax(pivotLeftID, MotorType.kBrushless);
        pivotLeader.restoreFactoryDefaults();
        pivotFollower = new CANSparkMax(pivotRightID, MotorType.kBrushless);
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.follow(pivotLeader, true);

        pivotPID = new GenericPID(pivotLeader, ControlType.kPosition, 0, 0, 0); //TODO: TUNE PID FOR USE
        pivotPID.setRatio(SHOOTER_PIVOT_RATIO);
    }

    /** @return Whether the shooter has reached its target speed */
    public boolean atTargetSpeed() { return shooterPID.atSetpoint(10); }
    /** @return Whether the shooter has reached its target position */
    public boolean atTargetPosition() { return pivotPID.atSetpoint(2); }
    /** @return The RPM of the shooter */
    public double getRPM() { return shooterPID.getMeasurement(); }
    /** @return The position of the shooter's pivot */
    public double getPosition() { return pivotPID.getMeasurement(); }

    /** Activates the shooter PID
     *  @param RPM The speed of the shooter in RPM */
    public void setSpeed(double RPM){
        shooterPID.activate(RPM); 
    }
    /** Stops the shooter */
    public void stopShooter() {
        shooterPID.pause();
        shooterLeader.stopMotor();
    }
    
    /** Activates the shooter PID
     *  @param angle The desired angle of the shooter */
    public void goToPosition(double angle) {
        pivotPID.activate(angle); 
    }
    /** Stops the shooter */
    public void stopPivot() {
        pivotPID.pause();
        pivotLeader.stopMotor();
    }

    @Override // Called every 20ms
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        SmartDashboard.putBoolean("Shooter at Target Speed", atTargetSpeed());

        SmartDashboard.putNumber("Shooter Pivot Position", getPosition());
        SmartDashboard.putBoolean("Shooter at Target Position", atTargetPosition());
    }
}
