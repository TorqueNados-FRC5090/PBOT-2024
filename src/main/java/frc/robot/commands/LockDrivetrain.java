package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/** Locks the wheels of the drivetrain into an X shape for anti-defense */
public class LockDrivetrain extends Command {
    // Variable declaration
    private final SwerveDrivetrain drivetrain;
    private final SwerveModuleState[] lockedStates = {
        new SwerveModuleState(.031, new Rotation2d(-45)),
        new SwerveModuleState(.031, new Rotation2d(45)),
        new SwerveModuleState(.031, new Rotation2d(-45)),
        new SwerveModuleState(.031, new Rotation2d(45))
    };

    /** Constructs a LockDrivetrain command
     *  
     * @param drivetrain The robot's drivetrain
    */
    public LockDrivetrain(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    
    @Override // Lock the drivetrain
    public void execute() {
        drivetrain.setModuleStates(lockedStates);
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Command never ends on its own
    }
}
