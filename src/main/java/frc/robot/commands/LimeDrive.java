package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

// Drives the robot to a target tracked by limelight
public class LimeDrive extends Command {
    private final SwerveDrivetrain drivetrain;
    private final boolean originalOrientation;
    private final Limelight limelight;
    private final double goalDistance;
    private final double goalOffset;
    
    private final PIDController driveControllerX;
    private final PIDController driveControllerY;
    private final PIDController headingController;
    
    // The command should be able to end on it's own unless specified otherwise
    private boolean ends = true;

    /** Constructs a LimeDrive command
     * 
     *  @param drivetrain The drivetrain that will be driven by this command
     *  @param limelight The limelight camera that will be used for tracking
     *  @param goalPosition Where the target should be at the end of the command relative to the robot
     */
    public LimeDrive(SwerveDrivetrain drivetrain, Limelight limelight, Translation2d goalPosition) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        originalOrientation = drivetrain.isFieldCentric();
        goalOffset = goalPosition.getX();
        goalDistance = goalPosition.getY();

        // Configure PID Controllers for each axis of movement
        driveControllerX = new PIDController(.7, 0, 0);
        driveControllerX.setTolerance(.1); // Allow for 10 centimeters of positional error

        driveControllerY = new PIDController(.7, 0, 0);
        driveControllerY.setTolerance(.1); // Allow for 10 centimeters of positional error

        headingController = new PIDController(.02, 0, .0005);
        headingController.setTolerance(2); // Allow for 2 degrees of rotational error
        headingController.enableContinuousInput(-180, 180); // -180 and 180 are the same heading

        addRequirements(drivetrain);
    }

    /** Constructs a LimeDrive command
     * 
     *  @param drivetrain The drivetrain that will be driven by this command
     *  @param limelight The limelight camera that will be used for tracking
     *  @param goalPosition Where the target should be at the end of the command relative to the robot
     *  @param ends Whether the command should end when the robot has reached its destination
     */
    public LimeDrive(SwerveDrivetrain drivetrain, Limelight limelight, Translation2d goalPosition, boolean ends) { 
        this(drivetrain, limelight, goalPosition);
        this.ends = ends;
    }

    // Called once when command is first scheduled.
    @Override
    public void initialize() {
        // This command should drive using robot centric movement
        drivetrain.setFieldCentric(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Do nothing if there is no valid target
        if (!limelight.hasValidTarget()) {
            drivetrain.drive(0, 0, 0);
            return;
        }

        // Get the location of the target from limelight
        double[] targetpose = limelight.getTargetPose();
        double targetX = targetpose[0];
        double targetDist = targetpose[2];
        double targetAngle = targetpose[4];

        // Preprocess the driving instructions
        double xRawOutput = driveControllerX.calculate(targetX, goalOffset);
        double xClamped = MathUtil.clamp(xRawOutput, -1, 1);
        double x = MathUtil.applyDeadband(xClamped, .01);
        
        double yRawOutput = -driveControllerY.calculate(targetDist, goalDistance);
        double yClamped = MathUtil.clamp(yRawOutput, -1, 1);    
        double y = MathUtil.applyDeadband(yClamped, .01);
        
        double rotRawOutput = -headingController.calculate(targetAngle, 0);
        double rotClamped = MathUtil.clamp(rotRawOutput, -1, 1);
        double rotation = MathUtil.applyDeadband(rotClamped, .01);
               
        // Send the instructions to the drivetrain
        drivetrain.sendDrive(x, y, rotation, true); 
    }

    
    @Override // Command ends when robot has reached its destination
    public boolean isFinished() {
        return ends &&
               driveControllerX.atSetpoint() &&
               driveControllerY.atSetpoint() &&
               headingController.atSetpoint();
    }

    
    @Override 
    public void end(boolean interrupted) {
        // Return to whatever orientation was being used when the command started
        drivetrain.setFieldCentric(originalOrientation);
    }
}
