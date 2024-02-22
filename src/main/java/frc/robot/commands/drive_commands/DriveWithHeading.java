package frc.robot.commands.drive_commands;

// Imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import java.util.function.DoubleSupplier;

/** Drives the robot with a locked heading */
public class DriveWithHeading extends Command {
    // Declare variables that will be initialized by the constructor
    private final SwerveDrivetrain drivetrain;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputY;
    private final double headingDegrees;
    private final PIDController headingController;
    
    /** Constructs a DriveWithHeading command
     *  @param drivetrain The robot's drivetrain 
     *  @param translationInputX The joystick that will be used to control horizontal movement
     *  @param translationInputY The joystick that will be used to control forward and backward movement
     *  @param headingDegrees The robot's desired heading in degrees */
    public DriveWithHeading(SwerveDrivetrain drivetrain, DoubleSupplier translationInputX, DoubleSupplier translationInputY, double headingDegrees) {
        this.drivetrain = drivetrain;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.headingDegrees = headingDegrees;
        this.headingController = new PIDController(.04, 0, .0005);
        
        addRequirements(drivetrain);
    }

    @Override // Configure the PID controller
    public void initialize() {
        headingController.setTolerance(2); // Allow for 2 degrees of rotational error
        headingController.enableContinuousInput(-180, 180); // -180 and 180 are the same heading
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the x and y values to drive with
        double x = inputX.getAsDouble();
        double y = inputY.getAsDouble();

        // Calculate the rotation instruction
        double pidOut = headingController.calculate(drivetrain.getHeadingDegrees(), headingDegrees);
        double rotation = -MathUtil.clamp(pidOut, -1, 1);
               
        // Send a drive instruction to the drivetrain
        drivetrain.drive(x, y, rotation);
    }

    // Command ends when robot is done rotating
    @Override
    public boolean isFinished() {
        return headingController.atSetpoint();
    }

    // Runs when the command ends
    @Override
    public void end(boolean interrupted) {}
}
