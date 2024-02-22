package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;
import static frc.robot.Constants.IntakeIDs.*;
import static frc.robot.Constants.IntakeConstants.IntakePosition;
import static frc.robot.Constants.ShooterIDs.*;
import static frc.robot.Constants.ClimberIDs.*;

// Command imports
import frc.robot.commands.*;
import frc.robot.commands.drive_commands.LockDrivetrain;
import frc.robot.commands.drive_commands.SwerveDriveCommand;
import frc.robot.commands.intake_commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Subsystem imports
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.*;

// Other imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);

    public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
    public final Intake intake = new Intake(INTAKE_DRIVER_ID, INTAKE_ROTATOR_ID, INTAKE_LIMIT_ID);
    public final Shooter shooter = new Shooter(SHOOTER_RIGHT_ID, SHOOTER_LEFT_ID, SHOOTER_PIVOT_RIGHT_ID, SHOOTER_PIVOT_LEFT_ID);
    public final Climber climber = new Climber(CLIMBER_RIGHT_ID, CLIMBER_LEFT_ID);
    public final Blinkin blinkin = new Blinkin();
    public final Limelight shooterLimelight = new Limelight("limelight-pbshoot");
    
    private final AutonContainer auton = new AutonContainer();
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();    

    /** Constructs a RobotContainer */
    public RobotContainer() {
        initChooser();
        
        setDriverControls();
        setOperatorControls();
        setDefaultCommands();
    }

    /** Initialize the auton selector on the dashboard */
    private void initChooser() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        autonChooser.setDefaultOption("Do Nothing", auton.doNothing());
    }


    /** Use this to pass the autonomous command to the main {@link Robot} class.
     *  @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /** Configures a set of commands that will run by default without human operation */
    private void setDefaultCommands() {
        // Set the intake to always be intaking by default
        intake.setDefaultCommand(new IntakePiece(intake));
        // we are making the LED to be on the entire time 
        blinkin.setDefaultCommand(new LEDControlCommand(intake, blinkin));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setDriverControls() {
        // Drives the robot with the joysticks
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, 
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX()));

        // HOLD X -> Lock the drivetrain for anti-defense
        driverController.x().whileTrue(new LockDrivetrain(drivetrain));
        // PRESS START -> Reset the heading of the robot so the currently faced direction becomes 0
        driverController.start().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
        // PRESS BACK -> Switch the robot between field-centric and robot-centric mode
        driverController.back().onTrue(new InstantCommand(() -> drivetrain.toggleFieldCentric()));
        // HOLD LT -> Activate the automatic intake
        driverController.leftTrigger().whileTrue(new IntakeAutoPickup(intake));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        // PRESS A -> Move the intake to the floor pickup position
        operatorController.a().onTrue(new SetIntakePosition(intake, IntakePosition.PICKUP));
        // PRESS B -> Move the intake to the climbing position (vertical)
        operatorController.b().onTrue(new SetIntakePosition(intake, IntakePosition.CLIMB));
        // PRESS Y -> Move the intake to the shooting position
        operatorController.y().onTrue(new SetIntakePosition(intake, IntakePosition.SHOOT));

        // HOLD RT -> Drive the intake outward for piece ejection
        operatorController.rightTrigger().whileTrue(new Eject(intake));

        // PRESS LB -> Set the shooter to half speed
        operatorController.leftBumper().onTrue(new InstantCommand(() -> shooter.setSpeed(2500)));
        // PRESS RB -> Set the shooter to full speed
        operatorController.rightBumper().onTrue(new InstantCommand(() -> shooter.setSpeed(5000)));
        // PRESS START -> Stop the shooter
        operatorController.start().onTrue(new InstantCommand(()-> shooter.stopShooter()));
    }
}