package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;
import static frc.robot.Constants.IntakeIDs.*;
import static frc.robot.Constants.IntakeConstants.IntakePosition;
import static frc.robot.Constants.ShooterIDs.*;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.intake_commands.IntakePiece;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.LEDControlCommand;
import frc.robot.commands.LockDrivetrain;
import frc.robot.commands.intake_commands.Eject;
import frc.robot.commands.LimeDrive;
import frc.robot.commands.intake_commands.IntakeAutoPickup;
import frc.robot.commands.intake_commands.SetIntakePosition;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

// Other imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final XboxController driverController = new XboxController(DRIVER_PORT);
    private final XboxController operatorController = new XboxController(OPERATOR_PORT);

    private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
    private final Intake intake = new Intake(INTAKE_DRIVER_ID, INTAKE_ROTATOR_ID, INTAKE_LIMIT_ID);
    private final Shooter shooter = new Shooter(SHOOTER_RIGHT_ID, SHOOTER_LEFT_ID);
    private final Blinkin blinkin = new Blinkin();
    private final Limelight shooterLimelight = new Limelight("limelight-pbshoot");
    
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
        Trigger lockBtn = new Trigger(() -> driverController.getXButton());
        lockBtn.whileTrue(new LockDrivetrain(drivetrain));
        // PRESS START -> Reset the heading of the robot so the currently faced direction becomes 0
        Trigger resetHeadingBtn = new Trigger(() -> driverController.getStartButton());
        resetHeadingBtn.onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
        // PRESS BACK -> Switch the robot between field-centric and robot-centric mode
        Trigger toggleOrientationBtn = new Trigger(() -> driverController.getBackButton());
        toggleOrientationBtn.onTrue(new InstantCommand(() -> drivetrain.toggleFieldCentric()));
        // HOLD RT -> The robot will automatically drive 2 meters infront of the nearest in-view apriltag
        Trigger limeDriveBtn = new Trigger(() -> driverController.getRightTriggerAxis() > .5);
        limeDriveBtn.whileTrue(new LimeDrive(drivetrain, shooterLimelight, -2, false));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        // PRESS A -> Move the intake to the floor pickup position
        Trigger pickUpBtn = new Trigger(() -> operatorController.getAButton());
        pickUpBtn.onTrue(new SetIntakePosition(intake, IntakePosition.PICKUP));
        // PRESS B -> Move the intake to the climbing position (vertical)
        Trigger climbBtn = new Trigger(() -> operatorController.getBButton());
        climbBtn.onTrue(new SetIntakePosition(intake, IntakePosition.CLIMB));
        // PRESS Y -> Move the intake to the shooting position
        Trigger shootBtn = new Trigger(() -> operatorController.getYButton());
        shootBtn.onTrue(new SetIntakePosition(intake, IntakePosition.SHOOT));

        // HOLD X -> Activate the automatic intake
        Trigger AutoIntakeBtn = new Trigger(() -> driverController.getLeftTriggerAxis() > .5);
        AutoIntakeBtn.whileTrue(new IntakeAutoPickup(intake));

        // HOLD RT -> Drive the intake outward for piece ejection
        Trigger ejectBtn = new Trigger(() -> operatorController.getRightTriggerAxis() > .5);
        ejectBtn.whileTrue(new Eject(intake));

        // PRESS LB -> Set the shooter to half speed
        Trigger halfShooterBtn = new Trigger(() -> operatorController.getLeftBumper());
        halfShooterBtn.whileTrue(new InstantCommand(() -> shooter.setSpeed(2500)));
        // PRESS RB -> Set the shooter to full speed
        Trigger fullShooterBtn = new Trigger(() -> operatorController.getRightBumper());
        fullShooterBtn.whileTrue(new InstantCommand(() -> shooter.setSpeed(5000)));
        // PRESS START -> Stop the shooter
        Trigger stopShootBtn = new Trigger(() -> operatorController.getStartButton());
        stopShootBtn.onTrue(new InstantCommand(()-> shooter.stop()));
    }
}