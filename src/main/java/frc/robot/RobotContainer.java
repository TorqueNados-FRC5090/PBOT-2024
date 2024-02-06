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
import frc.robot.commands.AutonContainer;
import frc.robot.commands.LimeDrive;
import frc.robot.commands.IntakePickUp;
import frc.robot.commands.SetIntakePosition;
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
    private final Limelight shooterLimelight = new Limelight("limelight-pbshoot");
    private final Blinkin blinkin = new Blinkin();
    
    private final AutonContainer auton = new AutonContainer();
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();    

    /** Constructs a RobotContainer */
    public RobotContainer() {
        initChooser();

        setDriverControls();
        setOperatorControls();
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

    private void setDriverControls() {
        // Drives the robot with the joysticks
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, 
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX()));
        
        Trigger limeDriveBtn = new Trigger(() -> driverController.getRightTriggerAxis() > .05);
        limeDriveBtn.whileTrue(new LimeDrive(drivetrain, shooterLimelight, -2, false));
    }

    private void setOperatorControls() {
        // use the xbox controller shooter half speed left bumber
        // full speed right bumber 
        // intake/pickup left trigger 
        // and eject right 
        // then intake up will be up on D-pad and down be down on the Dpad  

        Trigger halfShooterBtn = new Trigger(() -> operatorController.getLeftBumper());
        halfShooterBtn.whileTrue(new InstantCommand(() -> shooter.shootRPM(2500)));

        Trigger fullShooterBtn = new Trigger(() -> operatorController.getRightBumper());
        fullShooterBtn.whileTrue(new InstantCommand(() -> shooter.shootRPM(5000)));
        
        Trigger intakeBtn = new Trigger(() -> operatorController.getLeftTriggerAxis() > .5); 
        intakeBtn.onTrue(new InstantCommand(() -> intake.intake(.3)));
        intakeBtn.onFalse(new InstantCommand(() -> intake.stopIntake()));

        Trigger ejectBtn = new Trigger(() -> operatorController.getRightTriggerAxis() > .5);
        ejectBtn.onTrue(new InstantCommand(() -> intake.eject(1)));
        ejectBtn.onFalse(new InstantCommand(() -> intake.stopIntake()));

        Trigger intakeUpBtn = new Trigger(() -> operatorController.getPOV() == 0);
        intakeUpBtn.onTrue(new InstantCommand(() -> intake.manualRotate(.2)));
        intakeUpBtn.onFalse(new InstantCommand(() -> intake.stopRotate()));    

        Trigger intakeDownBtn = new Trigger(() -> operatorController.getPOV() == 180);
        intakeDownBtn.onTrue(new InstantCommand(() -> intake.manualRotate(-.2)));
        intakeDownBtn.onFalse(new InstantCommand(() -> intake.stopRotate()));

        Trigger pickUpBtn = new Trigger(() -> operatorController.getAButton());
        pickUpBtn.onTrue(new SetIntakePosition(intake, IntakePosition.PICKUP));

        Trigger climbBtn = new Trigger(() -> operatorController.getBButton());
        climbBtn.onTrue(new SetIntakePosition(intake, IntakePosition.CLIMB));

        Trigger shootBtn = new Trigger(() -> operatorController.getYButton());
        shootBtn.onTrue(new SetIntakePosition(intake, IntakePosition.SHOOT));

        Trigger stopShootBtn = new Trigger(() -> operatorController.getStartButton());
        stopShootBtn.onTrue(new InstantCommand(()-> shooter.stop()));

        Trigger AutoIntakeBtn = new Trigger(() -> operatorController.getXButton());
        AutoIntakeBtn.toggleOnTrue(new IntakePickUp(intake));
    }
}