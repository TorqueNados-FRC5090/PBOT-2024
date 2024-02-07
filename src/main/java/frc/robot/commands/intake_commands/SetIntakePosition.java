package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

/** This command moves the intake to a given position and ends */
public class SetIntakePosition extends Command {
    private Intake intake;
    private IntakePosition targetPosition;

    public  SetIntakePosition(Intake intake, IntakePosition targetPosition){
        this.intake = intake;

        this.targetPosition = targetPosition;
        
        addRequirements(intake);
    }

    @Override // Command the intake pid controller
    public void initialize() {
        intake.goTo(targetPosition);
    }

    @Override
    public void execute() {}

    @Override // Command ends when the intake is in position
    public boolean isFinished() {
        return intake.intakeAtSetPoint();
    }
   
    public void end(boolean interrupted) {}
}
