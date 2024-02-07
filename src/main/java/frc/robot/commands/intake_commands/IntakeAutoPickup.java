package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

/** This command attemps to automatically pick up a piece 
 *  from the floor and prepare it for shooting. */
public class IntakeAutoPickup extends Command{
    private Intake intake;
    
    public IntakeAutoPickup(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    } 

    @Override // Drop the intake to pickup position unless a piece is already being held
    public void initialize() {
        if (!intake.holdingPiece())
            intake.goTo(IntakePosition.PICKUP);
    }

    @Override // Try to pick up a piece
    public void execute(){
        intake.intake(.25);
    }

    @Override // Command ends once a piece is picked up
    public boolean isFinished(){
        return intake.holdingPiece() && intake.intakeAtSetPoint();
    }
    @Override // Make sure the intake is stopped and go to shooting position
    public void end(boolean interrupted){
        intake.stopIntake();
        intake.goTo(IntakePosition.SHOOT);
    }

}
