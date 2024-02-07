package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** This command exists largely to allow the 
 *  use of intake.intake() as a default command */
public class IntakePiece extends Command {
    private Intake intake;
    
    public IntakePiece(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
   
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.intake(.25);
    }

    
    @Override 
    public boolean isFinished() {
		return false; 
    }

        
    @Override 
    public void end(boolean interrupted) {}
}
