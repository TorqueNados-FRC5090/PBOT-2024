package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Intake;

public class LEDControlCommand extends Command {
    private Intake intake;
    private Blinkin blinkin;
    public LEDControlCommand(Intake intake, Blinkin blinkin) {
       this.intake = intake;
       this.blinkin = blinkin;
       addRequirements(blinkin);
    }

    @Override
    public void initialize() {
    // This command should turn on the LED at the start of the game
        
    }
    
    @Override
    public void execute() {
     // This command should change colors fro if there is an object
        if (intake.holdingPiece()) {
            blinkin.blueSolid(); }
        else blinkin.goldSolid();

        
    }
    @Override
    public boolean isFinished(){
        // This command should turn off the LED at the end of the game
        return false;
    }
}