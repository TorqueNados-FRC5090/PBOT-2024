package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase  {

    public Spark LED;

    public Blinkin(){
        LED = new Spark(0);
    }
    /** Sets LED to blue, light bouncing back and fourth */
    public void blueBounce(){
        LED.set(-0.01);
    }
    /** Sets LED to blue, flashing softly */
    public void heartbeatBlue(){
        LED.set(0.03);
    }
    /** Sets LED to gold, solid color */
    public void goldSolid(){
        LED.set(0.67);
    }
    /** Sets LED to blue, solid color */
    public void blueSolid(){
        LED.set(0.85);
    }
    /** Sets LED to green, solid color */
    public void greenSolid(){
        LED.set(0.77);
    }
    /** Sets LED to red, solid color */
    public void redSolid(){
        LED.set(0.61);
    }

}
