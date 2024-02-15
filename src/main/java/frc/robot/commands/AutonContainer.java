package frc.robot.commands;

import static frc.robot.Constants.SwerveConstants.ModuleConstants.WHEEL_DIAMETER;
import static frc.robot.Constants.SwerveConstants.MAX_TRANSLATION_SPEED;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {
    private final SwerveDrivetrain drivetrain;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robot) {
        drivetrain = robot.drivetrain;

        AutoBuilder.configureHolonomic(
            drivetrain::getPoseMeters, 
            drivetrain::setOdometry,
            drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    MAX_TRANSLATION_SPEED, // Max module speed, in m/s
                    WHEEL_DIAMETER, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> robot.onRedAlliance(),
            drivetrain);
    }

    public Command ppTest() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
        drivetrain.setOdometry(path.getPreviewStartingHolonomicPose());
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
    public Command ppCircle() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Circle");
        drivetrain.setOdometry(path.getPreviewStartingHolonomicPose());
        return AutoBuilder.followPath(path);
    }

    /** Auton that drops a piece high, reverses, and sets heading*/
    public Command doNothing() {
        return new WaitCommand(0);
    }
}
