package frc.robot.commands;
//
////import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
////import com.pathplanner.lib.path.PathPlannerTrajectory;
////import com.pathplanner.lib.auto.PIDConstants;
//
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.constants.DrivetrainConstants;
//import frc.robot.constants.TrajectoryConstants;
//import frc.robot.subsystems.SwerveSubsystem;
//
//import java.util.HashMap;
//import java.util.List;
//
//public class AutoTrajectories {
//    private final SwerveSubsystem swerveSubsystem;
//
//    public AutoTrajectories(SwerveSubsystem swerveSubsystem) {
//        this.swerveSubsystem = swerveSubsystem;
//    }
//
//    public Command followPathCommand(String pathName){
//        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
//
//        // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
//        return new FollowPathWithEvents(
//                new FollowPathHolonomic(
//                        path,
//                        this::getPose, // Robot pose supplier
//                        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
//                        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//                        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//                        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                                4.5, // Max module speed, in m/s
//                                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
//                                new ReplanningConfig() // Default path replanning config. See the API for the options here
//                        ),
//                        this // Reference to this subsystem to set requirements
//                ),
//                path, // FollowPathWithEvents also requires the path
//                this::getPose // FollowPathWithEvents also requires the robot pose supplier
//        );
//    }
//}
