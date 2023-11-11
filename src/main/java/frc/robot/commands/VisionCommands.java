package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.VisionUtils;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.VisionSubsystem;


public class VisionCommands {
    public interface Checker {
        public boolean run();
    }
    public static Command SetPipeline(VisionConstants.Pipelines pipeline) {
        return new InstantCommand(() -> VisionUtils.setPipelineIndex("", pipeline.ordinal()));
    }

    public static class RumbleCheck extends CommandBase {
        private final XboxController controller;
        private final Checker check;

        public RumbleCheck(XboxController controller, Checker check) {
            this.controller = controller;
            this.check = check;
            // !!! could be troublesome
            // addRequirements(controller);
        }

        @Override
        public void execute() {
            controller.setRumble(RumbleType.kBothRumble, check.run() ? 1.0 : 0.0);
        }

        @Override
        public void end(boolean interrupted) {
            controller.setRumble(RumbleType.kBothRumble, 0.0);
        }

        @Override
        public boolean isFinished() {
            return !check.run();
        }
    }

    public static class ZAlign extends CommandBase {
        private final SwerveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final double targetZ;
        private final BooleanEntry CommandRunning;



        public ZAlign(SwerveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetZ) {
            this.driveSubsystem = driveSubsystem;
            this.visionSubsystem = visionSubsystem;
            this.targetZ = targetZ;
            CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false);

            addRequirements(driveSubsystem, visionSubsystem);
        }

        @Override
        public void initialize() {
            CommandRunning.set(true);
            visionSubsystem.lineupZPID.setTolerance(0.1, 0.1);
            visionSubsystem.lineupZPID.reset(visionSubsystem.mostRecentZ);
            visionSubsystem.lineupZPID.setGoal(targetZ);
        }

        @Override
        public void execute() {
            double zOutput = visionSubsystem.lineupZPID.calculate(visionSubsystem.mostRecentZ);
            driveSubsystem.drive(zOutput / 2, 0.0, 0.0, true, false);
            visionSubsystem.ZOutput.set(-zOutput);
        }

        @Override
        public boolean isFinished() {
            return visionSubsystem.lineupZPID.atGoal();
        }

        @Override
        public void end(boolean interrupted) {
            CommandRunning.set(false);
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

    public static class RetroreflectiveAlign extends CommandBase {
        private final SwerveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final double xTarget;
        private final PIDController lineupXPID;
        private final BooleanEntry CommandRunning;

        public RetroreflectiveAlign(SwerveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double xTarget) {
            this.driveSubsystem = driveSubsystem;
            this.visionSubsystem = visionSubsystem;
            this.xTarget = xTarget;
            lineupXPID = new PIDController(
                    VisionConstants.retroreflectiveP,
                    VisionConstants.retroreflectiveI,
                    VisionConstants.retroreflectiveD
            );
            CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false);

            addRequirements(driveSubsystem, visionSubsystem);
        }

        @Override
        public void initialize() {
            CommandRunning.set(true);
            lineupXPID.setTolerance(0.5, 0.1);
        }

        @Override
        public void execute() {
            double xOutput = -lineupXPID.calculate(visionSubsystem.mostRecentX, xTarget);
            driveSubsystem.drive(0.0, xOutput, 0.0, true, false);
            visionSubsystem.XSetpoint.set(xTarget);
            visionSubsystem.XOutput.set(xOutput);
        }

        @Override
        public boolean isFinished() {
            return lineupXPID.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            CommandRunning.set(false);
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

    public static class TurnToAngle extends CommandBase {
        private final SwerveSubsystem driveSubsystem;
        private final VisionSubsystem visionSubsystem;
        private final double targetAngleRadians;
        private final ProfiledPIDController ttaPID;

        public TurnToAngle(SwerveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetAngleRadians) {
            this.driveSubsystem = driveSubsystem;
            this.visionSubsystem = visionSubsystem;
            this.targetAngleRadians = targetAngleRadians;
            ttaPID = new ProfiledPIDController(
                    VisionConstants.ttaP,
                    VisionConstants.ttaI,
                    VisionConstants.ttaD,
                    VisionConstants.ttaConstraints
            );

            addRequirements(driveSubsystem);
        }

        @Override
        public void initialize() {
            ttaPID.setTolerance(0.05, 0.1);
            ttaPID.reset(driveSubsystem.heading());
            ttaPID.enableContinuousInput(-Math.PI, Math.PI);
            ttaPID.setGoal(targetAngleRadians);
        }

        @Override
        public void execute() {
            double out = ttaPID.calculate(driveSubsystem.heading());
            driveSubsystem.drive(0.0, 0.0, out, true, false);
            visionSubsystem.TTASetpoint.set(targetAngleRadians);
            visionSubsystem.TTAMeasurement.set(driveSubsystem.heading());
            visionSubsystem.TTAOutput.set(out);
        }

        @Override
        public boolean isFinished() {
            return ttaPID.atGoal();
        }

        @Override
        public void end(boolean interrupted) {
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

    public static SequentialCommandGroup ChuteVision(SwerveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, XboxController controller) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return new SequentialCommandGroup(
                    SetPipeline(VisionConstants.Pipelines.APRILTAG),
                    new RumbleCheck(controller, () -> !VisionUtils.getTV("")),
                    new ZAlign(driveSubsystem, visionSubsystem, -2.39),
                    new TurnToAngle(driveSubsystem, visionSubsystem, -Math.PI / 2.0)
            );
        } else {
            return new SequentialCommandGroup(
                    SetPipeline(VisionConstants.Pipelines.APRILTAG),
                    new RumbleCheck(controller, () -> !VisionUtils.getTV("")),
                    new ZAlign(driveSubsystem, visionSubsystem, -2.39),
                    new TurnToAngle(driveSubsystem, visionSubsystem, Math.PI / 2.0)
            );
        }
    }

    public static SequentialCommandGroup CubeVision(SwerveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, XboxController controller) {
        return new SequentialCommandGroup(
                SetPipeline(VisionConstants.Pipelines.APRILTAG),
                new RumbleCheck(controller, () -> !VisionUtils.getTV("")),
                new ZAlign(driveSubsystem, visionSubsystem, -2.9)
        );
    }

    public static SequentialCommandGroup RetroreflectiveVision(SwerveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, XboxController controller) {
        return new SequentialCommandGroup(
                SetPipeline(VisionConstants.Pipelines.RETROREFLECTIVE),
                new RumbleCheck(controller, () -> !VisionUtils.getTV("")),
                new TurnToAngle(driveSubsystem, visionSubsystem, Math.PI),
                new RetroreflectiveAlign(driveSubsystem, visionSubsystem, 6.7)
        );
    }
}
