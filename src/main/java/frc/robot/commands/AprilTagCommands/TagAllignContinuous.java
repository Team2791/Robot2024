// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frckt.robot.subsystems.Drivetrain;
import org.photonvision.PhotonCamera;

public class TagAllignContinuous extends Command {

    PhotonCamera camera;
    Drivetrain drivetrain;

    double x;
    double setPoint = 420;
    XboxController controller;
    double power;
    PIDController pid;

    /**
     * Creates a new TagAllign.
     */
    public TagAllignContinuous(PhotonCamera camera, Drivetrain driveSubsystem, XboxController drivController) {
        this.controller = drivController;
        this.camera = camera;
        this.drivetrain = driveSubsystem;
        this.pid = new PIDController(.002, 0, 0);
        addRequirements(driveSubsystem);

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var res = camera.getLatestResult();

        if (res.hasTargets()) {
            controller.setRumble(RumbleType.kBothRumble, .1);
            var target = res.getBestTarget();

            if (target.getFiducialId() == 7 || target.getFiducialId() == 4) {

                // average as stream
                // 0 - 640
                x = target.getDetectedCorners().stream().mapToDouble((a) -> a.x).sum() / 4;
                SmartDashboard.putNumber("avg april tag position", x);
                SmartDashboard.putNumber("April tag yaw", target.getYaw());
                SmartDashboard.putString("target", target.toString());


                drivetrain.drive(-MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband), pid.calculate(x, setPoint), false, false);
            }

        } else {
            drivetrain.drive(-MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband), false, false);
            controller.setRumble(RumbleType.kBothRumble, 0);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
        drivetrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
