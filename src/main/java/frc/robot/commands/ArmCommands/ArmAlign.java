// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RGBLED;
import frc.robotkt.constants.VisionConstants;
import frc.robotkt.subsystems.Arm;
import frc.robotkt.subsystems.Camera;
import frc.robotkt.subsystems.Drivetrain;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.Set;

public class ArmAlign extends Command {
    final Drivetrain drivetrain;
    final Arm arm;
    final Camera camera;
    final RGBLED led;
    final CommandXboxController controller;

    final Set<Integer> targetIds = Set.of(4, 7);

    PIDController drivectl = new PIDController(.001, 0, 0);
    PIDController armctl = new PIDController(.027, 0, 0);
    double theta = 0;

    double x;
    double setPoint = 430;
    double armAngle;
    double power;
    PIDController armpid = new PIDController(.027, 0, 0);


    //   armpid.setTolerance(1.0);
    double distance;

    Timer timer = new Timer();
    double proportion;
    //List<PhotonTrackedTarget> targets;


    public ArmAlign(Drivetrain drivetrain, Arm arm, Camera camera, RGBLED led, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.camera = camera;
        this.led = led;
        this.controller = controller;

        camera.setMode(Camera.CameraMode.AprilTag);
        addRequirements(drivetrain, arm, camera, led);
    }

    public void initialize() {

        armpid.setTolerance(0.5);
        timer.reset();
        timer.start();
    }

    public void execute() {
        if (!camera.hasTargets()) {
            led.setColor(255, 0, 0);
            drivetrain.drive(controller);
        }

        List<PhotonTrackedTarget> targets = camera.getTargets();
        Optional<PhotonTrackedTarget> target = targets.stream().filter(t -> targetIds.contains(t.getFiducialId()) && t.getPoseAmbiguity() <= 0.6 && t.getPoseAmbiguity() != -1).findFirst();

        if (target.isEmpty()) return;
        led.setColor(0, 255, 0);

        PhotonTrackedTarget found = target.get();
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeight,
                VisionConstants.kTagHeight,
                VisionConstants.kCameraPitch,
                Units.degreesToRadians(found.getPitch())
        );


    }

//    public void execute() {
//        //PhotonPipelineResult result = Robot.camera1.getLatestResult();
//
//
//        var result = Robot.camera1.getLatestResult();
//
//
//        if (result.hasTargets()) {
//            //LIST of targets photon vision has
//            targets = result.getTargets();
//
//            //checks to see if there is a list of apriltags to check. if no targets are visable, end command
//            if (targets.isEmpty()) {
//                SmartDashboard.putBoolean("done", true);
//            }
//
//            var foundTargets = targets.stream().filter(t -> (t.getFiducialId() == 4 || t.getFiducialId() == 7)).filter(t -> !t.equals(4) && t.getPoseAmbiguity() <= .6 && t.getPoseAmbiguity() != -1).findFirst();
//
//            if (foundTargets.isPresent()) {
//
//                Robot.led.setColor(0, 255, 0);
//                //do stuff here
//
//                distance = PhotonUtils.calculateDistanceToTargetMeters(
//                        Units.inchesToMeters(9.451),
//                        Constants.VisionConstants.kAprilTagHeight,
//                        Constants.VisionConstants.kCameraPitchRadians,
//                        Units.degreesToRadians(foundTargets.get().getPitch()));
//                SmartDashboard.putNumber("distance", distance);
//
//
//                theta = Units.radiansToDegrees(Math.atan(Constants.VisionConstants.kspeakerHeight / distance));
//
//
//                x = foundTargets.get().getDetectedCorners().stream().mapToDouble((a) -> a.x).sum() / 4;
//                proportion = distance / 8.3;
//                SmartDashboard.putNumber("X", x);
//
//                Robot.shintake.setShooter(1, 1);
//
//                armAngle = ((53 - theta) + 7) - (8 * Math.pow(proportion, 4)); // 6, 2 , 5,4
//                if (distance > 3.5) armAngle -= proportion * 5;
//                SmartDashboard.putNumber("set angle", armAngle);
//                // armAngle = (3.33*Math.pow(distance,3) - 32.5 *(Math.pow(distance,2) )+ 108.17*distance -92.375);
//
//                if (armAngle < 0) armAngle = 0;
//                if (armAngle > 53) armAngle = 53;
//                //armpid.calculate(Robot.arm.getArmPot(),armAngle);
//                Robot.arm.armLeft.set(armpid.calculate(Robot.arm.getArmPot(), armAngle));
//                SmartDashboard.putNumber("set arm angle", armAngle);
//
//
//                if (armpid.atSetpoint()) {
//                    RobotContainer.driverctl.setRumble(RumbleType.kBothRumble, .3);
//                    RobotContainer.operctl.getHID().setRumble(RumbleType.kBothRumble, .5);
//                    x = foundTargets.get().getDetectedCorners().stream().mapToDouble((a) -> a.x).sum() / 4;
//                    Robot.arm.armLeft.set(armpid.calculate(Robot.arm.getArmPot(), armAngle));
//                    Robot.drivetrain.drive(.5 * -MathUtil.applyDeadband(RobotContainer.driverctl.getLeftY(), .5 * OIConstants.kDriveDeadband), -MathUtil.applyDeadband(RobotContainer.driverctl.getLeftX(), OIConstants.kDriveDeadband), drivepid.calculate(x, setPoint), false, false);
//                }
//
//
//                Robot.drivetrain.drive(-MathUtil.applyDeadband(RobotContainer.driverctl.getLeftY(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(RobotContainer.driverctl.getLeftX(), OIConstants.kDriveDeadband), drivepid.calculate(x, setPoint), false, false);
//                //Robot.shintake.setShooter(1,1);
//
//            }
//        } else {
//            Robot.arm.hold();
//            Robot.led.setColor(255, 0, 0);
//            Robot.drivetrain.drive(-MathUtil.applyDeadband(RobotContainer.driverctl.getLeftY(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(RobotContainer.driverctl.getLeftX(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(RobotContainer.driverctl.getRightX(), OIConstants.kDriveDeadband), false, false);
//            RobotContainer.driverctl.setRumble(RumbleType.kBothRumble, 0);
//            RobotContainer.operctl.getHID().setRumble(RumbleType.kBothRumble, 0);
//            Robot.shintake.setShooter(1, 1);
//
//        }
//
//
//    }


    @Override
    public void end(boolean interrupted) {
        Robot.arm.hold();
        RobotContainer.driverctl.setRumble(RumbleType.kBothRumble, 0);
        RobotContainer.operctl.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//timer.get()>5;
    }
}
