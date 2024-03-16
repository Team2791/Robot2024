// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAngleCommand extends Command {
	PIDController drivepid = new PIDController(.001, 0, 0);
	double theta=0;

	double x;
  double setPoint = 420;
  double armAngle;
  double  power;
  PIDController armpid = new PIDController(.015,0,0);
  
//   armpid.setTolerance(1.0);
  double distance;

  Timer timer= new Timer();
  double proportion;
	

	public PhotonAngleCommand() {
		
		addRequirements(Robot.m_drivetrain);
		addRequirements(Robot.arm);
	}

	public void initialize(){
		armpid.setTolerance(2);
		timer.reset();
		timer.start();
	}

	public void execute() {
		PhotonPipelineResult result = Robot.camera1.getLatestResult();

		if (result.hasTargets() && (result.getBestTarget().getFiducialId() == 7 || result.getBestTarget().getFiducialId()==4)) {



			// PhotonTrackedTarget target = result.getBestTarget();

			distance = PhotonUtils.calculateDistanceToTargetMeters(
				Units.inchesToMeters(9.451),
				Constants.VisionConstants.kAprilTagHeight,
				Constants.VisionConstants.kCameraPitchRadians,
				Units.degreesToRadians(result.getBestTarget().getPitch()));

		

			theta = Units.radiansToDegrees(Math.atan(Constants.VisionConstants.kspeakerHeight/distance));



			x = result.getBestTarget().getDetectedCorners().stream().mapToDouble((a) -> a.x).sum() / 4;
			proportion = distance / 8.3;


			Robot.shintake.setShooter(1,1);
			
			armAngle = ((53-theta)+6.0)-(8*Math.pow(proportion,4)); // 6, 2 , 5,4
			if(distance > 3.5)armAngle-=distance*.9;
			// armAngle = (3.33*Math.pow(distance,3) - 32.5 *(Math.pow(distance,2) )+ 108.17*distance -92.375);

			if(armAngle<0) armAngle=0;
			if(armAngle>53)armAngle = 53;

			Robot.arm.armLeft.set(armpid.calculate(Robot.arm.getArmPot(),armAngle));

			
			



			

      Robot.m_drivetrain.drive(-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband), drivepid.calculate(x, setPoint), false, false);
			

			

			

			SmartDashboard.putNumber("(Photon) Distance to AprilTag (feet)", Units.metersToFeet(distance));
			SmartDashboard.putNumber("arm set angle",armAngle);
			SmartDashboard.putNumber("theta to speaker", theta);
			SmartDashboard.putNumber("target pitch", Units.degreesToRadians(result.getBestTarget().getPitch()));
			SmartDashboard.putNumber("align", x);


			

			
		}

		else{
			Robot.arm.hold();
			Robot.m_drivetrain.drive(-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),  -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(),OIConstants.kDriveDeadband) , false, false);
			RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0);
			RobotContainer.m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
			Robot.shintake.setShooter(0, 0);

		}
	}



	@Override
  public void end(boolean interrupted) {

  Robot.arm.armLeft.set(armpid.calculate(Robot.arm.getArmPot(),armAngle));

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armpid.atSetpoint() || timer.get()>4;//timer.get()>5;
  }
}
