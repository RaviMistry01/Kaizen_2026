// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  
  private final boolean kUseLimelight = true;
  private boolean autoVisionInitialized = false;
  private double autoStartTime = 0.0;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for
     * on-field use.
     * Users typically need to provide a standard deviation that scales with the
     * distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible,
     * though exact implementation
     * of how to use vision should be tuned per-robot and to the team's
     * specification.
     */


    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight-l", headingDeg, 0, 0, 0, 0, 0);
      // var llMeasurement =
      // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      // if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps)
      // < 2.0) {
      // m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
      // llMeasurement.timestampSeconds);
      // }
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.drivetrain.applyRequest(() -> brake);
    
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // ✅ Tell drivetrain that AUTO has started
    // m_robotContainer.drivetrain.setAutoRunning(true);
    // autoVisionInitialized = false;
    // autoStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

    // // ✅ Time since auto started
    // double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() -
    // autoStartTime;

    // // ✅ Disable vision update after 0.25s
    // if (elapsed > 0.25) {
    // m_robotContainer.drivetrain.setAutoRunning(true); // disables vision
    // internally
    // }

  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void robotInit() {

  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.drivetrain.simulationPeriodic();
  }
}
