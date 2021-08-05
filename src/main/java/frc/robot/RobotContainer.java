// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.operatorinterface.DualShock4Controller;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private final DualShock4Controller pilotInput = new DualShock4Controller(OIConstants.PILOT_JOYSTICK_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final Command rotateClockwiseCommand = new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem);
  private final Command rotateCounterClockwiseCommand = new RunCommand(() -> driveSubsystem.arcadeDrive(0, -0.5), driveSubsystem);

  private final Command manualDriveCommand = new RunCommand(() -> driveSubsystem.arcadeDrive(
    pilotInput.getY(Hand.kLeft),
    pilotInput.getX(Hand.kRight)),
    driveSubsystem
  );

  SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(manualDriveCommand);
    configureButtonBindings();
    initAutonomousChooser();
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  private void configureButtonBindings() {
    pilotInput.getLeftBumper()
      .whenPressed(() -> driveSubsystem.setFastSpeed())
      .whenReleased(() -> driveSubsystem.setSlowSpeed());
  }

  private void initAutonomousChooser() {
    autonomousChooser.setDefaultOption("Simple Trajectory", getSimpleTrajectoryCommand());
    autonomousChooser.addOption("Rotate Clockwise", rotateClockwiseCommand);
    autonomousChooser.addOption("Rotate Counter Clockwise", rotateCounterClockwiseCommand);
    Shuffleboard.getTab("Autonomous").add(autonomousChooser);
  }

  private Command getSimpleTrajectoryCommand() {
    
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.FeedForwardGains.ksVolts,
        DriveConstants.FeedForwardGains.kvVoltSecondsPerMeter,
        DriveConstants.FeedForwardGains.kaVoltSecondsSquaredPerMeter
      ),
      DriveConstants.DriveKinematics,
      10
    );

    TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.MaxSpeedMetersPerSecond,
      DriveConstants.MaxAccelerationMetersPerSecondSquared
    )
    .setKinematics(DriveConstants.DriveKinematics)
    .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1, 4, new Rotation2d(0)),
      List.of(
        new Translation2d(2, 5),
        new Translation2d(3, 3)
      ),
      new Pose2d(4, 4, new Rotation2d()),
      config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveSubsystem::getPose,
        new RamseteController(DriveConstants.RamseteB, DriveConstants.RamseteZeta),
        new SimpleMotorFeedforward(
          DriveConstants.FeedForwardGains.ksVolts,
          DriveConstants.FeedForwardGains.kvVoltSecondsPerMeter,
          DriveConstants.FeedForwardGains.kaVoltSecondsSquaredPerMeter
        ),
        DriveConstants.DriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.FeedbackGains.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.FeedbackGains.kPDriveVel, 0, 0),
        driveSubsystem::tankDriveVolts,
        driveSubsystem
    );

    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}

