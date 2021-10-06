// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax leftFrontMotor = new PWMSparkMax(Constants.MOTOR_LEFT_FRONT);
  private final PWMSparkMax leftRearMotor = new PWMSparkMax(Constants.MOTOR_LEFT_REAR);
 
  private final PWMSparkMax rightFrontMotor = new PWMSparkMax(Constants.MOTOR_RIGHT_FRONT);
  private final PWMSparkMax rightRearMotor = new PWMSparkMax(Constants.MOTOR_RIGHT_REAR);

  private final SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
  private final SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  private final Encoder leftEncoder = new Encoder(Constants.ENCODER_LEFT_A, Constants.ENCODER_LEFT_B);
  private final Encoder rightEncoder = new Encoder(Constants.ENCODER_RIGHT_A, Constants.ENCODER_RIGHT_B);

  private final AnalogGyro gyro = new AnalogGyro(Constants.GYRO_CHANNEL);

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  private Field2d field = new Field2d();

  private final DriveSubsystemSimulation simulation = new DriveSubsystemSimulation(
    gyro,
    leftEncoder,
    rightEncoder,
    () -> leftMotorGroup.get(),
    () -> rightMotorGroup.get()
  );

  public DriveSubsystem() {
    drivetrain.setMaxOutput(0.5);
    initEncoders();
    initDashboard();
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateRobotPositionOnField();
  }

  public void arcadeDrive(double speed, double rotation) {
    drivetrain.arcadeDrive(speed, rotation);
  }

  public void simulationPeriodic() {
    simulation.update();
  }
  
  private void initEncoders() {
    double distancePerPulse = Math.PI * Constants.WHEEL_DIAMETER / Constants.ENCODER_RESOLUTION;

    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);

    resetEncoders();
  }

  private void  initDashboard() {
    SmartDashboard.putData("Field", field);
  }

  private void updateOdometry() {
    odometry.update(
      gyro.getRotation2d(),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );
  }

  private void updateRobotPositionOnField() {
    Pose2d pose = odometry.getPoseMeters();
    field.setRobotPose(pose);
  }

  private void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
}
