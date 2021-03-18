// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class DriveSubsystem extends SubsystemBase {
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

  // Convert to module states
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

  // Front left module state
  SwerveModuleState frontLeft = moduleStates[0];

  // Front right module state
  SwerveModuleState frontRight = moduleStates[1];

  // Back left module state
  SwerveModuleState backLeft = moduleStates[2];

  // Back right module state
  SwerveModuleState backRight = moduleStates[3];

  private final WPI_TalonFX frontLeftMotion = new WPI_TalonFX(Constants.frontLeftMotorCANID);
  private final WPI_TalonFX frontRightMotion = new WPI_TalonFX(Constants.frontRightMotorCANID);
  private final WPI_TalonFX backLeftMotion = new WPI_TalonFX(Constants.backLeftMotorCANID);
  private final WPI_TalonFX backRightMotion = new WPI_TalonFX(Constants.backRightMotorCANID);

  private final WPI_TalonFX frontLeftAngle = new WPI_TalonFX(Constants.frontLeftAngleCANID);
  private final WPI_TalonFX frontRightAngle = new WPI_TalonFX(Constants.frontRightAngleCANID);
  private final WPI_TalonFX backLeftAngle = new WPI_TalonFX(Constants.backLeftAngleCANID);
  private final WPI_TalonFX backRightAngle = new WPI_TalonFX(Constants.backRightAngleCANID);

  private final CANCoder canCoderFrontLeft = new CANCoder(Constants.CANCoderFrontLeftID);
  private final CANCoder canCoderFrontRight = new CANCoder(Constants.CANCoderFrontRightID);
  private final CANCoder canCoderBackLeft = new CANCoder(Constants.CANCoderBackLeftID);
  private final CANCoder canCoderBackRight = new CANCoder(Constants.CANCoderBackRightID);

  /** Creates a new Drive. */
  public DriveSubsystem() {
    this.frontRightMotion.configFactoryDefault();
    this.frontLeftMotion.configFactoryDefault();
    this.backLeftMotion.configFactoryDefault();
    this.backRightMotion.configFactoryDefault();

    this.frontRightAngle.configFactoryDefault();
    this.frontLeftAngle.configFactoryDefault();
    this.backLeftAngle.configFactoryDefault();
    this.backRightAngle.configFactoryDefault();

    this.backLeftAngle.set(0);
    this.backRightAngle.set(0);
    this.frontLeftAngle.set(0);
    this.frontRightAngle.set(0);

    this.frontRightMotion.setNeutralMode(NeutralMode.Brake);
    this.frontLeftMotion.setNeutralMode(NeutralMode.Brake);
    this.backLeftMotion.setNeutralMode(NeutralMode.Brake);
    this.backRightMotion.setNeutralMode(NeutralMode.Brake);

    this.frontRightAngle.setNeutralMode(NeutralMode.Brake);
    this.frontLeftAngle.setNeutralMode(NeutralMode.Brake);
    this.backLeftAngle.setNeutralMode(NeutralMode.Brake);
    this.backRightAngle.setNeutralMode(NeutralMode.Brake);
  }

  private double GetMotorSpeed(WPI_TalonFX motor) {
    return motor.getSelectedSensorVelocity();
  }

  private double GetMotorPos(WPI_TalonFX motor) {
    return motor.getSelectedSensorPosition();
  }

  public void Drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        new Rotation2d(frontLeftAngle.getSelectedSensorPosition()));

    var frontRightOptimized = SwerveModuleState.optimize(frontRight,
        new Rotation2d(frontRightAngle.getSelectedSensorPosition()));

    var backLeftOptimized = SwerveModuleState.optimize(backLeft,
        new Rotation2d(frontLeftAngle.getSelectedSensorPosition()));

    var backRightOptimized = SwerveModuleState.optimize(backRight,
        new Rotation2d(frontLeftAngle.getSelectedSensorPosition()));

    var frontLeftState = new SwerveModuleState(GetMotorPos(frontLeftMotion),
        Rotation2d.fromDegrees(canCoderFrontLeft.getAbsolutePosition()));

    var frontRightState = new SwerveModuleState(GetMotorPos(frontRightMotion),
        Rotation2d.fromDegrees(canCoderFrontRight.getAbsolutePosition()));

    var backLeftState = new SwerveModuleState(GetMotorPos(backLeftMotion),
        Rotation2d.fromDegrees(canCoderBackLeft.getAbsolutePosition()));

    var backRightState = new SwerveModuleState(GetMotorPos(backRightMotion),
        Rotation2d.fromDegrees(canCoderBackRight.getAbsolutePosition()));

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState,
        backRightState);

    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    double angular = chassisSpeeds.omegaRadiansPerSecond;

    System.out.println("forward " + forward);
    System.out.println("sideways " + sideways);
    System.out.println("angular " + angular);

    System.out.println("back-left position: " + canCoderBackLeft.getPosition());
    System.out.println("front-left position: " + canCoderFrontLeft.getPosition());
    System.out.println("front-right position: " + canCoderFrontRight.getPosition());
    System.out.println("back-right position: " + canCoderBackRight.getPosition());

    System.out.println("back-left position Absolute: " + canCoderBackLeft.getAbsolutePosition());
    System.out.println("front-left position Absolute: " + canCoderFrontLeft.getAbsolutePosition());
    System.out.println("front-right position Absolute: " + canCoderFrontRight.getAbsolutePosition());
    System.out.println("back-right position Absolute: " + canCoderBackRight.getAbsolutePosition());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
