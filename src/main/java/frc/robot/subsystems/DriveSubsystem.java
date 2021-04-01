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
    double r = Math.sqrt((Constants.HIGHT * Constants.HIGHT) + (Constants.WIDTH * Constants.WIDTH));

    double yPos = y.getAsDouble();
    double xPos = x.getAsDouble();

    yPos *= -1;

    double a = xPos - xPos * (Constants.HIGHT / r);
    double b = xPos + xPos * (Constants.HIGHT / r);
    double c = yPos - xPos * (Constants.WIDTH / r);
    double d = yPos + xPos * (Constants.WIDTH / r);

    double backLeftSpeed = Math.sqrt((a * a) + (b * b));
    double backRightSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    double backLeftAngle = Math.atan2(a, d) * 180 / Math.PI;
    double backRightAngle = Math.atan2(a, c) * 180 / Math.PI;
    double frontLeftAngle = Math.atan2(b, d) * 180 / Math.PI;
    double frontRightAngle = Math.atan2(b, c) * 180 / Math.PI;

    this.frontRightMotion.set(frontRightSpeed);
    this.frontLeftMotion.set(frontLeftSpeed);
    this.backLeftMotion.set(backLeftSpeed);
    this.backRightMotion.set(backRightSpeed);

    this.frontRightAngle.set(frontRightAngle);
    this.frontLeftAngle.set(frontLeftAngle);
    this.backLeftAngle.set(backLeftAngle);
    this.backRightAngle.set(backRightAngle);

    SmartDashboard.putNumber("Back Left Speed", backLeftSpeed);
    SmartDashboard.putNumber("Back Right Speed", backRightSpeed);
    SmartDashboard.putNumber("Front Right Speed", frontRightSpeed);
    SmartDashboard.putNumber("Front Left Speed", frontLeftSpeed);

    SmartDashboard.putNumber("Back Left Angle", backLeftAngle);
    SmartDashboard.putNumber("Back Right Angle", backRightAngle);
    SmartDashboard.putNumber("Front Left Angle", frontLeftAngle);
    SmartDashboard.putNumber("Front Right Angle", frontRightAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
