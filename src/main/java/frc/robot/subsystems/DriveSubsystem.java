/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.kLeftMotor1Port;
import static frc.robot.Constants.DriveConstants.kLeftMotor2Port;
import static frc.robot.Constants.DriveConstants.kRightMotor1Port;
import static frc.robot.Constants.DriveConstants.kRightMotor2Port;

import static frc.robot.Constants.DriveConstants.kEncoderDistancePerPulse;
import static frc.robot.Constants.DriveConstants.kLeftEncoderPorts;
import static frc.robot.Constants.DriveConstants.kLeftEncoderReversed;
import static frc.robot.Constants.DriveConstants.kRightEncoderPorts;
import static frc.robot.Constants.DriveConstants.kRightEncoderReversed;
import static frc.robot.Constants.DriveConstants.kGyroReversed;


/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  // The motor on the left side of drive
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(new WPI_TalonSRX(kLeftMotor1Port), 
          new WPI_VictorSPX(kLeftMotor2Port));

  // The motor on the right side of drive
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(new WPI_TalonSRX(kRightMotor1Port), 
          new WPI_VictorSPX(kRightMotor2Port));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
  new Encoder(kLeftEncoderPorts[0], kLeftEncoderPorts[1], kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
  new Encoder(kRightEncoderPorts[0], kRightEncoderPorts[1], kRightEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADIS16448_IMU();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem
   */
  public DriveSubsystem(){
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
  }

  /**
   * Returens the currently-estimated pose of the robot.
   * 
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),m_rightEncoder.getRate());
  }

  /** 
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  
  /**
   * Drives the robot using arcade controls.
   *  
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
  }



  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

/**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((m_leftEncoder.getDistance()+m_rightEncoder.getDistance())/2);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
  
public void setMaxOutput(double maxOutput){
  m_drive.setMaxOutput(maxOutput);
}
}
