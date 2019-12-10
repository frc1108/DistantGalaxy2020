/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.kLeftMotor1Port;
import static frc.robot.Constants.DriveConstants.kLeftMotor2Port;
import static frc.robot.Constants.DriveConstants.kRightMotor1Port;
import static frc.robot.Constants.DriveConstants.kRightMotor2Port;

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

public DriveSubsystem(){

}

public void arcadeDrive(double fwd, double rot){
  m_drive.arcadeDrive(-fwd, rot);
}

public void setMaxOutput(double maxOutput){
  m_drive.setMaxOutput(maxOutput);
}
}
