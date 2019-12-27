/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;
  private final BooleanSupplier m_boost;

  /**
   * 
   * @param subsystem The drive subsystem this command will run.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */

  public DefaultDrive(DriveSubsystem subsystem,DoubleSupplier forward, DoubleSupplier rotation, BooleanSupplier boost) {
    m_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    m_boost = boost;
    addRequirements(m_drive);
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (m_boost.getAsBoolean()) {
      m_drive.arcadeDrive(0.85*m_forward.getAsDouble(),0.85*m_rotation.getAsDouble());
    }
    else
    {
      m_drive.arcadeDrive(0.6*m_forward.getAsDouble(),0.6*m_rotation.getAsDouble());
    }
  }

}
