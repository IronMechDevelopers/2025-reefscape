// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CageDoorConstants;
import frc.robot.Constants.MotorIds;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CageDoorSubsystem extends SubsystemBase {
  
  private SparkMax cagedoorMotor;
    private double currentPercentage;
  
    /** Creates a new ClimberSubsystem. */
    public CageDoorSubsystem() {
      super();
      this.cagedoorMotor = new SparkMax(MotorIds.kCageDoorMotorCanId,
          MotorType.kBrushed);
      currentPercentage = 0;
  
    }
  
    public void setMotor(double speed) {
      this.cagedoorMotor.set(speed);
      currentPercentage = speed;
    }
  
    public void stopMotor() {
      setMotor(0);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("CageDoorSubsystem Percentage", currentPercentage);
    }
    
    public Command cagedooropenCommand() {
      return Commands.startEnd(() -> setMotor(CageDoorConstants.cageopenSpeed), ()
    -> stopMotor(), this);
  }

  public Command cagedoorcloseCommand() {
    return Commands.startEnd(() -> setMotor(CageDoorConstants.cagecloseSpeed),
    () -> stopMotor(), this);
  }
}