// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIds;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  
  private SparkMax climberMotor;
    private double currentPercentage;
  
    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
      super();
      this.climberMotor = new SparkMax(MotorIds.kClimberMotorCanId,
          MotorType.kBrushed);
      currentPercentage = 0;
  
    }
  
    public void setMotor(double speed) {
      this.climberMotor.set(speed);
      currentPercentage = speed;
    }
  
    public void stopMotor() {
      setMotor(0);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("ClimberSubsystem Percentage", currentPercentage);
    }
    
    public Command climberUpCommand() {
      return Commands.startEnd(() -> setMotor(ClimberConstants.climberSpeedUp), ()
    -> stopMotor(), this);
  }

  public Command climberDownCommand() {
    return Commands.startEnd(() -> setMotor(ClimberConstants.climberSpeedDown),
    () -> stopMotor(), this);
  }
}