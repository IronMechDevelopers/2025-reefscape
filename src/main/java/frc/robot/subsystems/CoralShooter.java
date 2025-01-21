package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorIds;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralShooter extends SubsystemBase {
    private SparkMax shooterTopMotor;
    public double maxRPM;
    private RelativeEncoder m_encoderTop;
  /** Creates a new ExampleSubsystem. */
  public CoralShooter() {

        this.shooterTopMotor = new SparkMax(MotorIds.kTopShooterMotor, MotorType.kBrushless);
  
        /**shooterTopMotor.restoreFactoryDefaults(); */
  
        m_encoderTop = shooterTopMotor.getEncoder();
  
        // shooterTopMotor.setInverted(true);
  
        maxRPM = 5700;
    }
  
    public void stopMotor() {
        setMotorToPercent(0);
    }
  
    public void setMotorToPercent(double speed) {
        if (speed > 1) {
            speed = 1;
        }
  
        shooterTopMotor.set(speed);
        SmartDashboard.putNumber("Shooter Percent", speed);
    }
  
    public void setMotorToPercent(String strength) {
        double percent = SmartDashboard.getNumber(strength, 0.45);
        setMotorToPercent(percent);
    }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shootCoralCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setMotorToPercent(speed);
        }).andThen(() -> {setMotorToPercent(0.0);});  // TODO: check if this is the right thing to do
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}