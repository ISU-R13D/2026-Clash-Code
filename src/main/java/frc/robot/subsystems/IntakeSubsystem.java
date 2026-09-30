package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax actuatorMotor = new SparkMax(IntakeConstants.ACTUATOR_ID, MotorType.kBrushless);
    private final SparkMax shakeMotor = new SparkMax(IntakeConstants.SHAKE_ID, MotorType.kBrushless); 
    private final SparkMaxConfig actuatorConfig = new SparkMaxConfig();
    private final SparkMaxConfig shakeConfig = new SparkMaxConfig();

    
    public IntakeSubsystem() {
        actuatorMotor.configure(actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shakeMotor.configure(shakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    public void setActuatorSpeed(double speed) {
        actuatorMotor.set(speed);
    }

    public void stopActuator() {
        actuatorMotor.stopMotor();
    }

    public void setShakeSpeed(double speed) {
        shakeMotor.set(speed);
    }
    
    public void stopShake() {
        shakeMotor.stopMotor();
    }
}
