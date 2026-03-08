package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants;

public class SpitterSubsystem extends SubsystemBase {

    private final SparkMax spitterMotor;

    /**
     * This subsytem that controls the spitter.
     */
    public SpitterSubsystem () {

    // Set up the spitter motor as a brushless motor
    spitterMotor = new SparkMax(ScorerConstants.kSpitterMotorCanID, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    spitterMotor.setCANTimeout(250);

    // Create and apply configuration for Spitter motor. Voltage compensation helps
    // the spitter behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the Spitter stalls.
    SparkMaxConfig spitterConfig = new SparkMaxConfig();
    spitterConfig.voltageCompensation(ScorerConstants.kScorerMotorVolatageCompensation);
    spitterConfig.smartCurrentLimit(ScorerConstants.kScorerMotorCurrentLimit);
    spitterConfig.idleMode(IdleMode.kBrake);
    spitterMotor.configure(spitterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    /**
     * Use to run the spitter, can be set to run from 100% to -100%.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runSpitter(double speed){
        spitterMotor.set(speed);
    }

}