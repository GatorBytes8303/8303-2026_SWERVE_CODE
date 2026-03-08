package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants;

public class SuckerSubsystem extends SubsystemBase {

    private final SparkMax suckerMotor;

    /**
     * This subsytem that controls the sucker.
     */
    public SuckerSubsystem () {

    // Set up the sucker motor as a brushless motor
    suckerMotor = new SparkMax(ScorerConstants.kSuckerMotorCanID, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    suckerMotor.setCANTimeout(250);

    // Create and apply configuration for Sucker motor. Voltage compensation helps
    // the sucker behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the Sucker stalls.
    SparkMaxConfig suckerConfig = new SparkMaxConfig();
    suckerConfig.voltageCompensation(ScorerConstants.kScorerMotorVolatageCompensation);
    suckerConfig.smartCurrentLimit(ScorerConstants.kScorerMotorCurrentLimit);
    suckerConfig.idleMode(IdleMode.kBrake);
    suckerMotor.configure(suckerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    /**
     * Use to run the spitter, can be set to run from 100% to -100%.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runSucker(double speed){
        suckerMotor.set(speed);
    }

}