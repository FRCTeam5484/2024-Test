package frc.robot.classes;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class swerveModule {
    private final CANcoder rotationEncoder;

    private final CANSparkMax drivingSparkMax;
    private final CANSparkMax turningSparkMax;

    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkPIDController drivingPIDController;
    private final SparkPIDController turningPIDController;

    private double angularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public swerveModule(int drivingCANId, int turningCANId, int canCoderId, double angularOffset) {
        rotationEncoder = new CANcoder(canCoderId);
            CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
            canCoderConfiguration.MagnetSensor.MagnetOffset = angularOffset;
            canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            //canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue. AbsoluteSensorRange.Unsigned_0_to_360;
            rotationEncoder.getConfigurator().apply(canCoderConfiguration);

        rotationEncoder.getPosition().setUpdateFrequency(100);
        rotationEncoder.getVelocity().setUpdateFrequency(100);
        drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        drivingSparkMax.restoreFactoryDefaults();
        turningSparkMax.restoreFactoryDefaults();

        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getEncoder();
        drivingPIDController = drivingSparkMax.getPIDController();
        turningPIDController = turningSparkMax.getPIDController();
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        drivingEncoder.setPositionConversionFactor(moduleConstants.kDrivingEncoderPositionFactor);
        drivingEncoder.setVelocityConversionFactor(moduleConstants.kDrivingEncoderVelocityFactor);

        turningEncoder.setPositionConversionFactor(moduleConstants.kTurningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(moduleConstants.kTurningEncoderVelocityFactor);

        //turningEncoder.setInverted(moduleConstants.kTurningEncoderInverted);

        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(moduleConstants.kTurningEncoderPositionPIDMinInput);
        turningPIDController.setPositionPIDWrappingMaxInput(moduleConstants.kTurningEncoderPositionPIDMaxInput);

        drivingPIDController.setP(moduleConstants.kDrivingP);
        drivingPIDController.setI(moduleConstants.kDrivingI);
        drivingPIDController.setD(moduleConstants.kDrivingD);
        drivingPIDController.setFF(moduleConstants.kDrivingFF);
        drivingPIDController.setOutputRange(moduleConstants.kDrivingMinOutput, moduleConstants.kDrivingMaxOutput);

        turningPIDController.setP(moduleConstants.kTurningP);
        turningPIDController.setI(moduleConstants.kTurningI);
        turningPIDController.setD(moduleConstants.kTurningD);
        turningPIDController.setFF(moduleConstants.kTurningFF);
        turningPIDController.setOutputRange(moduleConstants.kTurningMinOutput, moduleConstants.kTurningMaxOutput);

        drivingSparkMax.setIdleMode(moduleConstants.kDrivingMotorIdleMode);
        turningSparkMax.setIdleMode(moduleConstants.kTurningMotorIdleMode);
        drivingSparkMax.setSmartCurrentLimit(moduleConstants.kDrivingMotorCurrentLimit);
        turningSparkMax.setSmartCurrentLimit(moduleConstants.kTurningMotorCurrentLimit);

        drivingSparkMax.burnFlash();
        turningSparkMax.burnFlash();

        this.angularOffset = angularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }
    
    public SwerveModuleState getState() {return new SwerveModuleState(drivingEncoder.getVelocity(), getRotation2d()); }
    public SwerveModulePosition getPosition() {return new SwerveModulePosition(drivingEncoder.getPosition(), getRotation2d());}
    public double GetModuleAngle() { return rotationEncoder.getAbsolutePosition().getValueAsDouble(); }
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, getRotation2d());

        if(Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01){
            stopModule();
            return;
        }

        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        this.desiredState = desiredState;
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0); turningEncoder.setPosition(0);
    }
    public double getAngle() { 
        return rotationEncoder.getAbsolutePosition().getValueAsDouble(); 
    }
    public Rotation2d getRotation2d() { 
        return Rotation2d.fromDegrees(getAngle()); 
    }
    public void stopModule(){ 
        drivingSparkMax.stopMotor(); turningSparkMax.stopMotor();
    }
}
