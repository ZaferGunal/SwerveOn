package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import edu.wpi.first.math.Drake;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.cancoConfig.CANCoderUtil.CCUsage;
import frc.cancoConfig.CANCoderUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
public class SwerveModule  extends SubsystemBase{

    CANSparkMax driveMotor;
    CANSparkMax angleMotor;

    RelativeEncoder driveEncoder;
    RelativeEncoder angleEncoder;
        
    CANCoder cancoder ;
     public final boolean cancoderDirection;
      public final double cancoderOffsetRad;

    PIDController anglePidController;



public SwerveModule(int driveMotorID,int angleMotorID, int CANCoderID, boolean driveMotorReversed,
boolean angleMotorReversed, double CANCoderOffsetRad, boolean CANCoderReversed  ){

    ConfigurationCTRE();

this.driveMotor = new CANSparkMax(driveMotorID,MotorType.kBrushless);     //drive motor
driveMotor.setInverted(driveMotorReversed);


this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);    // angleMotor
angleMotor.setInverted(angleMotorReversed);

driveEncoder = driveMotor.getEncoder();
angleEncoder = angleMotor.getEncoder();                          //relativeEncoder connection



cancoder = new CANCoder(CANCoderID);
this.cancoderOffsetRad = CANCoderOffsetRad;
this.cancoderDirection = CANCoderReversed;
                                                             // need to config

driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRotation2Meter);
driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);
angleEncoder.setPositionConversionFactor(SwerveModuleConstants.kAngleEncoderRotation2rad);
angleEncoder.setVelocityConversionFactor(SwerveModuleConstants.kAngleEncoderRPM2MeterPerSec);

anglePidController = new PIDController(SwerveModuleConstants.kP_angle,0.0,0.0);
anglePidController.enableContinuousInput(-Math.PI, Math.PI);


     resetEncoders();     
                                       


}






public void resetEncoders(){
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(cancoder.getAbsolutePosition()-cancoderOffsetRad);                  // need to get absolute angle
}


public void ConfigurationCTRE(){
CANCoderConfiguration config = new CANCoderConfiguration();
config.absoluteSensorRange=AbsoluteSensorRange.Unsigned_0_to_360;
config.initializationStrategy= SensorInitializationStrategy.BootToAbsolutePosition;
config.sensorDirection= cancoderDirection;
config.sensorTimeBase= SensorTimeBase.PerSecond;


cancoder.configFactoryDefault();
CANCoderUtil.setCANCoderBusUsage(cancoder, CCUsage.kMinimal);
cancoder.configAllSettings(config);
}

public double getDrivePosition(){
    return driveEncoder.getPosition();
}

public double getDriveVelocity(){
    return driveEncoder.getVelocity();
}

public double getAnglePosition(){
    return angleEncoder.getPosition();
}

public double getAngleVelocity(){
    return angleEncoder.getVelocity();
}

 public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
 }

 public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMeterPerSec);
    angleMotor.set(anglePidController.calculate(getAnglePosition(), state.angle.getRadians()));
   
}

public void stop(){
    driveMotor.set(0.0);
    angleMotor.set(0.0);
}

}
