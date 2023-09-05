package frc.robot.subsystems;
import  frc.robot.subsystems.SwerveModule;
import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.Constants;
import frc.robot.Constants.ModuleFrontLeft;
import frc.robot.Constants.ModuleFrontRight;
import frc.robot.Constants.ModuleRearLeft;
import frc.robot.Constants.ModuleRearRight;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.Pigeon2Configuration;


public class SwerveSystem extends SubsystemBase {
    
public SwerveModule frontLeftModule = new SwerveModule(ModuleFrontLeft.driveMotorID, 
ModuleFrontLeft.angleMotorID,
 ModuleFrontLeft.CANCoderID,
ModuleFrontLeft.driveMotorReversed,
ModuleFrontLeft.angleMotorReversed,
ModuleFrontLeft.CANCoderOffsetRad,
ModuleFrontLeft.CANCoderReversed);

public SwerveModule frontRightModule = new SwerveModule(ModuleFrontRight.driveMotorID, 
ModuleFrontRight.angleMotorID,
 ModuleFrontRight.CANCoderID,
ModuleFrontRight.driveMotorReversed,
ModuleFrontRight.angleMotorReversed,
ModuleFrontRight.CANCoderOffsetRad,
ModuleFrontRight.CANCoderReversed);


public SwerveModule rearLeftModule = new SwerveModule(ModuleRearLeft.driveMotorID, 
ModuleRearLeft.angleMotorID,
 ModuleRearLeft.CANCoderID,
ModuleRearLeft.driveMotorReversed,
ModuleRearLeft.angleMotorReversed,
ModuleRearLeft.CANCoderOffsetRad,
ModuleRearLeft.CANCoderReversed);


public SwerveModule rearRightModule = new SwerveModule(ModuleRearRight.driveMotorID, 
ModuleRearRight.angleMotorID,
 ModuleRearRight.CANCoderID,
ModuleRearRight.driveMotorReversed,
ModuleRearRight.angleMotorReversed,
ModuleRearRight.CANCoderOffsetRad,
ModuleRearRight.CANCoderReversed);

Pigeon2 gyro =  new Pigeon2(11);            /// !!!!!!!!!!!









public SwerveSystem(){
    new Thread( () -> {
        try {
            Thread.sleep(1000);
            resetPigeon();
        } catch (Exception e) {
            
        }
    }).start();
}

public void resetPigeon(){
    gyro.configFactoryDefault();
    gyro.clearStickyFaults();
    Pigeon2Configuration conPig = new Pigeon2Configuration();
    conPig.EnableCompass = true;
    conPig.MountPosePitch = 0.0;
    conPig.MountPoseRoll = 0.0;
    conPig.MountPoseYaw = 0.0;
    gyro.configAllSettings(conPig);
}

public double getHeading(){
    return Math.IEEEremainder(gyro.getAbsoluteCompassHeading(), 360);         // 360 a bölünmese de olabilir
}

public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
}

public void stopSwerve(){
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
}


public void setSwerveModuleStates(  SwerveModuleState[] desiredStates){
 SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMeterPerSec);
 frontLeftModule.setDesiredState(desiredStates[0]);
 frontRightModule.setDesiredState(desiredStates[1]);
 rearLeftModule.setDesiredState(desiredStates[2]);
 rearRightModule.setDesiredState(desiredStates[3]);
}












}
