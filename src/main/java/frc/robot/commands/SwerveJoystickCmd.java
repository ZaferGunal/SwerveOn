package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java .util.function.Supplier;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.Constants.CmdSys;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
public class SwerveJoystickCmd extends CommandBase{

    private  SwerveSystem swerveSystem;
    private  Supplier<Double> xSpdFunc;
    private  Supplier<Double> ySpdFunc;
    private  Supplier<Double> turningSpdFunc;
    
    private  SlewRateLimiter xLimiter, yLimiter,turningLimiter;

    public SwerveJoystickCmd(SwerveSystem swerveSystem_, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction){
        this.swerveSystem = swerveSystem_;
        this.xSpdFunc = xSpdFunction;
        this.ySpdFunc = ySpdFunction;
        this.turningSpdFunc = turningSpdFunction;
        xLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelerationUnitsPerSec);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAccelerationUnitsPerSec);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleopMaxAngularAcclerationPerSec);
        addRequirements(swerveSystem);

    }

    @Override
    public void end(boolean interrupted) {
       swerveSystem.stopSwerve();
    }

    @Override
    public void execute() {
       double xSpd = xSpdFunc.get();
       double ySpd = ySpdFunc.get();
       double turningSpd = turningSpdFunc.get();

       xSpd = Math.abs(xSpd) < CmdSys.kDeadBand ? 0.0: xSpd;
       ySpd = Math.abs(ySpd) < CmdSys.kDeadBand ? 0.0 : ySpd;                       // ufak oynamaları göz ardı eder
       turningSpd = Math.abs(turningSpd) < CmdSys.kDeadBand ? 0.0 : turningSpd;            

       xSpd = xLimiter.calculate(xSpd);
       ySpd = yLimiter.calculate(ySpd);                                    //ivmeyi sınırlayarak daha kolay sürülmesini sağlar
       turningSpd = turningLimiter.calculate(turningSpd);

       xSpd *= DriveConstants.kAppropriateMaxMeterSpd; 
       ySpd *= DriveConstants.kAppropriateMaxMeterSpd;            // joystickden gelen -1 to 1 skalasındaki hızı perSec e ayarlar
       turningSpd *= DriveConstants.kAppropriateMaxRadSpd;
    
       ChassisSpeeds chassisSpeeds;
       if(CmdSys.fieldOriented){                                                                  
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpd,ySpd,turningSpd,swerveSystem.getRotation2d());  // gyrodan veri alır
       }
       else{
        chassisSpeeds = new ChassisSpeeds(xSpd,ySpd,turningSpd);
       }

       SwerveModuleState[] swerveModules = CmdSys.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
       
       swerveSystem.setSwerveModuleStates(swerveModules);

       

    }

    @Override
    public void initialize() {
      
    }

    @Override
    public boolean isFinished() {
     return false;
    }


    

}
