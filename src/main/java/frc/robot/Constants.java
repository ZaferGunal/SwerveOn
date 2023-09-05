// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


    public static class SwerveModuleConstants{
      
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
      public static final double kDriveMotorGearRatio = 1 /6.75;
      public static final double kAngleMotorGearRatio = 1 / 6.75;
      public static final double kDriveEncoderRotation2Meter = kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;
      public static final double kAngleEncoderRotation2rad = 2 * Math.PI * kAngleMotorGearRatio;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRotation2Meter / 60;
      public static final double kAngleEncoderRPM2MeterPerSec = kAngleEncoderRotation2rad / 60;
      public static final double kP_angle = 0.5;
      



    }
    public static class DriveConstants{






      public static final double kMaxSpeedMeterPerSec = 5.0;
      public static final double kMaxAngularSpeedRadPerSec = 2 * 2 *Math.PI;  // saniyede 2 tur

      
      
      public static final double kTeleopMaxAccelerationUnitsPerSec = 3.0;         // geç hızlanıyorsa arttır
      public static final double kTeleopMaxAngularAcclerationPerSec = 3.0;

      public static final double kAppropriateMaxMeterSpd = kMaxSpeedMeterPerSec / 4;          // çok hızlı azalt
      public static final double kAppropriateMaxRadSpd= kMaxAngularSpeedRadPerSec / 4;

      
    }
    public static class ModuleFrontLeft{
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 1;
      public static final int CANCoderID = 8;
      public static final boolean driveMotorReversed = false;
      public static final boolean angleMotorReversed = false;
      public static final double CANCoderOffsetRad = 000;        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
      public static final boolean CANCoderReversed = false;
    }

    public static class ModuleFrontRight{
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int CANCoderID = 9;
      public static final boolean driveMotorReversed = false;
      public static final boolean angleMotorReversed = false;
      public static final double CANCoderOffsetRad = 000;        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
      public static final boolean CANCoderReversed = false;
    }

    public static class ModuleRearLeft{
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int CANCoderID = 10;
      public static final boolean driveMotorReversed = false;
      public static final boolean angleMotorReversed = false;
      public static final double CANCoderOffsetRad = 000;        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
      public static final boolean CANCoderReversed = false;
    }
    public static class ModuleRearRight{
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int CANCoderID = 11;
      public static final boolean driveMotorReversed = false;
      public static final boolean angleMotorReversed = false;
      public static final double CANCoderOffsetRad = 000;        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
      public static final boolean CANCoderReversed = false;
    }




      public static class CmdSys{
        public static final double kDeadBand = 0.05;   // stick hassaslığına göre ayarlayabilirsin 
        public static final boolean fieldOriented = true;

        public static final double  kRobotWidth = Units.inchesToMeters(20);
        public static final double kRobotLenght = Units.inchesToMeters(20);      
           // ayarla en -- boy
         public static final Translation2d kFrontLeft = new Translation2d(kRobotLenght/2 ,kRobotWidth /2);
         public static final Translation2d kFrontRight = new Translation2d(kRobotLenght / 2, -kRobotWidth/2);
         public static final Translation2d kRearLeft = new Translation2d(-kRobotLenght/2, kRobotWidth/2);
         public static final Translation2d kRearRight = new Translation2d(-kRobotLenght/2, -kRobotWidth/2);

         public static final SwerveDriveKinematics swerveDriveKinematics =  new SwerveDriveKinematics(kFrontLeft,kFrontRight,kRearLeft,kRearRight);


         
        
      }

      public static class JoystickCon{
        public static final int kDriverJoystickPort = 0;
        public static final int kSecondJoystickPort = 1;
        public static final int LX = 0;
        public static final int LY = 1;
        public static final int RX= 4;
        public static final int RY = 5;
        
        



       }


  }

