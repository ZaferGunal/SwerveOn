// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.commands.SwerveJoystickCmd;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.JoystickCon;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  public SwerveSystem swerveSystem = new SwerveSystem();
  public Joystick driverJoystick = new Joystick(JoystickCon.kDriverJoystickPort);
  
  public RobotContainer() {
    swerveSystem.setDefaultCommand(new SwerveJoystickCmd(swerveSystem,
    () -> -driverJoystick.getRawAxis(JoystickCon.LY),
    () -> driverJoystick.getRawAxis(JoystickCon.LX),                     // karışıklığa karşı değiştir
    () -> driverJoystick.getRawAxis(JoystickCon.RX)
    ));
    
    configureBindings();
  }


  private void configureBindings() {
   
  }

  public Command getAutonomousCommand() {
  
    return  null;
  }

}
