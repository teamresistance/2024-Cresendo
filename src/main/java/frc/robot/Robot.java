/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.test.MecanumDriveCalculator;
import frc.robot.test.NavX;
import frc.robot.test.motorPID;



/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  
  private Joystick m_stick;
  private Joystick m_stick2;

  public static final double maxRpm = 5700;
  
  //public static final double maxRpm = 45.4609;
  
  //Maximum MPS(Meters per second) was derived from using maxRpm and wheel radius. 
  //maxRpm*radiusx2xpi/60
  //Wheel radius is 3 inches
  public static CANSparkMax frontLeftLd = new CANSparkMax(11, MotorType.kBrushless);
  public static CANSparkMax backLeftLd = new CANSparkMax(13, MotorType.kBrushless);
  public static CANSparkMax frontRightLd = new CANSparkMax(15, MotorType.kBrushless);
  public static CANSparkMax backRightLd = new CANSparkMax(17, MotorType.kBrushless);

  public static CANSparkMax frontLeftLg = new CANSparkMax(12, MotorType.kBrushless);
  public static CANSparkMax backLeftLg = new CANSparkMax(14, MotorType.kBrushless);
  public static CANSparkMax frontRightLg = new CANSparkMax(16, MotorType.kBrushless);
  public static CANSparkMax backRightLg = new CANSparkMax(18, MotorType.kBrushless);
  

  public static motorPID frontLeftLdPID = new motorPID(frontLeftLd, frontLeftLg, false, false, frontLeftLd.getPIDController());
  public static motorPID backLeftLdPID = new motorPID(backLeftLd, backLeftLg, false, false, backLeftLd.getPIDController());
  public static motorPID frontRightLdPID = new motorPID(frontRightLd, frontRightLg, true, false, frontRightLd.getPIDController());
  public static motorPID backRightLdPID = new motorPID(backRightLd, backRightLg, true, false, backRightLd.getPIDController());

  public static NavX navX = new NavX(SPI.Port.kMXP);

  @Override
  public void robotInit() {
    m_stick = new Joystick(0);
    m_stick2 = new Joystick(1);
   


    frontLeftLdPID.init();
    backLeftLdPID.init();
    frontRightLdPID.init();
    backRightLdPID.init();
  }

  @Override
  public void teleopPeriodic() {

    frontLeftLdPID.update();
    backLeftLdPID.update();
    frontRightLdPID.update();
    backRightLdPID.update();


    double setPointY = -m_stick.getY();
    double setPointX = -m_stick.getX();
    double setPointZ = m_stick2.getX();
    double gyroAngle = navX.getAngle();

    double[] inputs;

  
    inputs = MecanumDriveCalculator.calculateMecanumDrive(setPointX, setPointY, setPointZ, gyroAngle);

    frontLeftLdPID.updateSetpoint(inputs[0]*=maxRpm);
    frontRightLdPID.updateSetpoint(inputs[1]*=maxRpm);
    backLeftLdPID.updateSetpoint(inputs[2]*=maxRpm);
    backRightLdPID.updateSetpoint(inputs[3]*=maxRpm);


    SmartDashboard.putNumber("SetPointX", setPointX);
    SmartDashboard.putNumber("SetPointY", setPointY);
    SmartDashboard.putNumber("SetPointZ", setPointZ);
    
    SmartDashboard.putNumber("InputFL", inputs[0]);
    SmartDashboard.putNumber("InputFR", inputs[1]);
    SmartDashboard.putNumber("InputBL", inputs[2]);
    SmartDashboard.putNumber("InputBR", inputs[3]);
  }
}