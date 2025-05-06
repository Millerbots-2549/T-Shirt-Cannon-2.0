package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Queue;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Robot extends TimedRobot {
  // drive motors
  private final WPI_TalonSRX m_Left_Front_Motor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_Left_Back_Motor = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_Right_Front_Motor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_Right_Back_Motor = new WPI_TalonSRX(5);

  //shooter
  private final Solenoid m_solenoid = new Solenoid(5, PneumaticsModuleType.REVPH, 1);
  private final Compressor m_compressor = new Compressor(5,PneumaticsModuleType.REVPH);

  // turret
  private final TalonFX Turret_Motor = new TalonFX(6);

  //PDP
  PowerDistribution m_PDP = new PowerDistribution(0, ModuleType.kCTRE);
  
  
  // drive motor groups
  MotorControllerGroup leftMotors = new MotorControllerGroup(m_Left_Front_Motor, m_Left_Back_Motor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(m_Right_Front_Motor, m_Right_Back_Motor);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final XboxController m_driverController = new XboxController(0);

  // Called once at the beginning of the robot program.
  public Robot() {
    ShuffleboardTab tab = Shuffleboard.getTab("Main");

    //power
    tab.addDouble("Power Consomtion", m_PDP::getTotalCurrent);
    
    //compressor
    tab.addDouble("Compressor Current", m_compressor::getCurrent);
    tab.addBoolean("Compressor Active", m_compressor::isEnabled);
    tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);
    
    
    
    
    SendableRegistry.addChild(m_robotDrive, leftMotors);
    SendableRegistry.addChild(m_robotDrive, rightMotors);

    // change to left?
    rightMotors.setInverted(true);
    

    // turret motor pid
    /*var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    Turret_Motor.getConfigurator().apply(slot0Configs);*/

    
  }

  @Override
  public void teleopPeriodic() {
  
    m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
    m_solenoid.set(m_driverController.getXButton());
    
    Turret_Motor.set(m_driverController.getLeftTriggerAxis()/2); // fix
    Turret_Motor.set(-m_driverController.getLeftTriggerAxis()/2); // fix
/*
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    Turret_Motor.setControl(m_request.withPosition(10.0)); */


    




  }
}
