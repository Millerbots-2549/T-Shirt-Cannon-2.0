package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

public class Robot extends TimedRobot {

  // Drive motorss
  private final WPI_TalonSRX m_LeftFrontMotor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_LeftBackMotor = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_RightFrontMotor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_RightBackMotor = new WPI_TalonSRX(4);

  @SuppressWarnings("removal")
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_LeftFrontMotor, m_LeftBackMotor);
  @SuppressWarnings("removal")
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_RightFrontMotor, m_RightBackMotor);

  private final DifferentialDrive m_RobotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Pneumatics
  private final Solenoid m_Solenoid = new Solenoid(5, PneumaticsModuleType.REVPH, 0);
  private final Compressor m_Compressor = new Compressor(5, PneumaticsModuleType.REVPH);

  // Turret
  private final SparkMax m_TurretMotor = new SparkMax(8, MotorType.kBrushless);
  private final TrapezoidProfile.Constraints m_turrretProfile = 
  new TrapezoidProfile.Constraints(100000, 30);
  ProfiledPIDController m_TurretPID = new ProfiledPIDController(0.06, 0.0, 0.0, m_turrretProfile);
  
  

  // PDP
  private final PowerDistribution m_PDP = new PowerDistribution(0, ModuleType.kCTRE);

  // Controller
  private final XboxController m_DriverController = new XboxController(0);

  @SuppressWarnings("removal")
  public Robot() {
    // PID
    //m_TurretPID.setTolerance(2, 100);
    //m_TurretPID.enableContinuousInput(0, 360);
    
    
    // Start camera feed
    CameraServer.startAutomaticCapture();
    
    m_TurretMotor.getEncoder().setPosition(0);
    
    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    
    tab.addDouble("Compressor Current", () -> m_Compressor.getCurrent());
    tab.addDouble("Turret Encoder", () -> m_TurretMotor.getEncoder().getPosition());
    tab.addDouble("Turret Pose", () -> (m_TurretMotor.getEncoder().getPosition() * 360) / 70.56);
    tab.addDouble("Power Consumption", () -> m_PDP.getTotalPower());
    tab.addBoolean("Compressor Running", () -> m_Compressor.isEnabled());
    tab.addBoolean("Compressor Pressure Switch", () -> m_Compressor.getPressureSwitchValue());
  

    
    
    

    // Add motors to registry
    SendableRegistry.addChild(m_RobotDrive, m_leftMotors);
    SendableRegistry.addChild(m_RobotDrive, m_rightMotors);

    // Invert right side motors
    m_rightMotors.setInverted(true);
  }

  @Override 
  public void teleopPeriodic() {
    m_RobotDrive.arcadeDrive(-m_DriverController.getLeftY(), -m_DriverController.getRightX());

    m_Solenoid.set(m_DriverController.getAButton());
    
    double turretSpeed = m_DriverController.getLeftTriggerAxis() - m_DriverController.getRightTriggerAxis();
    //double turretSpeed = m_TurretPID.calculate(m_TurretMotor.getEncoder().getPosition()/84.7777777777, 0.5);
    double TurretAngle = ((m_TurretMotor.getEncoder().getPosition()*360)/70.56);
    //double turretSpeed = m_TurretPID.calculate(TurretAngle, 180);
    
    

    m_TurretMotor.set(turretSpeed);
    
    
  }
}
