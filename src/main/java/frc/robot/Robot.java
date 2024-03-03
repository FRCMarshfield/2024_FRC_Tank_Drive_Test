// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

//added
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick; //driver controller
  //private Joystick m_rightStick; //arm controller
  private static final int leftFrontDeviceID = 4;
  private static final int leftRearDeviceID = 3;
  private static final int rightFrontDeviceID = 1;
  private static final int rightRearDeviceID = 2;
  private static final int armPivotLeftID = 5;
  private static final int armPivotRightID = 6;
  private static final int intakeShootBottomID = 7;
  private static final int intakeShootTopID = 8;
  private static final int intake = 9;
  private CANSparkMax m_leftFront;
  private CANSparkMax m_leftRear;
  private CANSparkMax m_rightFront;
  private CANSparkMax m_rightRear;
  private CANSparkMax m_armPivotLeft;
  private CANSparkMax m_armPivotRight;
  private CANSparkMax m_intakeShootBottom;
  private CANSparkMax m_intakeShootTop;
  private CANSparkMax m_intake;
  static final DutyCycleEncoder encoder = new DutyCycleEncoder(0); //pivot encoder

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_leftFront = new CANSparkMax(leftFrontDeviceID, MotorType.kBrushless);
    m_leftRear = new CANSparkMax(leftRearDeviceID, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(rightFrontDeviceID, MotorType.kBrushless);
    m_rightRear = new CANSparkMax(rightRearDeviceID, MotorType.kBrushless);
    m_armPivotLeft = new CANSparkMax(armPivotLeftID, MotorType.kBrushless);
    m_armPivotRight = new CANSparkMax(armPivotRightID, MotorType.kBrushless);
    m_intakeShootBottom = new CANSparkMax(intakeShootBottomID, MotorType.kBrushless);
    m_intakeShootTop = new CANSparkMax(intakeShootTopID, MotorType.kBrushless);
    m_intake = new CANSparkMax(intake, MotorType.kBrushless);
    

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    
    m_leftFront.restoreFactoryDefaults();
    m_leftRear.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    m_rightRear.restoreFactoryDefaults();

    m_leftRear.follow(m_leftFront);
    m_rightRear.follow(m_rightFront);

    m_myRobot = new DifferentialDrive(m_leftFront, m_rightFront);

    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

   //motor vars
   static boolean intakeOn = false;
   static boolean pivotOn = false;
   static boolean shootOn = false;

   //arm motor setter
   public void setArmMotor(double percent){
    if(pivotOn == true){
      m_armPivotLeft.set(-percent);
      m_armPivotRight.set(percent);
    }else{
      m_armPivotLeft.set(0);
      m_armPivotRight.set(0);
    }

    if(intakeOn == true){
      m_intake.set(0.5);
    }else{
      m_intake.set(0);
    }
    
    if(shootOn == true){
      m_intakeShootTop.set(-0.5);
      m_intakeShootBottom.set(0.5);
    }else{
      m_intakeShootTop.set(0);
      m_intakeShootBottom.set(0);
    }
  }

  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //uncommented the above line
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    encoder.reset();
  }

  /** This function is called periodically during operator control. */
  static boolean driveDirection = false; //front switch
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Distance", encoder.getAbsolutePosition()); //read out encoder pos

    //drive flip (weirdly inconsistant)
    if (m_leftStick.getRawButton(5)){
      driveDirection = !driveDirection;
    }
    if(driveDirection == false){
      m_myRobot.arcadeDrive(m_leftStick.getX(), m_leftStick.getY(), true);
    }
    if(driveDirection == true){
      m_myRobot.arcadeDrive(-m_leftStick.getX(), -m_leftStick.getY(), true);
    }
    //potential dual joystick driving
    //m_myRobot.arcadeDrive(m_leftStick.getX(), m_leftStick.getY(), true);
    //m_myRobot.arcadeDrive(-m_leftStick.getRawAxis(1), m_leftStick.getRawAxis(4), true);

    //intake button
    if(m_leftStick.getRawButton(2)){
      intakeOn = true;
      setArmMotor(0);
    }else{
      intakeOn = false;
    }

    //shoot button
    if(m_leftStick.getRawButton(2)){
      intakeOn = true;
      setArmMotor(0);
    }else{
      intakeOn = false;
    }

    //pivot main arm up
    if(m_leftStick.getRawButton(4)){
      pivotOn = true;
      if(encoder.getAbsolutePosition() > 0.8){
        setArmMotor(0.5);
      }else if(encoder.getAbsolutePosition() > 0.7){
        setArmMotor(0.15);
      }else if(encoder.getAbsolutePosition() == 0.7){
        setArmMotor(0);
      }  
    }
    else{
      setArmMotor(0);
      pivotOn = false;
    }

    //pivot main arm down
    if(m_leftStick.getRawButton(1)){
      if(encoder.getAbsolutePosition() < 0.9){
        setArmMotor(-0.5);
      }else if(encoder.getAbsolutePosition() < 0.95){
        setArmMotor(-0.15);
      }else if(encoder.getAbsolutePosition() == 1){
        setArmMotor(0);
      }  
    }
    else{
      setArmMotor(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
