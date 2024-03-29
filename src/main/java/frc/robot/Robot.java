// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// the above isnt working


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
  private Joystick m_Driver; //driver controller
  private Joystick m_Arm; //arm controller
  private static final int leftFrontDeviceID = 4;
  private static final int leftRearDeviceID = 2;
  private static final int rightFrontDeviceID = 1;
  private static final int rightRearDeviceID = 3;
  private static final int armPivotLeftID = 5;
  private static final int armPivotRightID = 6;
  private static final int intakeShootBottomID = 8;
  private static final int intakeShootTopID = 7;
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
  DigitalInput laser = new DigitalInput(4);
  static final double kP = 0;
  static final double kI = 0;
  static final double kD = 0;
  PIDController pid = new PIDController(kP, kI, kD);
  //DigitalInput laser2 = new DigitalInput();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Left", kDefaultAuto);
    m_chooser.addOption("Right", kCustomAuto);
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

    m_Driver = new Joystick(0);
    m_Arm = new Joystick(1);
    m_Driver.setXChannel(4);
    m_Driver.setYChannel(1);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

   //motor vars
   static boolean intakeOn = false; //false = not active
   static boolean pivotOn = false; //false = not active
   static boolean shootOn = false; //false = not active
   static double shooterSpeed = 0.5; //main shoot speed 39
   static boolean intakeMode = false; //false = sucky uppy, true = shoooooooot
   static boolean ampShoot = false; //false = regular, true = shoot the amp
   static boolean ampMode = false; //false = regular, true = amp feed mode
   static boolean driveDirection = false; //front switch var
   static int pivotDirection = 2; //0 down, 1 up, 2 nothing

   //arm motor setter
   public void setArmMotor(double percent){
    m_armPivotLeft.set(-percent);
    m_armPivotRight.set(percent);

    if(intakeOn == true){
      if(intakeMode == false){ //sucky uppy
        m_intake.set(0.2);
      }else if(intakeMode == true){ //shoot mode
        m_intake.set(1);
      }
    }else{
      m_intake.set(0);
    }
    
    if(ampShoot == true){ //amp shoot mode
      m_intakeShootTop.set(0.2);
      m_intakeShootBottom.set(0.2);
      m_intake.set(0.2);
    }else{
      m_intakeShootTop.set(0);
      m_intakeShootBottom.set(0);
    }

    if(shootOn == true){
      m_intakeShootTop.set(shooterSpeed);
      m_intakeShootBottom.set(shooterSpeed);
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
  double autoTimeStart;
  double timeRun;
  double leftSpeed;
  double rightSpeed;
  boolean left = true; //true = left, false = right
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimeStart = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    timeRun = Timer.getFPGATimestamp() - autoTimeStart;
    m_myRobot.tankDrive(leftSpeed, rightSpeed);

    if(timeRun < 3){
      if(encoder.getAbsolutePosition() < 0.97){
        setArmMotor(-0.30);
      }else if(encoder.getAbsolutePosition() < 0.99){
        setArmMotor(-0.15);
      }else if(encoder.getAbsolutePosition() >= 0.99){
        setArmMotor(0);
      }  
    }else if(timeRun < 4){ //shoot the note from start position
      setArmMotor(0);
      shootOn = true;
    }
    
    if(timeRun > 5){
      if(timeRun < 6){
        intakeMode = false;
        intakeOn = false;
        shootOn = false;
        setArmMotor(0);
      }
    }else if(timeRun > 4){
      intakeMode = true;
        intakeOn = true;
      setArmMotor(0);
    }

    if(left == true){
      if(timeRun > 13){
        intakeMode =false;
        intakeOn = false;
        shootOn = false;
        setArmMotor(0);
      }else if(timeRun > 12){
        intakeMode = true;
        intakeOn = true;
        setArmMotor(0);
      }else if(timeRun > 10){
        leftSpeed = 0;
        rightSpeed = 0;
        shootOn = true;
        setArmMotor(0);
      }else if(timeRun > 8){
        intakeOn = false;
        leftSpeed = -0.37;
        rightSpeed = 0.37;
        setArmMotor(0);
      }else if(timeRun > 6){
        intakeOn = laser.get();
        setArmMotor(0);
        leftSpeed = 0.35;
        rightSpeed = -0.35;
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    encoder.reset();
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Pivot Read Out", encoder.getAbsolutePosition()); //read out encoder pos
    SmartDashboard.putNumber("PivotDirection", pivotDirection);
    SmartDashboard.putBoolean("Laser", laser.get());

    //drive flip
    if (m_Driver.getRawButton(1)){
      driveDirection = !driveDirection;
    }
    if(driveDirection == false){
      m_myRobot.arcadeDrive(m_Driver.getX()*75, m_Driver.getY(), true);
    }
    if(driveDirection == true){
      m_myRobot.arcadeDrive(-m_Driver.getX()*75, -m_Driver.getY(), true);
    }

    //arm manual direction set
    if(m_Arm.getY() > 0.06 ){
      pivotDirection = 0;
    }else if(m_Arm.getY() < -0.06){
      pivotDirection = 1;
    }else{
      pivotDirection = 2;
    }

    //amp shoot
    if(m_Arm.getRawButton(2)){
      ampShoot = true;
    }else{
      ampShoot = false;
    }

    //intake note
    if(m_Arm.getRawButton(1)){
      if(laser.get()){ //&& laser2.get()){
          intakeOn = false;
      }else{
        intakeOn = true;
      }
    }else{
      intakeOn = false;
    }

    //shoot setpoint (not finished yet)
    if(m_Arm.getRawButton(4)){
      if(encoder.get() > 0.9){
        if(encoder.get() > 0.95){
          setArmMotor(0.3);
        }else{
          setArmMotor(0.05);
        }
      }
      if(encoder.get() < 0.9){
        if(encoder.get() < 0.85){
          setArmMotor(0.3);
        }else{
          setArmMotor(0.05);
        }
      }
    }

    //shoot spinup
    if(m_Arm.getRawButton(5)){
      shootOn = true;
    }else{
      shootOn = false;
    }

    //shoot the note
    if(m_Arm.getRawButton(6)){
      if(shootOn == true){
        intakeMode = true;
        intakeOn = true;
      }  
    }else{
      intakeMode = false;
    }

    //manual arm control
    if(pivotDirection == 0){
      if(encoder.getAbsolutePosition() > 0.79){
        setArmMotor(m_Arm.getY()*0.5);
      }else if(encoder.getAbsolutePosition() > 0.74){
        setArmMotor(0.15);
      }else if(encoder.getAbsolutePosition() <= 0.74){
        setArmMotor(0);
      }  
    }else if(pivotDirection == 1){
      if(encoder.getAbsolutePosition() < 0.96){
        setArmMotor(m_Arm.getY()*0.5);
      }else if(encoder.getAbsolutePosition() < 0.98){
        setArmMotor(-0.15);
      }else if(encoder.getAbsolutePosition() >= 0.98){
        setArmMotor(0);
      }  
    }else{
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
