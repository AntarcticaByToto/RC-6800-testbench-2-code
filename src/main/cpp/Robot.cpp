/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/AnalogPotentiometer.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>

#include <frc/PWMVictorSPX.h>
#include <rev/CANSparkMax.h>
#include <frc/Talon.h>

#include <frc/SpeedControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include <frc/livewindow/LiveWindow.h>

#include <wpi/StringMap.h>




#define CAN_ID_LEFT_A 1
#define CAN_ID_LEFT_B 2
#define CAN_ID_LEFT_C 3  //3
//sketchy

#define CAN_ID_RIGHT_A 4 //4
#define CAN_ID_RIGHT_B 5 //5
#define CAN_ID_RIGHT_C 6

#define MAX_OUTPUT 1.0
#define DEADBAND 0.05

int setDrirection = 1;
int Direction = 1;

int shooterRunning = 0;
int hopperRunning = 0;
int driverunning = 0;
/**
 * This sample program provides an example for ShuffleBoard, an alternative
 * to SmartDashboard for displaying values and properties of different robot
 * parts.
 *
 * ShuffleBoard can use pre-programmed widgets to display various values, such
 * as Boolean Boxes, Sliders, Graphs, and more. In addition, they can display
 * things in various Tabs.
 *
 * For more information on how to create personal layouts and more in
 * ShuffleBoard, feel free to reference the official FIRST WPILib documentation
 * online.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Add a widget titled 'Max Speed' with a number slider.


    // wpi::StringMap properties = {
    //   std::make_pair("min", nt::Value::MakeDouble(0)),
    //   std::make_pair("max", nt::Value::MakeDouble(1))
    // };


    // wpi::StringMap a {};
    //   auto entry = frc::Shuffleboard::GetTab("Drive")
    //   .Add("Max Speed", 1)
    //   .WithWidget("Number Slider")
    //   .WithProperties(properties)
    //   .GetEntry();




    m_maxSpeedShooter = frc::Shuffleboard::GetTab("Configuration")
                     .Add("Max Speed S", 1)
                     .WithWidget("Number Slider")
                     .GetEntry();
    
    m_maxSpeedT = frc::Shuffleboard::GetTab("Configuration")
                     .Add("Max Speed T", 1)
                     .WithWidget("Number Slider")
                     .GetEntry();

    m_maxSpeedHopper = frc::Shuffleboard::GetTab("Configuration")
                     .Add("Max Speed H", 1)
                     .WithWidget("Number Slider")
                     .GetEntry();

    m_maxSpeedI = frc::Shuffleboard::GetTab("Configuration")
                     .Add("Max Speed I", 1)
                     .WithWidget("Number Slider")
                     .GetEntry();

    m_maxSpeedD = frc::Shuffleboard::GetTab("Configuration")
                     .Add("Max speed D", 1)
                     .WithWidget("Number Slider")
                     .GetEntry();


    m_robotDrive.SetMaxOutput(MAX_OUTPUT);
    m_robotDrive.SetDeadband(DEADBAND);

    // m_drive_l_A.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // m_drive_l_B.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // m_drive_l_C.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    
    // m_drive_r_A.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // m_drive_r_B.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // m_drive_r_C.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_drive_l_A.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_drive_l_B.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_drive_l_C.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_drive_r_A.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_drive_r_B.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_drive_r_C.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_drive_r_A.SetInverted(!m_drive_r_A.GetInverted());
    m_drive_r_B.SetInverted(!m_drive_r_B.GetInverted());
    m_drive_r_C.SetInverted(!m_drive_r_C.GetInverted());

    // Create a 'DriveBase' tab and add the drivetrain object to it.
   frc::ShuffleboardTab& driveBaseTab = frc::Shuffleboard::GetTab("DriveBase");
    driveBaseTab.Add("TankDrive", m_robotDrive);

   
   }

  void TeleopPeriodic(){

    if(m_stickO.GetRawButtonPressed(8))
    {
      if (shooterRunning == 0)
      {
        shooterRunning = 1;
      }
    }
    else if (m_stickO.GetRawButtonPressed(7))
    {
      if (shooterRunning != 0)
      {
        shooterRunning = 0;
      }
    }



    if(m_stickO.GetRawButtonPressed(4))
    {
      if (shooterRunning == 0)
      {
        shooterRunning = 1;
      }
    }
    else if (m_stickO.GetRawButtonPressed(7))
    {
      if (shooterRunning != 0)
      {
        shooterRunning = 0;
      }
    }
    


    //  if(m_stickO.GetRawButtonPressed(1))
    // {
    //   if (driverunning == 0)
    //   {
    //     driverunning = 1;
    //   }
    // }
    // else if (m_stickO.GetRawButtonPressed(7))
    // {
    //   if (driverunning != 0)
    //   {
    //     driverunning = 0;
    //   }
    // }

    double maxRatioS = m_maxSpeedShooter.GetDouble(1.0);
    double maxRatioT = m_maxSpeedT.GetDouble(1.0);
    double maxRatioH = m_maxSpeedHopper.GetDouble(1.0);
    double maxRatioI = m_maxSpeedI.GetDouble(1.0);
    double maxRatioD = m_maxSpeedD.GetDouble(1.0);
    
    //double rightStick = m_stickD.GetX(frc::GenericHID::JoystickHand::kRightHand);    //Testing
    double rightStick = m_stickD.GetX(frc::GenericHID::JoystickHand::kRightHand);    //Testing
    double leftStick = m_stickD.GetY(frc::GenericHID::JoystickHand::kLeftHand);


    //Drive
    m_robotDrive.ArcadeDrive(leftStick * -1, rightStick * 0.5, true);
    //m_robotDrive.TankDrive(leftStick * -1,leftStick * -1);

    double appliedOutputLA = m_drive_l_A.GetAppliedOutput();
    double appliedOutputLB = m_drive_l_B.GetAppliedOutput();
    double appliedOutputLC = m_drive_l_C.GetAppliedOutput();

    double appliedOutputRA = m_drive_r_A.GetAppliedOutput();
    double appliedOutputRB = m_drive_r_B.GetAppliedOutput();
    double appliedOutputRC = m_drive_r_C.GetAppliedOutput();

    // if (driverunning == 1)
    //  {
    //    m_robotDrive.TankDrive(maxRatioD ,maxRatioD);   //Testing
    //  }
    //  else
    //  {
    //   m_robotDrive.TankDrive(0,0);
    //  }
    
    

      //Shooter
     if (shooterRunning == 1)
     {
       m_shooter.Set(std::clamp( maxRatioS * Direction, -1.0, 1.0));   //Testing
     }
     else
     {
      m_shooter.Set(0);
     }

    
    //Hopper
    if (m_stickO.GetRawButton(6))
     {
       m_Hopper.Set(std::clamp( maxRatioH * Direction, -1.0, 1.0));   //Testing
     }
     else if (m_stickO.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand))
     {
       m_Hopper.Set(std::clamp( maxRatioH * -1 * Direction, -1.0 , 1.0));
     }
      else
     {
       m_Hopper.Set(0);
     }

    //Intake
    if (m_stickO.GetRawButton(5))
     {
       m_zero.Set(std::clamp( maxRatioI * Direction, -1.0, 1.0));   //Testing
     }
     else if (m_stickO.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand))
     {
       m_zero.Set(std::clamp( maxRatioI * -1 * Direction, -1.0 , 1.0));
     }
      else
     {
       m_zero.Set(0);
     }
    

    
    frc::SmartDashboard::PutNumber("Right Stick" , rightStick);


    frc::SmartDashboard::PutNumber("Left Applied A" , appliedOutputLA);
    frc::SmartDashboard::PutNumber("Left Applied B" , appliedOutputLB);
    frc::SmartDashboard::PutNumber("Left Applied C" , appliedOutputLC);

    frc::SmartDashboard::PutNumber("Right Applied A" , appliedOutputLA);
    frc::SmartDashboard::PutNumber("Right Applied B" , appliedOutputLB);
    frc::SmartDashboard::PutNumber("Right Applied C" , appliedOutputLC);
  

    // frc::SmartDashboard::PutBoolean("button 1", m_stick.GetRawButton(1));
    // frc::SmartDashboard::PutBoolean("button 2", m_stick.GetRawButton(2));
    // frc::SmartDashboard::PutBoolean("button 3", m_stick.GetRawButton(3));
    // frc::SmartDashboard::PutBoolean("button 4", m_stick.GetRawButton(4));
   }

  void AutonomousInit() override {
    // Update the Max Output for the drivetrain.
   // m_robotDrive.SetMaxOutput(m_maxSpeed.GetDouble(1.0));
  }

  
 private:
  frc::PWMVictorSPX m_zero{0}; //S
  frc::PWMVictorSPX m_one{1}; //S
  frc::PWMVictorSPX m_two{2};  //T

  frc::SpeedControllerGroup m_shooter{m_zero, m_one, m_two};

  frc::PWMVictorSPX m_three{3};  //H
  frc::PWMVictorSPX m_four{4}; //H
  frc::PWMVictorSPX m_five{5};  //H

  frc::SpeedControllerGroup m_Hopper{m_three, m_four, m_five};

  //i
  

  //frc::Talon m_eight{6};
  //frc::Talon m_nine{7};


  // drive
  rev::CANSparkMax m_drive_l_A { CAN_ID_LEFT_A ,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_l_B { CAN_ID_LEFT_B ,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_l_C { CAN_ID_LEFT_C ,  rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_drive_r_A { CAN_ID_RIGHT_A ,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_r_B { CAN_ID_RIGHT_B ,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_drive_r_C { CAN_ID_RIGHT_C ,  rev::CANSparkMax::MotorType::kBrushless};

  frc::SpeedControllerGroup m_drive_l{m_drive_l_A, m_drive_l_B , m_drive_l_C};
  frc::SpeedControllerGroup m_drive_r{m_drive_r_A, m_drive_r_B, m_drive_r_C};

  frc::DifferentialDrive m_robotDrive{m_drive_l, m_drive_r};
  
  

  frc::XboxController m_stickD{0};
  frc::XboxController m_stickO{1};


  nt::NetworkTableEntry m_maxSpeedShooter;
  nt::NetworkTableEntry m_maxSpeedT;
  nt::NetworkTableEntry m_maxSpeedHopper;
  nt::NetworkTableEntry m_maxSpeedI;

  nt::NetworkTableEntry m_maxSpeedD;

};


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
