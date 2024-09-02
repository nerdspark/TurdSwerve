// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Orchestra m_orchestra;
  TalonFX [] m_Fxes = {new TalonFX(13), new TalonFX(14), new TalonFX(17), new TalonFX(18)};

  String [] m_songs = new String[]{
    //   "intergalactic.chrp",
    //   "industryBaby.chrp",
    //   "backToTheFuture.chrp",
    //   "bohemianRhapsody.chrp",
    //   "carelessWhisper.chrp",
    //   "neverGonnaGiveYouUp.chrp",
    //   "piratesCaribbean.chrp",
    //   "stillDRE.chrp",
      "swanLake.chrp"
  };

  /* track which song is selected for play */
  int m_songSelection = 0;

  /* overlapped actions */
  int m_timeToPlayLoops = 0;

  /* joystick vars */
  Joystick m_joy;
  int m_lastButton = 0;
  int m_lastPOV = 0;

  //------------- joystick routines --------------- //
  /** @return 0 if no button pressed, index of button otherwise. */
  int getButton() {
      for (int i = 1; i < 9; ++i) {
          if (m_joy.getRawButton(i)) {
              return i;
          }
      }
      return 0;
  }

  void LoadMusicSelection(int offset)
  {
      /* increment song selection */
      m_songSelection += offset;
      /* wrap song index in case it exceeds boundary */
      if (m_songSelection >= m_songs.length) {
          m_songSelection = 0;
      }
      if (m_songSelection < 0) {
          m_songSelection = m_songs.length - 1;
      }
      /* load the chirp file */
      m_orchestra.loadMusic(m_songs[m_songSelection]); 

      /* print to console */
      System.out.println("Song selected is: " + m_songs[m_songSelection] + ".  Press left/right on d-pad to change.");
      
      /* schedule a play request, after a delay.  
          This gives the Orchestra service time to parse chirp file.
          If play() is called immedietely after, you may get an invalid action error code. */
      m_timeToPlayLoops = 10;
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

            /* A list of TalonFX's that are to be used as instruments */
        ArrayList<TalonFX> m_instruments = new ArrayList<TalonFX>();

        /* Create the orchestra with the TalonFX instruments */
        m_orchestra = new Orchestra();
      
        /* Initialize the TalonFX's to be used */
        for (int i = 0; i < m_Fxes.length; ++i) {
            m_instruments.add(m_Fxes[i]);
            m_orchestra.addInstrument(m_Fxes[i]);
        }

        
        m_joy = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    SignalLogger.setPath("/media/sda1/");
    SignalLogger.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    SignalLogger.stop();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // SignalLogger.setPath("/media/sda1/");
    // SignalLogger.start();

    /* load whatever file is selected */
    LoadMusicSelection(0);
  }

  @Override
  public void teleopPeriodic() {
      /* poll gamepad */
      int btn = getButton();
      int currentPOV = m_joy.getPOV();

      /* if song selection changed, auto-play it */
      if (m_timeToPlayLoops > 0) {
          --m_timeToPlayLoops;
          if (m_timeToPlayLoops == 0) {
              /* scheduled play request */
              System.out.println("Auto-playing song.");
              m_orchestra.play();
          }
      }


      /* has a button been pressed? */
      if (m_lastButton != btn) {
          m_lastButton = btn;

          switch (btn) {
              case 1: /* toggle play and paused */
                  if (m_orchestra.isPlaying()) {
                      m_orchestra.pause();
                      System.out.println("Song paused");
                  }  else {
                      m_orchestra.play();
                      System.out.println("Playing song...");
                  }
                  break;
                  
              case 2: /* toggle play and stop */
                  if (m_orchestra.isPlaying()) {
                      m_orchestra.stop();
                      System.out.println("Song stopped.");
                  }  else {
                      m_orchestra.play();
                      System.out.println("Playing song...");
                  }
                  break;
          }
      }

      /* has POV/D-pad changed? */
      if (m_lastPOV != currentPOV) {
          m_lastPOV = currentPOV;

          switch (currentPOV) {
              case 90:
                  /* increment song selection */
                  LoadMusicSelection(+1);
                  break;
              case 270:
                  /* decrement song selection */
                  LoadMusicSelection(-1);
                  break;
          }
      }
  }

  @Override
  public void teleopExit() {
    // SignalLogger.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
