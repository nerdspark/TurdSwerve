package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class Concert {

        final Timer musicTimer = new Timer();
        double timePlay = 15.0;

    public void playMusic() {
        musicTimer.start();

        // for (int i; i <  2 ; i++){
        //     double timeStart = musicTimer.get();
        // }
        
        // Create an Orchestra object with the instruments
        Orchestra m_orchestra = new Orchestra();
        
        // Add instruments to the orchestra
        m_orchestra.addInstrument(new TalonFX(13)); // Add TalonFX with device ID 1
        m_orchestra.addInstrument(new TalonFX(14)); // Add TalonFX with device ID 2
        m_orchestra.addInstrument(new TalonFX(17)); // Add TalonFX with device ID 2
        m_orchestra.addInstrument(new TalonFX(18)); // Add TalonFX with device ID 2


        // Load the Chirp file
        m_orchestra.loadMusic("industryBaby.chrp");

        // Play the music
        if (musicTimer.get() > timePlay){
            m_orchestra.stop();
        }else{
            m_orchestra.play();
        }

    }
    
}