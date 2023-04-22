package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

public class Timer706 {
    private Timer timer = new Timer();
    public double pausedTime = 0;
    public double pausedStart = 0;
    public boolean isPaused = false;

    public Timer706(){

    }

    public double get(){
        return timer.get()-pausedTime;
    }

    public void reset(){
        timer.reset();
    }

    public void start(){
        if(isPaused){
            pausedTime+=timer.get()-pausedStart;
        }else{
            timer.start();
        }
    }

    public void stop(){
        timer.stop();
    }

    public void pause(){
        if(!isPaused){
            pausedStart = timer.get();
            isPaused = true;
        }
    }
}

