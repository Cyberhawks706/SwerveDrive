package frc.robot.commands;




import edu.wpi.first.wpilibj.DriverStation;

public class Clock {
    
    public double startTime;
    public double elapsedTime;
    double [] ArrayStateLengths;
    Timer706 timer = new Timer706();


    public Clock(double [] arrayStateLengths){ //
        ArrayStateLengths = arrayStateLengths;
        startTime = -999999999;//make it small - no task should take this long
        timer.start();
    }
    public void setStart (){//use this when the climb program is started
        startTime = timer.get();
        DriverStation.reportWarning("START CLIMB NOW", false);
    }

    
    public int getState(){

        int state = 1;
        double currentTime = timer.get();
        elapsedTime = currentTime - startTime;
        double duration = elapsedTime;
        DriverStation.reportWarning("Current Elapsed time: " + elapsedTime, false);
        double test = duration;
        for(int i = 0;i<ArrayStateLengths.length;i++){
            test -= ArrayStateLengths[i];
            if(test<0){
                return state;
            }
            state++;
        }
        return 0;
    }
    
    
}
