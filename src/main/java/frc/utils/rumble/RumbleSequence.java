package frc.utils.rumble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RumbleSequence implements RumbleBase{
    public List<Rumble> rumbles;
    private double duration = 0.0;
    private double currentStrength = 0.0;
    private Rumble currentRumble;

    public RumbleSequence(ArrayList<Rumble> rumbles){
        this.rumbles = rumbles;
        this.duration = 0;
        for(Rumble r : rumbles){
            this.duration += r.getTime();
        }
    }
    public RumbleSequence(Rumble[] rumbles){
        this.rumbles = new ArrayList<>(Arrays.asList(rumbles));
        this.duration = 0;
        for(Rumble r : rumbles){
            this.duration += r.getTime();
        }
    }
    @Override
    public void subtractTime(double time) {
        if(rumbles.size() > 0){
            Rumble r = rumbles.get(0);
            double t = time;
            while(t>0) {
                if(t <= r.getTime()){
                    r.subtractTime(t);
                    t = 0.0;
                } else {
                    double temp = r.getTime();
                    r.setTime(0);
                    t -= temp;
                }
                if(r.getTime() <= 0){
                    rumbles.remove(0);
                }
                
                if(rumbles.size() > 0){
                    r = rumbles.get(0);
                } else {
                    break;
                }
            }
        }
        update();
    }
    @Override
    public double getTime() {
        return this.duration;
    }
    @Override
    public void setTime(double time) {
        this.duration = time;
    }
    @Override
    public double getStrength() {
        return currentStrength;
    }
    @Override
    public void update(){
        if(rumbles.size() > 0){
            if(rumbles.get(0).getTime() <= 0){
                rumbles.remove(0);
            }
        }
        if(rumbles.size() > 0){
            currentRumble = rumbles.get(0);
            duration = 0;
            for(Rumble r : rumbles){
                duration += r.getTime();
            }
            currentStrength = currentRumble.getStrength();
        } else {
            currentStrength = 0;
        }
    }
}