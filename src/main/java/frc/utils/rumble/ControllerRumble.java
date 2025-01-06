package frc.utils.rumble;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

public class ControllerRumble {
    private XboxController controller;
    List<RumbleBase> rumbles = new ArrayList<>();
    public ControllerRumble(XboxController controller){
        this.controller = controller;
    }
    public void addRumble(double time, double power, RumbleType type){
        switch (type) {
            case OVERRIDE:
                rumbles.clear();
                rumbles.add(new Rumble(time, power));    
            break;
            case ADD:
                rumbles.add(new Rumble(time, power));
            break;
            case OVERLAY:
            if(rumbles.size() > 0){
                RumbleBase r = rumbles.get(0);
                double t = time;
                while(t>0) {
                    if(t <= r.getTime()){
                        r.subtractTime(t);
                        t = 0;
                    } else {
                        double temp = r.getTime();
                        r.setTime(0);
                        t -= temp;
                    }
                    if(rumbles.get(0).getTime() <= 0){
                        rumbles.remove(0);
                    }
                    
                    if(rumbles.size() > 0){
                        r = rumbles.get(0);
                    } else {
                        break;
                    }
                }
            }
            rumbles.add(0, new Rumble(time, power));
            break;
        
            default:
                break;
        }
    }
    
    public void addRumble(RumbleSequence rumble, RumbleType type){
        switch (type) {
            case OVERRIDE:
                rumbles.clear();
                rumbles.add(rumble);    
            break;
            case ADD:
                rumbles.add(rumble);
            break;
            case OVERLAY:
            if(rumbles.size() > 0){
                RumbleBase r = rumbles.get(0);
                double t = r.getTime();
                while(t>0) {
                    if(t <= r.getTime()){
                        r.subtractTime(t);
                        t = r.getTime();
                    } else {
                        double temp = r.getTime();
                        r.setTime(0);
                        t -= temp;
                    }
                    if(rumbles.size() > 0){
                        if(rumbles.get(0).getTime() <= 0){
                            rumbles.remove(0);
                        }
                    }
                    if(rumbles.size() > 0){
                        r = rumbles.get(0);
                    } else {
                        break;
                    }
                }
            }
            rumbles.add(0, rumble);
        
            default:
                break;
        }
    }
    public void clearRumble(){
        rumbles.clear();
        controller.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
    public void update(){
        if(rumbles.size() > 0){
            rumbles.get(0).subtractTime(0.02);
            rumbles.get(0).update();
        }
        timelessUpdate();
        
    }
    //update without changing time values
    public void timelessUpdate(){
        if(rumbles.size() > 0){
            if(rumbles.get(0).getTime() <= 0){
                rumbles.remove(0);
            }
        }
        if(rumbles.size() > 0){
            controller.setRumble(GenericHID.RumbleType.kBothRumble, rumbles.get(0).getStrength());
        } else {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}
