package frc.utils.rumble;

import java.util.ArrayList;
import java.util.Collection;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumbleHandler {
    private XboxController controller;
    private double port;

    private boolean following = false;
    private RumbleHandler lead;

    private ArrayList<Rumble> que = new ArrayList<Rumble>();
    public RumbleHandler(XboxController controller){
        this.controller = controller;
        this.port = controller.getPort();
    }
    public RumbleHandler(RumbleHandler lead){
        this.following = true;
        this.lead = lead;
    }
    /**
     * clear rumble que, effectivly stopping all rumble
     */
    public void clearQue(){
        que.clear();
    }
    public void addToQue(Rumble[] a){
        for(Rumble b : a){
            que.add(b);
        }
    }
    public void addToQue(Collection<Rumble> a){
        for(Rumble b : a){
            que.add(b);
        }
    }
    public void addToQue(Rumble a){
        que.add(a);
    }

    public void overrideQue(Rumble a){
        clearQue();
        que.add(a);
    }
    public void overrideQue(Collection<Rumble> a){
        clearQue();
        for(Rumble b : a){
            que.add(b);
        }
    }
    public void overrideQue(Rumble[] a){
        clearQue();
        for(Rumble b : a){
            que.add(b);
        }
    }
    public void update(double loopTime){
        if(following){
            que = lead.que;
        }
        for (int i=0; i < que.size(); i++) {
            que.get(i).time -= loopTime;
            if(que.get(i).time <= 0){
                que.remove(i);
            }
        }
        if(que.size() >= 1){
            controller.setRumble(RumbleType.kBothRumble, que.get(0).pow);
            Logger.recordOutput("haptics/rumble: "+port+"/currentStrength", que.get(0).pow);
            Logger.recordOutput("haptics/rumble: "+port+"/que", getPows());
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
            Logger.recordOutput("haptics/rumble: "+port+"/currentStrength", 0.0);
            Logger.recordOutput("haptics/rumble: "+port+"/que", new double[0][0]);
        }
        Logger.recordOutput("haptics/rumble: "+port+"/following", following);
        
        if(following){
            Logger.recordOutput("haptics/rumble: "+port+"/following rumble", lead.port);
        }
    } 
    private double[][] getPows(){
        double[][] pows = new double[que.size()][2];
        for(int i=0; i < que.size(); i++){
            Rumble d = que.get(i);
            pows[i][0] = d.pow;
            pows[i][1] = d.time;
        }
        return pows;
    }
}