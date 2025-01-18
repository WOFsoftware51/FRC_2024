package frc.robot.field;


import edu.wpi.first.math.geometry.Pose3d;

public class AprilTag {

    private int id;
    private Pose3d position; 

    public AprilTag(int _id, Pose3d _position){
        id = _id;
        position = _position;
    }
    
    public int getId(){
        return id;
    }
    public Pose3d getPosition(){
        return position;
    }
}
