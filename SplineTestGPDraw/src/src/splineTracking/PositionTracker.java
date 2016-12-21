package src.splineTracking;

public class PositionTracker {
	
	private double xLeftPos = 0;
	private double yLeftPos = 0;
	private double xRightPos = 0;
	private double yRightPos = 0;
	private double lastLeftDistance = 0;
	private double lastRightDistance = 0;


	public PositionTracker(){
	}
	
	public void setPosistions(double newAngle, double distance){
		double angle = newAngle;
		double leftDistance = distance - lastLeftDistance;
		lastLeftDistance = distance;
		
		double rightDistance = distance - lastRightDistance;
		lastRightDistance = distance;
		
		xLeftPos += Math.sin(Math.toRadians(angle)) * leftDistance;
		yLeftPos += Math.cos(Math.toRadians(angle)) * leftDistance;
		
		xRightPos += Math.sin(Math.toRadians(angle)) * rightDistance;
		yRightPos += Math.cos(Math.toRadians(angle)) * rightDistance;
	}
	
	public void setRightX(double pos){
		xRightPos = pos;
	}
	
	public void setRightY(double pos){
		yRightPos = pos;
	}
	
	public double getLeftXPos(){
		return xLeftPos;
	}
	
	public double getRightXPos(){
		return xRightPos;
	}
	
	public double getLeftYPos(){
		return yLeftPos;
	}
	
	public double getRightYPos(){
		return yRightPos;
	}
}
