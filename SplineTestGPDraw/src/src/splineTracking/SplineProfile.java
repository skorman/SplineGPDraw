package src.splineTracking;

public class SplineProfile {

	private double Kp, Ki, Kd, Ka, Kv, goal, cruiseVel, 
	splineCruiseVel, maxAcc, cruiseVelScaleFactor, splineRadius,
	splineAngle, totalDistance, lastInnerPos = 0, sumAngle = 0,
	leftStartPos, rightStartPos, motionProfileStartVel,
	motionProfileDecelerateVel;
	private double lastTime;
	public Segment currentSegment = new Segment(0, 0, 0);
	public Segment nextSegment = new Segment(0, 0, 0);
	public Segment currentOuterSegment = new Segment(0, 0, 0);
	public Segment currentInnerSegment = new Segment(0, 0, 0);
	public Segment nextOuterSegment = new Segment(0, 0, 0);
	public Segment nextInnerSegment = new Segment(0, 0, 0);
	private double maxSplineVel;
	private double width = 3;
	private double rightOutput = 0, leftOutput = 0;
	private boolean angleInverted = false, isFullProfile = false, splineBackwards = false;
	
	public SplineProfile(double Kp, double Ki, double Kd, double Ka,
			double Kv){
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Ka = Ka;
		this.Kv = Kv;
		currentSegment = new Segment(0, 0, 0);
		nextSegment = new Segment(0, 0, 0);
		currentOuterSegment = new Segment(0, 0, 0);
		currentInnerSegment = new Segment(0, 0, 0);
		nextOuterSegment = new Segment(0, 0, 0);
		nextInnerSegment = new Segment(0, 0, 0);
	}
	
	private static class Segment{
		public double pos, vel, acc;
		
		public Segment(double pos, double vel, double acc){
			this.pos = pos;
			this.vel = vel;
			this.acc = acc;
		}
	}
	
	public void configureNewMotionProfile(double distance, double leftCurrentPos, double rightCurrentPos){
		this.isFullProfile = false;
		this.goal = distance;
		this.totalDistance = this.goal;
		this.maxAcc = 0.0002;
		this.cruiseVelScaleFactor = 0.95;
		if(distance < 0){
			this.maxAcc *= -1;
		}
		this.cruiseVel = getCruiseVel(this.goal);
		if(distance < 0){
			this.cruiseVel *= -1;
		}
		this.leftStartPos = leftCurrentPos;
		this.rightStartPos = rightCurrentPos;
		this.currentSegment = new Segment(0, 0, 0);
		this.nextSegment = new Segment(0, 0, 0);
		this.currentOuterSegment = new Segment(0, 0, 0);
		this.currentInnerSegment = new Segment(0, 0, 0);
		this.nextOuterSegment = new Segment(0, 0, 0);
		this.nextInnerSegment = new Segment(0, 0, 0);
		this.motionProfileDecelerateVel = 0;
		setState(MotionState.ACCELERATING);
	}
	
	public void configureNewMotionProfile(double distance, double leftCurrentPos, double rightCurrentPos, double decelerateVel, double startVel){
		this.isFullProfile = false;
		this.goal = distance;
		this.totalDistance = this.goal;
		this.maxAcc = 0.0002;
		this.cruiseVelScaleFactor = 0.95;
		if(distance < 0){
			this.maxAcc *= -1;
		}
		this.cruiseVel = getCruiseVel(this.goal);
		if(distance < 0){
			this.cruiseVel *= -1;
		}
		this.leftStartPos = leftCurrentPos;
		this.rightStartPos = rightCurrentPos;
		this.currentSegment = new Segment(0, startVel, 0);
		this.nextSegment = new Segment(0, 0, 0);
		this.currentOuterSegment = new Segment(0, 0, 0);
		this.currentInnerSegment = new Segment(0, 0, 0);
		this.nextOuterSegment = new Segment(0, 0, 0);
		this.nextInnerSegment = new Segment(0, 0, 0);
		this.motionProfileDecelerateVel = decelerateVel;
		setState(MotionState.ACCELERATING);
	}
	
	public void configureNewFullProfile(double distance, double radius, double angle, double pointToAvoid, double leftCurrentPos, double rightCurrentPos){
		this.isFullProfile = true;
		this.goal = pointToAvoid - radius;
		this.totalDistance = distance;
		this.maxAcc = 0.0002;
		this.cruiseVelScaleFactor = 0.95;
		this.maxSplineVel = 11231;
		this.splineAngle = Math.abs(angle);
		this.leftStartPos = leftCurrentPos;
		this.rightStartPos = rightCurrentPos;
		if(angle < 0) angleInverted = true;
		System.out.println(angle);
		System.out.println(angleInverted);
		this.splineRadius = radius;
		if(distance < 0){
			this.maxAcc *= -1;
			this.goal *= -1;
			this.splineBackwards = true;
		}
		this.cruiseVel = getCruiseVel(this.goal);
		if(distance < 0){
			this.cruiseVel *= -1;
		}
		this.splineCruiseVel = getSplineCruiseVel(this.goal);
		this.motionProfileDecelerateVel = this.splineCruiseVel;
		this.currentSegment = new Segment(0, 0, 0);
		this.nextSegment = new Segment(0, 0, 0);
		this.currentOuterSegment = new Segment(0, 0, 0);
		this.currentInnerSegment = new Segment(0, 0, 0);
		this.nextOuterSegment = new Segment(0, 0, 0);
		this.nextInnerSegment = new Segment(0, 0, 0);
		this.isFullProfile = true;
		setState(MotionState.ACCELERATING);
	}
	
	public void configureSplineProfile(double radius, double angle, double leftCurrentPos, double rightCurrentPos) {
		this.isFullProfile = false;
		this.goal = angle * radius;
		this.maxAcc = 0.0002;
		this.cruiseVelScaleFactor = 0.7;
		this.maxSplineVel = 11231;
		this.splineAngle = Math.abs(angle);
		this.leftStartPos = leftCurrentPos;
		this.rightStartPos = rightCurrentPos;
		if(angle < 0) angleInverted = true;
		this.splineRadius = radius;
		this.totalDistance = this.splineRadius * this.splineAngle;
		this.cruiseVel = getCruiseVel(this.totalDistance);
		this.splineCruiseVel = getSplineCruiseVel(this.totalDistance);
		this.currentSegment = new Segment(0, 0, 0);
		this.nextSegment = new Segment(0, 0, 0);
		this.currentOuterSegment = new Segment(0, 0, 0);
		this.currentInnerSegment = new Segment(0, 0, 0);
		this.nextOuterSegment = new Segment(0, 0, 0);
		this.nextInnerSegment = new Segment(0, 0, 0);
		setState(MotionState.SPLINE);
	}
	
	public void configureSplineProfile(double radius, double angle, double leftCurrentPos, double rightCurrentPos, double startVel, boolean direction){
		this.angleInverted = false;
		this.isFullProfile = false;
		this.maxAcc = 0.0002;
		this.cruiseVelScaleFactor = 0.7;
		this.maxSplineVel = 11231;
		this.splineAngle = Math.abs(angle);
		this.leftStartPos = leftCurrentPos;
		this.rightStartPos = rightCurrentPos;
		if(angle < 0) angleInverted = true;
		this.splineRadius = radius;
		this.goal = this.splineAngle * this.splineRadius;
		this.totalDistance = this.splineRadius * this.splineAngle;
		//if(!direction) this.totalDistance *= -1;
		this.cruiseVel = startVel;
		this.splineCruiseVel = getSplineCruiseVel(this.totalDistance);
		this.splineBackwards = !direction;
		if(!direction) {
			this.splineCruiseVel *= -1;
			this.angleInverted = true;
		}
		this.currentSegment = new Segment(0, 0, 0);
		this.nextSegment = new Segment(0, 0, 0);
		this.currentOuterSegment = new Segment(0, 0, 0);
		this.currentInnerSegment = new Segment(0, 0, 0);
		this.nextOuterSegment = new Segment(0, 0, 0);
		this.nextInnerSegment = new Segment(0, 0, 0);
		setState(MotionState.SPLINE);
	}
	
	private enum MotionState{
		ACCELERATING, CRUISING, DECELERATING, SPLINE, END
	}
	
	private MotionState state = MotionState.END;
	
	private void setState(MotionState newState){
		state = newState;
	}
	
	private MotionState getState(){
		return state;
	}
	
	private double getCruiseVel(double distance){
		double halfDist = distance / 2;
		double maxVelOverHalfDistance = Math.sqrt(2 * halfDist * maxAcc);
		//System.out.println(maxVelOverHalfDistance);
		return Math.min(maxVelOverHalfDistance * cruiseVelScaleFactor, 345223456);
	}
	
	private double getCruiseVel(double distance, double initialVel){
		double halfDist = distance / 2;
		double maxVelOverHalfDistance = Math.sqrt(2 * halfDist * maxAcc) + initialVel;
		//System.out.println(maxVelOverHalfDistance);
		return Math.min(maxVelOverHalfDistance * cruiseVelScaleFactor, 345223456);
	}
	
	private double getSplineCruiseVel(double distance){
		if(distance > 0) return Math.min(this.cruiseVel, maxSplineVel);
		return Math.max(this.cruiseVel, -maxSplineVel);
	}
	
	public void calculate(double time){
		if(getState() == MotionState.DECELERATING){
			System.out.println(getState());
		}
		//System.out.println(currentOuterSegment.pos);
		//System.out.println(currentInnerSegment.pos);
		double dt;
		double currentTime = time;
		dt = currentTime - lastTime;
		lastTime = currentTime;
		/*
		double dt;
		double currentTime = Timer.getFPGATimestamp();
		dt = currentTime - lastTime;
		lastTime = currentTime;
		*/
		
		if(getState() == MotionState.SPLINE){
			splineCalculate(dt);
			return;
		}

		double currentVel = currentSegment.vel;
		double distanceToGo = goal - currentSegment.pos;
		
		double t_to_cruise = Math.abs((cruiseVel - currentVel) / maxAcc); //time to accelerate to cruise speed
		double x_to_cruise = currentVel * t_to_cruise + .5 * maxAcc * t_to_cruise * t_to_cruise; //distance to get to cruise speed
		
		double t_to_spline = Math.abs((cruiseVel - motionProfileDecelerateVel) / maxAcc); //time to get to zero speed from cruise speed
		double x_to_spline = currentVel * t_to_spline - .5 * maxAcc * t_to_spline * t_to_spline; //distance to get to zero speed
						
		double cruiseX;
		if(goal > 0){
			cruiseX  = Math.max(0, distanceToGo - x_to_cruise - x_to_spline);
		}
		else{
			cruiseX  = Math.min(0, distanceToGo - x_to_cruise - x_to_spline);
		}
		double cruiseT = Math.abs(cruiseX / cruiseVel);
				
		if (getState() == MotionState.ACCELERATING){
			if (t_to_cruise < dt){
				setState(MotionState.CRUISING);
			}
		}
		
		if(getState() == MotionState.CRUISING){
			if(t_to_cruise + cruiseT < dt){
				setState(MotionState.DECELERATING);
			}
		}
		
		if(getState() == MotionState.DECELERATING){
			if(t_to_cruise + cruiseT + t_to_spline < dt){
				if (this.isFullProfile) setState(MotionState.SPLINE);
				else setState(MotionState.END);
			}
		}
		
		if(getState() == MotionState.ACCELERATING){
			nextSegment.pos = currentVel * dt + .5 * maxAcc * dt * dt;
			nextSegment.vel = currentVel + dt * maxAcc;
			nextSegment.acc = maxAcc;
			//System.out.println("accelerating");
		}
		else if(getState() == MotionState.CRUISING){
			nextSegment.pos = cruiseVel * dt;
			nextSegment.vel = cruiseVel;
			nextSegment.acc = 0;
			//System.out.println("cruising");
		}
		else if(getState() == MotionState.DECELERATING){
			nextSegment.pos = currentVel * dt - 0.5 * maxAcc * dt * dt;
			nextSegment.vel = currentVel - maxAcc * dt;
			nextSegment.acc = -maxAcc;
			//System.out.println("decelerating");
		}
		else if(getState() == MotionState.SPLINE){
			nextSegment.pos = distanceToGo;
			nextSegment.vel = splineCruiseVel;
			nextSegment.acc = 0;
		}
		else{
			nextSegment.pos = 0;
			nextSegment.vel = 0;
			nextSegment.acc = 0;
		}
		
		currentSegment.pos += nextSegment.pos;
		currentSegment.vel = nextSegment.vel;
		currentSegment.acc = nextSegment.acc;
		
		currentOuterSegment.pos += nextSegment.pos;
		currentOuterSegment.vel = currentSegment.vel;
		currentOuterSegment.acc = currentSegment.acc;
		
		currentInnerSegment.pos += nextSegment.pos;
		currentInnerSegment.vel = currentSegment.vel;
		currentInnerSegment.acc = currentSegment.acc;
		
		double outerOutput = Kv * currentOuterSegment.vel + Ka * currentOuterSegment.acc;
		double innerOutput = Kv * currentInnerSegment.vel + Ka * currentInnerSegment.acc;
		
		if(splineAngle >= 0) {
			rightOutput = outerOutput;
			leftOutput = innerOutput;
		}
		else{
			rightOutput = innerOutput;
			leftOutput = outerOutput;
		}
	}
	
	public double getRightOutput(){
		if(angleInverted) return currentInnerSegment.pos;
		return currentOuterSegment.pos;
	}
	
	public double getLeftOutput(){
		if(angleInverted) return currentOuterSegment.pos;
		return currentInnerSegment.pos;
	}
	
	public double getAngle(){
		//if(angleInverted) return -sumAngle;
		return sumAngle;
	}
	
	public double getVel(){
		return currentSegment.vel;
	}
	
	public double getNextOuterDist(){
		if(!(getState() == MotionState.SPLINE)){
			return nextSegment.pos;
		}
		if(angleInverted) return nextInnerSegment.pos;
		return nextOuterSegment.pos;
	}
	
	public double getNextInnerDist(){
		if(!(getState() == MotionState.SPLINE)){
			return nextSegment.pos;
		}
		if(angleInverted) return nextOuterSegment.pos;
		return nextInnerSegment.pos;
	}
	
	private void splineCalculate(double delta_t){
		double dt = delta_t;
				
		double currentInnerVel = currentInnerSegment.vel;
		double outerCircDistance = splineAngle * splineRadius;
		if(this.splineBackwards) outerCircDistance *= -1;
				
		nextOuterSegment.pos = splineCruiseVel * dt;
		nextOuterSegment.vel = splineCruiseVel;
		nextOuterSegment.acc = 0;
		
		double innerRadius = splineRadius - width;
		double angle = nextOuterSegment.pos / splineRadius;
		if(angleInverted) sumAngle -= Math.toDegrees(angle);
		else sumAngle += Math.toDegrees(angle);

		nextInnerSegment.pos = angle * innerRadius;
		double displacement = nextInnerSegment.pos;
		nextInnerSegment.vel = displacement / dt;
		lastInnerPos = displacement;
		double finalVel = nextInnerSegment.vel;
		nextInnerSegment.acc = (finalVel - currentInnerVel) / dt;
				
		currentOuterSegment.pos += nextOuterSegment.pos;
		currentOuterSegment.vel = nextOuterSegment.vel;
		currentOuterSegment.acc = nextOuterSegment.acc;
		
		currentInnerSegment.pos += nextInnerSegment.pos;
		currentInnerSegment.vel = nextInnerSegment.vel;
		currentInnerSegment.acc = nextInnerSegment.acc;
		
		double outerOutput = Kv * currentOuterSegment.vel + Ka * currentOuterSegment.acc;
		double innerOutput = Kv * currentInnerSegment.vel + Ka * currentInnerSegment.acc;
		
		if(splineAngle >= 0) {
			rightOutput = outerOutput;
			leftOutput = innerOutput;
		}
		else{
			rightOutput = innerOutput;
			leftOutput = outerOutput;
		}
			
		double x_to_goal = outerCircDistance + currentSegment.pos - currentOuterSegment.pos;
		double t_to_goal = Math.abs(x_to_goal / splineCruiseVel);
	
		//System.out.println(outerCircDistance);
		
		if(t_to_goal < dt){
			if(!this.isFullProfile) this.totalDistance = outerCircDistance;
			if(this.isFullProfile) setState(MotionState.ACCELERATING);
			else setState(MotionState.END);
			double finalDist = this.totalDistance - this.goal - outerCircDistance;
			//System.out.println(goal + "  " + outerCircDistance + "  " + totalDistance + "  " + finalDist);
			this.goal = totalDistance;
			this.cruiseVel = getCruiseVel(finalDist);
			currentSegment.vel = currentOuterSegment.vel;
			currentSegment.pos = currentOuterSegment.pos;
			currentSegment.acc = currentOuterSegment.acc;
			this.splineCruiseVel = 0;
		}
	}
	public boolean isFinishedTrajectory() {
        return Math.abs(currentOuterSegment.pos - totalDistance) < 1
                && Math.abs(currentSegment.vel) < 1;
    }
}
