package src.splineTracking;

import java.util.Scanner;

import gpdraw.DrawingTool;
import gpdraw.SketchPad;

public class Draw {

	SketchPad paper;
	static DrawingTool pen;
	double xPos;
	double yPos;
	double rightDistance = 0, leftDistance = 0;
	Scanner in;
	public static PositionTracker leftPosition = new PositionTracker();
	public static PositionTracker rightPosition = new PositionTracker();
	GoToPosition find = new GoToPosition();
	SplineProfile sp = new SplineProfile(0, 0, 0, 0.05, 0.05);

	
	public Draw(){

		paper = new SketchPad(100000, 100000);
		pen = new DrawingTool(paper);
		in = new Scanner(System.in);
		xPos = 0;
		yPos = 0;
	}
	
	public void draw(){
		rightPosition.setRightX(30);
		leftPosition.setRightY(-200);
		rightPosition.setRightY(-200);
		double i = 0;
		
		//System.out.println(rightPosition.getRightXPos());
		
		sp.configureSplineProfile(6, Math.toRadians(90), 0, 0.02, true);
		sp.initializeProfile(0, 0);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
		}
		
		i = 0;
		
		sp.configureSplineProfile(6, Math.toRadians(-90), 0.02, 0, true);
		sp.initializeProfile(0, 0);
		//System.out.println(sp.getCruiseVel());
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
		}
		//System.out.println(-(pen.getDirection() + 270));
		//System.out.println(sp.getAngle());
		//i = 0;
		/*
		sp.configureSplineProfile(5, Math.toRadians(-45), 0.05, true);
		sp.initializeProfile(0, 0);
		//System.out.println(sp.getAngle());
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			//System.out.println(sp.getAngle());
			//System.out.println(sp.isFinishedTrajectory());
			doSomething();
		}
		/*
		/*
		System.out.println(rightPosition.getRightXPos() - 30);
		System.out.println(leftPosition.getRightXPos());
		
		
		pen.up();
		pen.move(0, 0);
		pen.setDirection(0);
		double angle = Math.cos(Math.toRadians(45));
		double secondAngle = Math.sin(Math.toRadians(45));
		double a = 50 - (angle * 50);
		double b = ((secondAngle * 50)) + ((secondAngle * 20));
		double c = 20 - (angle * 20);
		pen.down();
		pen.forward(a);
		pen.setDirection(90);
		pen.forward(b);
		pen.setDirection(0);
		pen.forward(c);
		
		System.out.println(a + c);
		*/
		/*
		sp.configureSplineProfile(1.5, Math.toRadians(-85), 0, 0, 0.1, true);
		while(!sp.isFinishedTrajectory()){
			sp.setTolerance(0.01);
			i += (Math.random() * 0.05);
			sp.calculate(i);
			doSomething();
		}
/*
		sp.configureNewMotionProfile(20, 0, 0, 0, 0);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
		}
		/*
		sp.configureNewMotionProfile(20, 0, 0, 0, 0);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			/*
			pen.up();
			pen.move(i - 250, (sp.getVel() * 1000) - 150);
			//System.out.println(sp.getRightOutput());
			pen.down();
			pen.drawCircle(0.01);
			doSomething();
		}
		sp.configureSplineProfile(10, Math.toRadians(-180), 0, 0, 0.05, false);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
			//System.out.println("not good");
		}
		sp.configureNewMotionProfile(-20, 0, 0, 0, -0.05);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
			//System.out.println("good");
		}/*
		sp.configureSplineProfile(20, Math.toRadians(-90), 0, 0, 0.06, false);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
		}
		/*
		sp.configureNewFullProfile(-30, 6, Math.toRadians(45), 10, 0, 0);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 2);
			sp.calculate(i);
			doSomething();
		}
		*/
		//System.out.println(sp.getRightOutput());
		
	}
	private void doSomething(){
		
		rightDistance += sp.getRightOutput() * 10;
		rightPosition.setPosistions(sp.getAngle(), rightDistance);
		
		leftDistance += sp.getLeftOutput() * 10;
		leftPosition.setPosistions(sp.getAngle(), leftDistance);

		pen.up();
		pen.move(rightPosition.getRightXPos(), rightPosition.getRightYPos());
		pen.setDirection(-(sp.getAngle() + 270));
		pen.down();
		pen.forward(sp.getRightOutput() * 10);
		
		//System.out.println(-(pen.getDirection() + 270));
	
		
		pen.up();
		pen.move(leftPosition.getRightXPos(), leftPosition.getRightYPos());
		pen.setDirection(-(sp.getAngle() + 270));
		pen.down();
		pen.forward(sp.getLeftOutput() * 10);
		
	}
	
	

	
	
	
	
	
	
	
	
	
	/*
	double distance = 0;
	
	pen.move(0, 0);
	pen.setDirection(0);

	pen.turnLeft(45);
	pen.forward(60);
	distance += 60;
	PositionTracker.setPosistions(45, distance);
	
	System.out.println(PositionTracker.getLeftXPos() == pen.getXPos() && PositionTracker.getLeftYPos() == pen.getYPos());

	
	pen.setDirection(0);
	pen.turnLeft();
	pen.forward(43);
	distance += 43;
	PositionTracker.setPosistions(90, distance);
	
	System.out.println(PositionTracker.getLeftXPos() == pen.getXPos() && PositionTracker.getLeftYPos() == pen.getYPos());
	
	pen.setDirection(0);
	pen.turnLeft(54);
	pen.forward(23);
	distance += 23;
	PositionTracker.setPosistions(54, distance);
	
	System.out.println(PositionTracker.getLeftXPos() == pen.getXPos() && PositionTracker.getLeftYPos() == pen.getYPos());

	pen.setDirection(0);
	pen.turnRight(104);
	pen.forward(23);
	distance += 23;
	PositionTracker.setPosistions(-104, distance);
	
	System.out.println(PositionTracker.getLeftXPos() == pen.getXPos() && PositionTracker.getLeftYPos() == pen.getYPos());

	pen.setDirection(0);
	pen.turnRight(104);
	pen.forward(103);
	distance += 103;
	PositionTracker.setPosistions(-104, distance);
	
	System.out.println(PositionTracker.getLeftXPos() == pen.getXPos() && PositionTracker.getLeftYPos() == pen.getYPos());
	*/
	

	/*
	double distance = 0;
	double angle = 10;  
	pen.move(0, 0);
	pen.setDirection(90);
	pen.setDirection(-(angle + 270));
	pen.forward(0);
	distance += 0;
	position.setPosistions(angle, distance);
	*/
	//System.out.println(position.getLeftXPos() == pen.getXPos() && position.getLeftYPos() == pen.getYPos());
	//System.out.println("Pos " + position.getLeftXPos() + " " + position.getLeftYPos());
	//System.out.println("Expected " + pen.getXPos() + " " + pen.getYPos());
	
	/*
	for(int i = -90; i <= 90; i += 20){
		angle = i;
		distance = 0;
		
		pen.move(200, -30);;
		pen.setDirection(-(angle + 270));
		pen.forward(360);
		distance += 360;
		position.setPosistions(angle, 360);
		
		System.out.println("Pos " + position.getLeftXPos() + " " + position.getLeftYPos());
		System.out.println("Expected " + pen.getXPos() + " " + pen.getYPos());
		System.out.println();
	}
	for(int i = -90; i <= 90; i += 20){
		angle = i;
		distance = 0;
		
		pen.move(200, -30);;
		pen.setDirection(-(angle + 270));
		pen.forward(-360);
		distance += -360;
		position.setPosistions(angle, -360);
		
		System.out.println("Pos " + position.getLeftXPos() + " " + position.getLeftYPos());
		System.out.println("Expected " + pen.getXPos() + " " + pen.getYPos());
		System.out.println();
	}
	*/
}
