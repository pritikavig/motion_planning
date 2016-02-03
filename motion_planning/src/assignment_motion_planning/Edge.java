package assignment_motion_planning;

public class Edge {
	
	private Vector start;
	private Vector end;
	private double duration;
	private Vector control;
	
	public Edge(Vector s, Vector e, double duration, Vector control){
		this.start = s;
		this.end = e;
		this.duration = duration;
		this.control = control;
		
	}
	
	public Vector getStart(){
		return start;
	}
	
	public Vector getEnd(){
		return end;
	}
	
	public double getDur(){
		return duration;
	}
	
	public Vector getControl(){
		return control;
	}
	
	

}
