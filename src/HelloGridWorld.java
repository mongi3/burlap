import java.awt.Color;
import java.util.List;

import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.PerformancePlotter;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.GoalBasedRF;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.*;
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.auxiliary.common.ConstantStateGenerator;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SinglePFTF;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.Visualizer;

public class HelloGridWorld{
	final static int GRID_DIM = 50;
	
	GridWorldDomain				gw;
	Domain						domain;
	StateParser					sp;
	RewardFunction				rf;
	TerminalFunction			tf;
	StateConditionTest			goalCondition;
	State						initialState;
	DiscreteStateHashFactory	hashingFactory;
	StateGenerator				sg;
	
	public static void main(String [] args){
		HelloGridWorld hw = new HelloGridWorld();
	}
	
	HelloGridWorld() {

		boolean MAZE = true; 
		char[][] maz = null;
		if(MAZE) {
			gw = new GridWorldDomain(GRID_DIM,GRID_DIM);
			// Generate maze for world
			Prim p = new Prim(gw.getWidth(), gw.getHeight());
			maz = p.getMaze(0);
			
			// setup maze walls
			for(int i=0;i<maz.length;i++) {
				for(int j=0;j<maz[0].length;j++) {
					if(maz[i][j] == '*') {
						gw.setObstacleInCell(i, j);
					}
				}
			}
		}
		else {
			// Four rooms
			gw = new GridWorldDomain(11,11); //11x11 grid world
			gw.setMapToFourRooms(); //four rooms layout		
		}
		
		gw.setProbSucceedTransitionDynamics(1.0); //stochastic transitions with 0.8 success rate
		domain = gw.generateDomain(); //generate the grid world domain

		System.out.printf("GridSize(w x h): %d x %d\n", gw.getWidth(), gw.getHeight());
		
		//setup initial state (start and end goals)
		initialState = GridWorldDomain.getOneAgentOneLocationState(domain);
		if(MAZE) {
			for(int i=0;i<maz.length;i++) {
				for(int j=0;j<maz[0].length;j++) {
					if(maz[i][j] == 'S') {
						GridWorldDomain.setAgent(initialState, i, j);
						System.out.printf("Agent Start (r, u-zero based): %d x %d\n", i, j);
					}
					if(maz[i][j] == 'E') {
						GridWorldDomain.setLocation(initialState, 0, i, j);
						System.out.printf("Goal (r, u-zero based): %d x %d\n", i, j);
					}
				}
			}
		}
		else {
			GridWorldDomain.setAgent(initialState, 0, 0);
			GridWorldDomain.setLocation(initialState, 0, 10, 10);
		}
		
		//ends when the agent reaches a location
		tf = new SinglePFTF(domain.
			getPropFunction(GridWorldDomain.PFATLOCATION)); 

		//reward function definition
		rf = new GoalBasedRF(new TFGoalCondition(tf), 5., -0.1);

		//initial state generator
		sg = new ConstantStateGenerator(initialState);
		sp = new GridWorldStateParser(domain); 

//		//set up the state hashing system for looking up states
//		hashingFactory = new DiscreteStateHashFactory();
//
		//set up the state hashing system
		hashingFactory = new DiscreteStateHashFactory();
		hashingFactory.setAttributesForClass(GridWorldDomain.CLASSAGENT, 
				domain.getObjectClass(GridWorldDomain.CLASSAGENT).attributeList);
		
		ValueIterationExample("test");
//		testPerformance();
		PolicyIterationExample("test");
		QLearningExample("test");
		
//		visualize();
		
//		VisualActionObserver observer = new VisualActionObserver(domain, 
//				GridWorldVisualizer.getVisualizer(gw.getMap()));
//				((SADomain)this.domain).setActionObserverForAllAction(observer);
//				observer.initGUI();
	}

	public void ValueIterationExample(String outputPath){
		
		System.out.printf("===============\n");
		System.out.printf("VALUE ITERATION\n");
		System.out.printf("===============\n");

		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}
		
		
		OOMDPPlanner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 10000);
		
		final long startTime = System.currentTimeMillis();
		planner.planFromState(initialState);
		final long endTime = System.currentTimeMillis();
		System.out.println("Total execution time: " + (endTime - startTime) );
		
		//create a Q-greedy policy from the planner
		Policy p = new GreedyQPolicy((QComputablePlanner)planner);
		
		// Get plan data
		EpisodeAnalysis ea = p.evaluateBehavior(initialState, rf, tf);
		printStats(ea, outputPath);
		
		//visualize the value function and policy
		System.out.printf("------------\n");
		System.out.printf("GUI outputs:\n");
		System.out.printf("------------\n");
		this.valueFunctionVisualize((QComputablePlanner)planner, p);
	}
	
	private void printStats(EpisodeAnalysis ea, String outputPath) {
		//record the plan results to a file
		ea.writeToFile(outputPath + "planResult", sp);
		
		// Output Performance Stats
		System.out.printf("------------\n");
		System.out.printf("Plan Results\n");
		System.out.printf("------------\n");
		int numTimeSteps = ea.numTimeSteps();
		double discountedReward = ea.getDiscountedReturn(.99);
		System.out.println("Time Steps: " + numTimeSteps);
		System.out.println("Discounted Reward: " + discountedReward);
		System.out.println("Avg Reward: " + discountedReward / numTimeSteps);
	}
	
	public void PolicyIterationExample(String outputPath){
		
		System.out.printf("================\n");
		System.out.printf("POLICY ITERATION\n");
		System.out.printf("================\n");

		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}
		
		
		PolicyIteration planner = new PolicyIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 1000, 100);

		final long startTime = System.currentTimeMillis();
		planner.planFromState(initialState);
		final long endTime = System.currentTimeMillis();
		System.out.println("Total execution time: " + (endTime - startTime) );
		
		//create a Q-greedy policy from the planner
//		Policy p = new GreedyQPolicy((QComputablePlanner)planner);
		Policy p = planner.getComputedPolicy();
		
		EpisodeAnalysis ea = p.evaluateBehavior(initialState, rf, tf);
		printStats(ea, outputPath);

		//visualize the value function and policy
		System.out.printf("------------\n");
		System.out.printf("GUI outputs:\n");
		System.out.printf("------------\n");
		this.valueFunctionVisualize((QComputablePlanner)planner, p);
	}
	
	public void QLearningExample(String outputPath){
		
		System.out.printf("==========\n");
		System.out.printf("Q-LEARNING\n");
		System.out.printf("==========\n");

		if(!outputPath.endsWith("/")){
			outputPath = outputPath + "/";
		}
		
		//creating the learning algorithm object; discount= 0.99; initialQ=0.0; learning rate=0.9
		LearningAgent agent = new QLearning(domain, rf, tf, 0.99, hashingFactory, 0., 0.9);
		
		//run learning for 100 episodes
		for(int i = 0; i < 200; i++){
			EpisodeAnalysis ea = agent.runLearningEpisodeFrom(initialState);
//			ea.writeToFile(String.format("%se%03d", outputPath, i), sp); 
			System.out.println(i + ": " + ea.numTimeSteps());
			printStats(ea, outputPath);
		}
		
		experimenterAndPlotter();
		//visualize the value function and policy
//		this.valueFunctionVisualize((QComputablePlanner)planner, p);
	}	
	
	public void valueFunctionVisualize(QComputablePlanner planner, Policy p){
		List <State> allStates = StateReachability.getReachableStates(initialState, 
			(SADomain)domain, hashingFactory);
		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);
		
		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX, 
			GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
		
		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX, 
			GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTY);
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphRenderStyle.DISTSCALED);
		
		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, planner);
		gui.setSpp(spp);
		gui.setPolicy(p);
		gui.setBgColor(Color.GRAY);
		gui.initGUI();
	}
	
	public void visualize() {
		//create visualizer and explorer
		Visualizer v = GridWorldVisualizer.getVisualizer(gw.getMap());
		VisualExplorer exp = new VisualExplorer(domain, v, initialState);
		
		//set control keys to use w-s-a-d
		exp.addKeyAction("w", GridWorldDomain.ACTIONNORTH);
		exp.addKeyAction("s", GridWorldDomain.ACTIONSOUTH);
		exp.addKeyAction("a", GridWorldDomain.ACTIONWEST);
		exp.addKeyAction("d", GridWorldDomain.ACTIONEAST);
		
		exp.initGUI();
	}
	public void testPerformance() {
		PerformancePlotter pp = new PerformancePlotter("Joe", rf, 500, 250, 2, 1000, TrialMode.MOSTRECENTANDAVERAGE, 
				PerformanceMetric.CUMULATIVESTEPSPEREPISODE, 
				PerformanceMetric.AVERAGEEPISODEREWARD);
		pp.startGUI();
	}
	public void experimenterAndPlotter() {
		
		//custom reward function for more interesting results
//		final RewardFunction rf = new GoalBasedRF(this.goalCondition, 5., -0.1);

		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */

		LearningAgentFactory qLearningFactory = new LearningAgentFactory() {
			
			@Override
			public String getAgentName() {
				return "Q-learning";
			}
			
			@Override
			public LearningAgent generateAgent() {
				return new QLearning(domain, rf, tf, 0.99, hashingFactory, 0.0, 0.1);
			}
		};

//		LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {
//			
//			@Override
//			public String getAgentName() {
//				return "SARSA";
//			}
//			
//			@Override
//			public LearningAgent generateAgent() {
//				return new SarsaLam(domain, rf, tf, 0.99, hashingFactory, 0.0, 0.1, 1.);
//			}
//		};

//		StateGenerator sg = new ConstantStateGenerator(this.initialState);

		final int numTrial = 10;
		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter((SADomain)this.domain, 
			rf, sg, numTrial, 2000, qLearningFactory);

		exp.setUpPlottingConfiguration(500, 250, 2, 1000, 
			TrialMode.MOSTRECENTANDAVERAGE, 
			PerformanceMetric.CUMULATIVESTEPSPEREPISODE, 
			PerformanceMetric.AVERAGEEPISODEREWARD);

		final long startTime = System.currentTimeMillis();
		exp.startExperiment();
		final long endTime = System.currentTimeMillis();
		System.out.println("Total execution time/trial: " + (endTime - startTime)/(float)numTrial );


		exp.writeStepAndEpisodeDataToCSV("expData");


	}

}
				