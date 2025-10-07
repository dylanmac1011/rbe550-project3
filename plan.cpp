#include "BoxChain.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    goal[4] = -1.57079;

    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);
}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 
    goal[4] = -1.57079;

    //Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}

void planScenario1(ompl::geometric::SimpleSetup &ss)
{
    ss.setPlanner(std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation()));
    ompl::base::PlannerStatus solved = ss.solve(20.0);
    
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().printAsMatrix(std::cout);
        std::ofstream outputFile;
        //clear file
        outputFile.open("narrow_path.txt", std::ofstream::out | std::ofstream::trunc);
        outputFile.close();
        //reopen and fill
        outputFile.open("narrow_path.txt");
        if (outputFile.is_open()) {
            ss.getSolutionPath().printAsMatrix(outputFile);
            outputFile.close();
        } else {std::cerr << "Unable to open path file" << std::endl;}
    }
    else{std::cout << "No solution found" << std::endl;}         
}

void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark PRM with uniform, bridge, gaussian, and obstacle-based Sampling. Do 20 trials with 20 seconds each 
    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");
}

void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    auto si = ss.getSpaceInformation();
    ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(si));
    ss.setOptimizationObjective(std::make_shared<BoxChainClearanceObjective>(si));
    ompl::base::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().printAsMatrix(std::cout);
        std::ofstream outputFile;
        //clear file
        outputFile.open("clear_path.txt", std::ofstream::out | std::ofstream::trunc);
        outputFile.close();
        //reopen and fill
        outputFile.open("clear_path.txt");
        if (outputFile.is_open()) {
            ss.getSolutionPath().printAsMatrix(outputFile);
            outputFile.close();
        } else {std::cerr << "Unable to open path file" << std::endl;}
    }
    else{std::cout << "No solution found" << std::endl;}  
}

void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark RRT*, PRM*, RRT# for 10 trials with 60 secounds timeout.
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");
}

// Orriginal version without ChainBoxSpace
std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{
    // Create the component spaces: 4 link manipulator + SE2 robot
    auto arm_space = std::make_shared<KinematicChainSpace>(4, 1.0);
    auto se2_space = std::make_shared<ompl::base::SE2StateSpace>();

    // Create the bounds of the SE2 state space
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -5);
    bounds.setLow(1, -5);
    bounds.setHigh(0, 5);
    bounds.setHigh(1, 5);
    se2_space->setBounds(bounds);

    // Create the compound state space
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();
    space->addSubspace(se2_space, 1.0);
    space->addSubspace(arm_space, 1.0);
    return space;
}
// Better object oriented version
std::shared_ptr<BoxChainSpace> createChainBoxSpace(Environment &env)
{
    return std::make_shared<BoxChainSpace>(&env);
}
void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env)
{
    auto si = ss.getSpaceInformation();
    auto checker = std::make_shared<BoxChainValidityChecker>(si);
    ss.setStateValidityChecker(checker);
}

    
int main(int argc, char **argv)
{
    using Environment = std::vector<Segment>;
    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 3);

    switch (scenario)
    {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace(env);
    ompl::geometric::SimpleSetup ss(space);
    setupCollisionChecker(ss, env);

    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);
    
    switch (scenario)
    {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss);
            benchScenario2(ss);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

}
