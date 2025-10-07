#ifndef BOX_CHAIN_

#include "KinematicChain.h"

// Custom state space for the Box-Chain robot
class BoxChainSpace : public ompl::base::CompoundStateSpace
{
public:
    BoxChainSpace(Environment *env = nullptr) : environment_(env)
    {
        // Set up the SE2 space with proper bounds
        auto se2_space = std::make_shared<ompl::base::SE2StateSpace>();
        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow(0, -5);
        bounds.setLow(1, -5);
        bounds.setHigh(0, 5);
        bounds.setHigh(1, 5);
        se2_space->setBounds(bounds);

        // Create the workspace boundary environment for collision checking
        boundary_.emplace_back(5, 5, -5, 5);
        boundary_.emplace_back(-5, 5, -5, -5);
        boundary_.emplace_back(-5, -5, 5, -5);
        boundary_.emplace_back(5, -5, 5, 5);

        // Add the subspaces
        addSubspace(se2_space, 1.0);
        addSubspace(std::make_shared<KinematicChainSpace>(4, 1.0), 1.0);
        lock();
    }

    class StateType : public ompl::base::CompoundStateSpace::StateType
    {
    public:
        ompl::base::SE2StateSpace::StateType &getSE2State()
        { 
            return *as<ompl::base::SE2StateSpace::StateType>(0); 
        }
        const ompl::base::SE2StateSpace::StateType &getSE2State() const
        { 
            return *as<ompl::base::SE2StateSpace::StateType>(0); 
        }

        KinematicChainSpace::StateType &getChainState()
        {
            return *as<KinematicChainSpace::StateType>(1);
        }
        const KinematicChainSpace::StateType &getChainState() const
        {
            return *as<KinematicChainSpace::StateType>(1);
        }

        void getBox(Environment &env) const
        {
            // The robot represented at the origin
            env.emplace_back(0.5, 0.5, -0.5, 0.5);
            env.emplace_back(-0.5, 0.5, -0.5, -0.5);
            env.emplace_back(-0.5, -0.5, 0.5, -0.5);
            env.emplace_back(0.5, -0.5, 0.5, 0.5);

            transformEnv(env, &getSE2State());
        }

        void getChain(Environment &env, const KinematicChainSpace *space) const
        {
            getChainState().getArm(env, space);
            transformEnv(env, &getSE2State());
        }
    };

    KinematicChainSpace *getChainSpace() { return getSubspace(1)->as<KinematicChainSpace>(); }
    const KinematicChainSpace *getChainSpace() const { return getSubspace(1)->as<KinematicChainSpace>(); }

    const Environment *environment() const { return environment_; }
    const Environment *boundary() const { return &boundary_; }

private:
    Environment* environment_;
    Environment boundary_;
};

// Custom Validity Checker for the Box-Chain robot
class BoxChainValidityChecker : public KinematicChainValidityChecker
{
public:
    BoxChainValidityChecker(const ompl::base::SpaceInformationPtr &si) 
        : KinematicChainValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *s) const override
    {
        // Obtain state space from ompl
        auto space = si_->getStateSpace()->as<BoxChainSpace>();
        auto state = s->as<BoxChainSpace::StateType>();

        // Build the components of the environment
        Environment box, arm;
        state->getBox(box);
        state->getChain(box, space->getChainSpace());

        // Check each invalid condition
        return KinematicChainValidityChecker::selfIntersectionTest(arm)
            && KinematicChainValidityChecker::environmentIntersectionTest(arm, *space->environment())
            && KinematicChainValidityChecker::environmentIntersectionTest(box, *space->environment())
            && KinematicChainValidityChecker::environmentIntersectionTest(arm, *space->boundary())
            && KinematicChainValidityChecker::environmentIntersectionTest(box, *space->boundary()) 
            && selfIntersectionTestBox(box, arm);  
    }

protected:
    // returns true iff box only intersects with the first link of arm
    bool selfIntersectionTestBox(const Environment &box, const Environment &arm) const
    {
        for (unsigned int i = 0; i < box.size(); ++i)
            for (unsigned int j = 1; j < arm.size(); ++j)
                if (intersectionTest(box[i], arm[j]))
                    return false;
        return true;
    }

    Environment *environment_;
};

// Custom Clearance Objective for the Box-Chain robot
class BoxChainClearanceObjective : public ompl::base::OptimizationObjective
{
public:
    BoxChainClearanceObjective(const ompl::base::SpaceInformationPtr &si)
        : OptimizationObjective(si)
    {}

    ompl::base::Cost stateCost(const ompl::base::State *state) const override
    {
        return ompl::base::Cost(0.0);
    }

protected:
    
};


#endif // BOX_CHAIN_