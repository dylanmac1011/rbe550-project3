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
        Environment box, chain;
        state->getBox(box);
        state->getChain(chain, space->getChainSpace());

        // Check each invalid condition
        return KinematicChainValidityChecker::selfIntersectionTest(chain)
            && KinematicChainValidityChecker::environmentIntersectionTest(chain, *space->environment())
            && KinematicChainValidityChecker::environmentIntersectionTest(box, *space->environment())
            && KinematicChainValidityChecker::environmentIntersectionTest(chain, *space->boundary())
            && KinematicChainValidityChecker::environmentIntersectionTest(box, *space->boundary()) 
            && selfIntersectionTestBox(box, chain);  
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
    BoxChainClearanceObjective(const ompl::base::SpaceInformationPtr &si, bool use_center = true)
        : OptimizationObjective(si), use_center_(use_center)
    {}

    ompl::base::Cost stateCost(const ompl::base::State *s) const override
    {
        auto space = si_->getStateSpace()->as<BoxChainSpace>();
        auto state = s->as<BoxChainSpace::StateType>();

        // We invert the distance since OMPL minimizes the cost by default
        // Add 1 to normalize to between 0 and 1
        double min_dist = use_center_ ? costFromCenter(state, space) : costFromAll(state, space);
        return ompl::base::Cost(1.0 / (1.0 + min_dist));
    }

    ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        return ompl::base::Cost(std::min(stateCost(s1).value(), stateCost(s2).value()));
    }

protected:
    double costFromCenter(const BoxChainSpace::StateType* state, const BoxChainSpace* space)
    {
        auto pos = state->getSE2State();
        return distToEnv(*space->environment(), pos.getX(), pos.getY());
    }

    double costFromAll(const BoxChainSpace::StateType* state, const BoxChainSpace* space)
    {
        Environment box, chain;
        state->getBox(box);
        state->getChain(chain, space->getChainSpace());

        return std::min(distEnv(chain, *space->environment()), distEnv(box, *space->environment()));
    }

    // return the minimum distance between 2 envrionments
    double distEnv(const Environment &env1, const Environment &env2) const
    {
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto &seg : env1)
            min_dist = std::min(min_dist, distSegEnv(env2, seg));
        return min_dist;
    }

    // return the minimum distance between a segment and environment
    double distSegEnv(const Environment &env, const Segment &seg) const
    {
        return std::min(distToEnv(env, seg.x0, seg.y0), distToEnv(env, seg.x1, seg.y1));        
    }

    // Return the minimum distance between a point and environment
    double distToEnv(const Environment &env, double x, double y) const
    {
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& seg : env)
            min_dist = std::min(min_dist, distToSeg(seg, x, y));
        return min_dist;
    }

    // Return the minimum distance between a line segment and a point
    double distToSeg(const Segment& seg, double x, double y) const
    {
        double dx = seg.x1 - seg.x0, dy = seg.y1 - seg.y0;
        double numer = (x - seg.x0) * dx + (y - seg.y0) * dy;
        double denom = dx*dx + dy*dy;
        if (denom == 0)
        {   // Segment is just a point
            dx = x - seg.x0; dy = y - seg.y0;
            return std::sqrt(dx*dx + dy*dy);
        }
        double t = numer / denom;
        t = std::max(0.0, std::min(t, 1.0));

        // Closest point on the segment
        double cx = seg.x0 + t*dx, cy = seg.y0 + t*dy;
        dx = x - cx; dy = y - cy;
        return std::sqrt(dx*dx + dy*dy);
    }

    bool use_center_;
};


#endif // BOX_CHAIN_