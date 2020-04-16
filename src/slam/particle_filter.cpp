#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/range.hpp>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    float sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.01);
    for (auto&p : posterior_) {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
    posterior_.back().pose = pose;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior); //action
        posterior_ = computeNormalizedPosterior(proposal, laser, map); //sensor
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    std::vector<particle_t> prior;
    float inv_M = 1.0 / kNumParticles_;
    float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/ inv_M));
    float c = posterior_[0].weight;
    int i = 0;
    for (int m = 0; m < kNumParticles_; m ++) {
        float U = r + m * inv_M;
        while (U > c) {
            i += 1;
            if (i == posterior_.size()) {
                i -= 1;
                break;
            }
            c += posterior_[i].weight;
        } 
        prior.push_back(posterior_[i]);
    }
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for (auto& p : prior) {
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double norm_factor = 0.0;
    for (auto& prop : proposal) {
        double likelihood = sensorModel_.likelihood(prop, laser, map);
        particle_t p = prop;
        p.weight = likelihood;
        norm_factor += p.weight;
        posterior.push_back(p);
    }
    for (auto& post : posterior) {
        post.weight /= norm_factor;
        // std::cout<< post.weight<<std::endl;
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // double max_weight = 0.0;
    pose_xyt_t pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    float cartesian_x = 0.0;
    float cartesian_y = 0.0;
    for (auto& p : posterior) {
        pose.x += p.weight * p.pose.x;
        pose.y += p.weight * p.pose.y;
        cartesian_x += p.weight * std::cos(p.pose.theta);
        cartesian_y += p.weight * std::sin(p.pose.theta);
        // if (p.weight >= max_weight) {
        //     max_weight = p.weight;
        //     pose = p.pose;
        // }
    }
    pose.theta = atan2(cartesian_y, cartesian_x);
    return pose;
}
