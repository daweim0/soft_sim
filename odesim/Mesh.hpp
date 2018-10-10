
#ifndef _MESH_HPP_
#define _MESH_HPP_

#include <vector>
#include <string>
#include <set>

#include <ode/ode.h>
#include <Eigen/Dense>
#include <memory>


using spring_type = std::tuple<int, int, double, double>;

struct NodeState
{
    std::vector<Eigen::Vector3f> x; // positions
    std::vector<Eigen::Vector4f> q; // orientation quaternions
    std::vector<Eigen::Vector3f> v; // linear velocities
    std::vector<Eigen::Vector3f> w; // angular velocities
};

struct SpringInfo
{
    dBodyID body1;
    dBodyID body2;
    Eigen::Matrix3f Rij0;
    Eigen::Matrix3f Rji0;
    dReal l0;
    dReal l0_orrigional;
    dReal k;
    dReal k1;
    dReal k2;
};

class Mesh
{
    std::vector<dBodyID> nodes;
    std::vector<std::shared_ptr<SpringInfo> > springs;
    std::vector<std::vector<std::shared_ptr<SpringInfo> > > groups;

    dWorldID world;
    dSpaceID space;
    bool isInitialized;

    public: Mesh(dWorldID world, dSpaceID space);
    public: void Init(std::string filename);
    public: void Init(std::vector<spring_type> input_springs);
    public: void UpdateSpringForces();
    public: void Draw();
    public: void ApplyControl(std::vector<size_t> active);
    public: void SaveState(NodeState &state);
    public: void RestoreState(const NodeState &state);
    public: std::vector<Eigen::Vector3f> getPositions();

    // Parse XML and construct mesh
    private: bool InitHelper(std::string filename);
};

#endif
