
#ifndef _MESH_HPP_
#define _MESH_HPP_

#include <vector>
#include <string>

#include <ode/ode.h>
#include <Eigen/Dense>

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
    dReal k;
    dReal k1;
    dReal k2;
};

class Mesh
{
    std::vector<dBodyID> nodes;
    std::vector<SpringInfo> springs;
    std::vector<std::vector<SpringInfo *> > groups;

    dWorldID world;
    dSpaceID space;
    bool isInitialized;

    public: Mesh(dWorldID world, dSpaceID space);
    public: void Init(std::string filename);
    public: void UpdateSpringForces();
    public: void Draw();
    public: void ApplyControl(std::vector<size_t> active);
    public: void SaveState(NodeState &state);
    public: void RestoreState(const NodeState &state);

    // Parse XML and construct mesh
    private: bool InitHelper(std::string filename);
};

#endif
