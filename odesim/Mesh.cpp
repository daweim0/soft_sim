#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <stdio.h>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <tinyxml.h>

#include "Mesh.hpp"

using namespace Eigen;

static dReal r_node = 0.004;

Mesh::Mesh(dWorldID world, dSpaceID space) : isInitialized(false)
{
    this->world = world;
    this->space = space;
}

void Mesh::Init(std::string filename)
{
    if (!this->isInitialized)
    {
        if (this->InitHelper(filename))
        {
            this->isInitialized = true;
        }
        else // failed to load
        {
            this->nodes.clear();
            this->springs.clear();
        }
    }
}


void Mesh::Init(std::vector<std::tuple<int, int, double, double>> input_springs)
{

    this->isInitialized = true;


//    TiXmlDocument doc(filename);
//    if (!doc.LoadFile()) return false;
//
//    TiXmlElement *root, *elem, *param;
//
//    root = doc.FirstChildElement("mesh");
//    if (root == nullptr) return false;

    // Count number of nodes
    size_t n = 0;
//    TiXmlNode *child;
//    for (child = root->FirstChild("node"); child; child = child->NextSibling("node"))
//    {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++){
            for (int k = 0; k < 2; k++) {
                ++n;
                this->nodes.push_back(NULL);
            }

        }
    }
//    }

    std::vector<dReal> ms_node(n);
    std::vector<dReal> ks_node(n);

//    elem = root->FirstChildElement("node");

//    while (elem)
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                size_t idx;
                float k_node, m_node;
                Vector3f pos;

                // Extract node info
//                if (sscanf(elem->Attribute("idx"), "%lu", &idx) < 1) return false;
                idx = x * 4 + y * 2 + z + 1;
//                 if (sscanf(elem->Attribute("k"), "%f", &k_node) < 1) return false;
                k_node = 0.15;
//                if (sscanf(elem->Attribute("m"), "%f", &m_node) < 1) return false;
                m_node = 0.002;
//                if ((param = elem->FirstChildElement("x")) == nullptr) return false;
//                if (sscanf(param->GetText(), "%f %f %f", &pos[0], &pos[1], &pos[2]) < 3) return false;
                pos[0] = 0.035 * x;
                pos[1] = 0.035 * y;
                pos[2] = 0.035 * z;

                pos[1] = pos[1] + 2; // shift all nodes +y

                // Create ODE body
                dBodyID body = dBodyCreate(this->world);
                dGeomID geom = dCreateSphere(this->space, r_node);
                dGeomSetBody(geom, body);
                dBodySetPosition(body, pos[0], pos[1], pos[2] + 0.04);
                this->nodes.at(idx - 1) = body;

                // Anchor static nodes
//                if (elem->FirstChildElement("static") != nullptr) {
                if (x == 0) {
                    dJointID anchor = dJointCreateFixed(this->world, 0);
                    dJointAttach(anchor, NULL, body);
                    dJointSetFixed(anchor);
                }

                // Store parameters
                ms_node.at(idx - 1) = m_node;
                ks_node.at(idx - 1) = k_node;

//                elem = elem->NextSiblingElement("node");
            }
        }
    }

//    elem = root->FirstChildElement("spring");
    auto set_iter = input_springs.begin();
    for (; set_iter != input_springs.end(); set_iter++)
    {
        size_t node1, node2;
        float k_spring, m_spring, l0;
        Vector3f rij0, rji0;

//        // Extract spring info
//        if (sscanf(elem->Attribute("k"), "%f", &k_spring) < 1) return false;
//        if (sscanf(elem->Attribute("m"), "%f", &m_spring) < 1) return false;
//        if ((param = elem->FirstChildElement("node1")) == nullptr) return false;
//        if (sscanf(param->GetText(), "%lu", &node1) < 1) return false;
//        if ((param = elem->FirstChildElement("node2")) == nullptr) return false;
//        if (sscanf(param->GetText(), "%lu", &node2) < 1) return false;
//        if ((param = elem->FirstChildElement("rij0")) == nullptr) return false;
//        if (sscanf(param->GetText(), "%f %f %f", &rij0[0], &rij0[1], &rij0[2]) < 3) return false;
//        if ((param = elem->FirstChildElement("rji0")) == nullptr) return false;
//        if (sscanf(param->GetText(), "%f %f %f", &rji0[0], &rji0[1], &rji0[2]) < 3) return false;
//        if ((param = elem->FirstChildElement("l0")) == nullptr) return false;
//        if (sscanf(param->GetText(), "%f", &l0) < 1) return false;

        auto next_spring = *set_iter;
        node1 = std::get<0>(next_spring);
        node2 = std::get<1>(next_spring);
//        int z = std::get<2>(next_spring);
        k_spring = std::get<2>(next_spring);
        m_spring = 0.01;
        rji0[0] = 0;
        rji0[1] = 1.5708;
        rji0[2] = 0;

        // TODO: actually calculate this
//        l0 = 0.035;
        l0 = std::get<3>(next_spring);


        // Place spring links in their rest orientations
        Matrix3f Rij0;
        Rij0 = AngleAxisf(rij0[0], Vector3f::UnitZ())
               * AngleAxisf(rij0[1], Vector3f::UnitY())
               * AngleAxisf(rij0[2], Vector3f::UnitX());
        Matrix3f Rji0;
        Rji0 = AngleAxisf(rji0[0], Vector3f::UnitZ())
               * AngleAxisf(rji0[1], Vector3f::UnitY())
               * AngleAxisf(rji0[2], Vector3f::UnitX());

        // Divide mass between endpoint nodes
        ms_node.at(node1-1) += m_spring/2.0;
        ms_node.at(node2-1) += m_spring/2.0;

        // Populate spring structure
        std::shared_ptr<SpringInfo> spring = std::make_shared<SpringInfo>();
        spring->body1 = this->nodes.at(node1-1);
        spring->body2 = this->nodes.at(node2-1);
        spring->Rij0 = Rij0;
        spring->Rji0 = Rji0;
        spring->l0 = l0;
        spring->l0_orrigional = l0;
        spring->k = k_spring;
        spring->k1 = ks_node.at(node1-1);
        spring->k2 = ks_node.at(node2-1);
        this->springs.push_back(spring);

        // make spring its own group
        std::vector<std::shared_ptr<SpringInfo> > slist;
        slist.push_back(this->springs.back());
        this->groups.push_back(slist);

    }
    // Set node masses (lumped)
    for (size_t i = 0; i < n; ++i)
    {
        dMass mass;
        dMassSetSphereTotal(&mass, ms_node.at(i), r_node);
        dBodySetMass(this->nodes.at(i), &mass);
    }

//    elem = root->FirstChildElement("group");
//    while (elem)
//    {
//        size_t idx, sid;
//
//        // Extract group info
//        if (sscanf(elem->Attribute("idx"), "%lu", &idx) < 1) return false;
//        if ((param = elem->FirstChildElement("springs")) == nullptr) return false;
//        std::stringstream ss(param->GetText());
//        std::vector<SpringInfo *> slist;
//        while (ss >> sid)
//        {
//            slist.push_back(&(this->springs[sid-1]));
//        }
//        this->groups.push_back(slist);
////        elem = elem->NextSiblingElement("group");
//    }

//    return true;
}


void Mesh::UpdateSpringForces()
{
    if (this->isInitialized)
    {
        for (auto spring : this->springs)
        {
            // Get endpoint positions and orientations
            dBodyID body1 = spring->body1;
            dBodyID body2 = spring->body2;
            Vector3f pos1(dBodyGetPosition(body1));
            Vector3f pos2(dBodyGetPosition(body2));
            const dReal *rot1 = dBodyGetRotation(body1);
            const dReal *rot2 = dBodyGetRotation(body2);
            Matrix3f R1, R2;

            //TODO: what does this loop do?
            for (size_t row = 0; row < 3; ++row)
            {
                for (size_t col = 0; col < 3; ++col)
                {
                    R1(row,col) = rot1[4*row + col];
                    R2(row,col) = rot2[4*row + col];
                }
            }

            // Distance between endpoints
            Vector3f dvec = (pos2 - pos1);
            float d = dvec.norm();
            dvec.normalize();

            // Spring stretch
            float fs = spring->k*(d - spring->l0);
            // apply force to bodies
            dBodyAddForce(body1, fs*dvec[0], fs*dvec[1], fs*dvec[2]);
            dBodyAddForce(body2,-fs*dvec[0],-fs*dvec[1],-fs*dvec[2]);

            // Spring deflection
            // TODO: what is this?
            Vector3f vec;
            float vmag;
            
            vec = R1*spring->Rij0.col(2);
            vec = vec.cross(dvec);
            vmag = vec.norm();
            if (fabs(vmag) < 1e-8)
                vec = Vector3f::Zero();
            else
                vec = (spring->k1 * std::asin(vmag) / vmag) * vec;
            dBodyAddTorque(body1, vec[0], vec[1], vec[2]);
            vec = dvec.cross(vec)/d;
            dBodyAddForce(body2, vec[0], vec[1], vec[2]);
            dBodyAddForce(body1,-vec[0],-vec[1],-vec[2]);

            vec = -R2*spring->Rji0.col(2);
            vec = vec.cross(-dvec);
            vmag = vec.norm();
            if (fabs(vmag) < 1e-8)
                vec = Vector3f::Zero();
            else
                vec = (spring->k2 * std::asin(vmag) / vmag) * vec;
            dBodyAddTorque(body2, vec[0], vec[1], vec[2]);
            vec = -dvec.cross(vec)/d;
            dBodyAddForce(body1, vec[0], vec[1], vec[2]);
            dBodyAddForce(body2,-vec[0],-vec[1],-vec[2]);

            // Spring twist (?)
        }
    }
}


std::vector<Vector3f> Mesh::getPositions() {
    std::vector<Vector3f> node_positions;

    for(int i = 0; i < nodes.size(); i++)
    {
        Vector3f position(dBodyGetPosition(nodes.at(i)));
        node_positions.push_back(position);
    }

    return node_positions;
}


void Mesh::Draw()
{
    if (this->isInitialized)
    {
        dsSetColor(1, 0, 0.2);
        for (auto node : this->nodes)
        {
            // Draw the nodes as spheres
            const dReal *pos = dBodyGetPosition(node);
            const dReal *R = dBodyGetRotation(node);
            dsDrawSphere(pos, R, r_node);
        }
        for (auto spring : this->springs)
        {
            // Get endpoint positions and orientations
            Vector3f pos1(dBodyGetPosition(spring->body1));
            Vector3f pos2(dBodyGetPosition(spring->body2));
            const dReal *rot1 = dBodyGetRotation(spring->body1);
            const dReal *rot2 = dBodyGetRotation(spring->body2);
            Matrix3f R1, R2;
            for (size_t row = 0; row < 3; ++row)
            {
                for (size_t col = 0; col < 3; ++col)
                {
                    R1(row,col) = rot1[4*row + col];
                    R2(row,col) = rot2[4*row + col];
                }
            }

            // Calculate axes of spring frame in world frame
            Vector3f com = (pos1 + pos2)/2.0;
            Vector3f zvec = pos2 - pos1;
            float zmag = zvec.norm();
            zvec.normalize();
            Vector3f xvec = Vector3f::UnitX();
            Vector3f yvec = Vector3f::UnitY();
            if (fabs(xvec.dot(zvec)) < 0.6675)
            {
                xvec = xvec.cross(zvec);
                yvec = zvec.cross(xvec);
            }
            else
            {
                yvec = yvec.cross(zvec);
                xvec = yvec.cross(zvec);
            }
            xvec.normalize();
            yvec.normalize();

            // Use axes to construct orientation matrix
            dReal R[12];
            R[0] = xvec[0];
            R[4] = xvec[1];
            R[8] = xvec[2];
            R[1] = yvec[0];
            R[5] = yvec[1];
            R[9] = yvec[2];
            R[2] = zvec[0];
            R[6] = zvec[1];
            R[10]= zvec[2];
            R[3] = 0.0;
            R[7] = 0.0;
            R[11]= 0.0;

            // Draw the springs as cylinders
            if (spring->k > 6000 || spring->l0 != spring->l0_orrigional)
                dsSetColor(0.72, 0.45, 0.20);
            else
                dsSetColor(1, 1, 1);
            //dsSetColor(0, 0.45, 0.75);
            dsDrawCylinder(com.data(), R, zmag, 0.001);
        }
    }
}

void Mesh::ApplyControl(std::vector<size_t> active)
{
    // Deactivate all springs
//    for (auto &group : this->groups)
    for (int i = 0; i < this->groups.size(); i++)
    {
        std::vector<std::shared_ptr<SpringInfo> > group = this->groups.at(i);
//        for (auto sptr : group)
        for (int j = 0; j < group.size(); j++)
        {
            std::shared_ptr<SpringInfo> sptr = group.at(j);
            sptr->k = 6000;
//            sptr->l0 = 0.035;
            sptr->l0 = sptr->l0_orrigional;
        }
    }

    // Reactivate springs in active groups
    for (const auto &idx : active)
    {
        if (idx < this->groups.size())
        {
            for (auto sptr : this->groups.at(idx-1))
            {
//                sptr->k = 8000;
                sptr->l0 *= 0.9;
            }
        }
    }
}

void Mesh::SaveState(NodeState &state)
{
    state.x.clear();
    state.q.clear();
    state.v.clear();
    state.w.clear();

    for (auto node : this->nodes)
    {
        Vector3f x(dBodyGetPosition(node));
        Vector4f q(dBodyGetQuaternion(node));
        Vector3f v(dBodyGetLinearVel(node));
        Vector3f w(dBodyGetAngularVel(node));
        state.x.push_back(x);
        state.q.push_back(q);
        state.v.push_back(v);
        state.w.push_back(w);
    }
}

void Mesh::RestoreState(const NodeState &state)
{
    dQuaternion q;
    for (std::vector<dBodyID>::size_type i = 0; i != this->nodes.size(); ++i)
    {
        dBodySetPosition(this->nodes.at(i), state.x[i][0], state.x[i][1], state.x[i][2]);
        for (size_t j = 0; j < 4; ++j)
            q[j] = state.q[i][j];
        dBodySetQuaternion(this->nodes.at(i), q);
        dBodySetLinearVel(this->nodes.at(i), state.v[i][0], state.v[i][1], state.v[i][2]);
        dBodySetAngularVel(this->nodes.at(i), state.w[i][0], state.w[i][1], state.w[i][2]);
    }
}

bool Mesh::InitHelper(std::string filename)
{
    TiXmlDocument doc(filename);
    if (!doc.LoadFile()) return false;

    TiXmlElement *root, *elem, *param;

    root = doc.FirstChildElement("mesh");
    if (root == nullptr) return false;

    // Count number of nodes
    size_t n = 0;
    TiXmlNode *child;
    for (child = root->FirstChild("node"); child; child = child->NextSibling("node"))
    {
        ++n;
        this->nodes.push_back(NULL);
    }

    std::vector<dReal> ms_node(n);
    std::vector<dReal> ks_node(n);

    elem = root->FirstChildElement("node");
    while (elem)
    {
        size_t idx;
        float k_node, m_node;
        Vector3f pos;

        // Extract node info
        if (sscanf(elem->Attribute("idx"), "%lu", &idx) < 1) return false;
        if (sscanf(elem->Attribute("k"), "%f", &k_node) < 1) return false;
        if (sscanf(elem->Attribute("m"), "%f", &m_node) < 1) return false;
        if ((param = elem->FirstChildElement("x")) == nullptr) return false;
        if (sscanf(param->GetText(), "%f %f %f", &pos[0], &pos[1], &pos[2]) < 3) return false;

        pos[1] = pos[1] + 2; // shift all nodes +y

        // Create ODE body
        dBodyID body = dBodyCreate(this->world);
        dGeomID geom = dCreateSphere(this->space, r_node);
        dGeomSetBody(geom, body);
        dBodySetPosition(body, pos[0], pos[1], pos[2] + 0.04);
        this->nodes.at(idx-1) = body;

        // Anchor static nodes
        if (elem->FirstChildElement("static") != nullptr)
        {
            dJointID anchor = dJointCreateFixed(this->world, 0);
            dJointAttach(anchor, NULL, body);
            dJointSetFixed(anchor);
        }

        // Store parameters
        ms_node.at(idx-1) = m_node;
        ks_node.at(idx-1) = k_node;

        elem = elem->NextSiblingElement("node");
    }

    elem = root->FirstChildElement("spring");
    while (elem)
    {
        size_t node1, node2;
        float k_spring, m_spring, l0;
        Vector3f rij0, rji0;

        // Extract spring info
        if (sscanf(elem->Attribute("k"), "%f", &k_spring) < 1) return false;
        if (sscanf(elem->Attribute("m"), "%f", &m_spring) < 1) return false;
        if ((param = elem->FirstChildElement("node1")) == nullptr) return false;
        if (sscanf(param->GetText(), "%lu", &node1) < 1) return false;
        if ((param = elem->FirstChildElement("node2")) == nullptr) return false;
        if (sscanf(param->GetText(), "%lu", &node2) < 1) return false;
        if ((param = elem->FirstChildElement("rij0")) == nullptr) return false;
        if (sscanf(param->GetText(), "%f %f %f", &rij0[0], &rij0[1], &rij0[2]) < 3) return false;
        if ((param = elem->FirstChildElement("rji0")) == nullptr) return false;
        if (sscanf(param->GetText(), "%f %f %f", &rji0[0], &rji0[1], &rji0[2]) < 3) return false;
        if ((param = elem->FirstChildElement("l0")) == nullptr) return false;
        if (sscanf(param->GetText(), "%f", &l0) < 1) return false;

        // Place spring links in their rest orientations
        Matrix3f Rij0;
        Rij0 = AngleAxisf(rij0[0], Vector3f::UnitZ())
           * AngleAxisf(rij0[1], Vector3f::UnitY())
           * AngleAxisf(rij0[2], Vector3f::UnitX());
        Matrix3f Rji0;
        Rji0 = AngleAxisf(rji0[0], Vector3f::UnitZ())
           * AngleAxisf(rji0[1], Vector3f::UnitY())
           * AngleAxisf(rji0[2], Vector3f::UnitX());

        // Divide mass between endpoint nodes
        ms_node[node1-1] += m_spring/2.0;
        ms_node[node2-1] += m_spring/2.0;

        // Populate spring structure
        std::shared_ptr<SpringInfo> spring = std::make_shared<SpringInfo>();
        spring->body1 = this->nodes.at(node1-1);
        spring->body2 = this->nodes.at(node2-1);
        spring->Rij0 = Rij0;
        spring->Rji0 = Rji0;
        spring->l0 = l0;
        spring->l0_orrigional = l0;
        spring->k = k_spring;
        spring->k1 = ks_node.at(node1-1);
        spring->k2 = ks_node.at(node2-1);
        this->springs.push_back(spring);

        elem = elem->NextSiblingElement("spring");
    }
    // Set node masses (lumped)
    for (size_t i = 0; i < n; ++i)
    {
        dMass mass;
        dMassSetSphereTotal(&mass, ms_node[i], r_node);
        dBodySetMass(this->nodes.at(i), &mass);
    }

    elem = root->FirstChildElement("group");
    while (elem)
    {
        size_t idx, sid;

        // Extract group info
        if (sscanf(elem->Attribute("idx"), "%lu", &idx) < 1) return false;
        if ((param = elem->FirstChildElement("springs")) == nullptr) return false;
        std::stringstream ss(param->GetText());
        std::vector<std::shared_ptr<SpringInfo> > slist;
        while (ss >> sid)
        {
            slist.push_back(this->springs.at(sid-1));
        }
        this->groups.push_back(slist);
        elem = elem->NextSiblingElement("group");
    }

    return true;
}
