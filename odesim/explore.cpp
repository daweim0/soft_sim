#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <cstring>
#include <cstdlib>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "Mesh.hpp"

dWorldID world;
dJointGroupID contactgroup;

void NearCallback (void *data, dGeomID o1, dGeomID o2)
{
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    dContact contact;
    contact.surface.mu = 0.01;
    if (dCollide(o1, o2, 1, &contact.geom, sizeof(dContact)))
    {
        dJointID c = dJointCreateContact(world, contactgroup, &contact);
        dJointAttach(c, b1, b2);
    }
}

int main(int argc, char **argv)
{
    // World setup
    dInitODE();
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetDamping(world, 0.1, 0.1);

    dSpaceID space = dHashSpaceCreate (0);
    dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // Create mesh
    Mesh mesh(world, space);
    mesh.Init("mesh.xml");
    NodeState state;
    mesh.SaveState(state);

    // Create sphere
    const dReal r =  0.030;
    const dReal temp = (dReal) 2 - r - 0.005;
    const dReal spos[3] = {0.045, temp, r};
    dQuaternion q0;
    dQSetIdentity(q0);
    dMass mass;
    dBodyID bSphere = dBodyCreate(world);
    dGeomID geom = dCreateSphere(space, r);
    dMassSetSphereTotal(&mass, 0.01, r);
    dBodySetMass(bSphere, &mass);
    dGeomSetBody(geom, bSphere);
    dBodySetPosition(bSphere, spos[0], spos[1], spos[2]);

    for (int i = 0; i < 2; ++i)
    {
        // Run simulation
        for (int j = 0; j < 5000; ++j)
        {
            // Randomly choose a control
            if ((j % 500) == 0)
            {
                int r = rand() % 4;
                std::cout << r << ",";

                std::vector<size_t> active;
                for (int k = 0; k < 4; ++k)
                    if ((1 << k) & r)
                        active.push_back(k+1);
                mesh.ApplyControl(active);
            }

            // Advance the simulation
            dSpaceCollide(space, 0, &NearCallback);
            mesh.UpdateSpringForces();
            dWorldStep(world, 0.001);
            dJointGroupEmpty(contactgroup);
        }
        const dReal *pos = dBodyGetPosition(bSphere);
        std::cout << pos[3] << std::endl;

        // Reset simulation
        mesh.RestoreState(state);
        dBodySetPosition(bSphere, spos[0], spos[1], spos[2]);
        dBodySetQuaternion(bSphere, q0);
        dBodySetLinearVel(bSphere, 0, 0, 0);
        dBodySetAngularVel(bSphere, 0, 0, 0);
    }

    // Clean up
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}

