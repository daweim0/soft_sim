#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <cstring>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "Mesh.hpp"

dWorldID world;
dSpaceID space;
dJointGroupID contactgroup;
Mesh *mptr;

dBodyID bSphere;
const dReal r =  0.030;

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

// At simulation start
void SimStart()
{
    static float xyz[3] = {0.3f, 2.0f-0.2f, 0.17f};
    static float hpr[3] = {140.0f, -20.0f, 0.0f};
    dsSetViewpoint(xyz, hpr);
}

// Step the simulation
unsigned long count;
void SimStep(int pause)
{
    std::vector<std::size_t> active_group;
    if (!pause)
    {
//        if (count == 0) {
////            mptr->ApplyControl({1});
//        }
//        if (count == 250) {
//            std::vector<std::size_t> active_group;
//            for (int i = 0; i < 16; i++)
//                active_group.push_back(i + 4);
//            mptr->ApplyControl(active_group);
//
////            mptr->ApplyControl({1,2});
//        }


        unsigned long current_config = count / 3;
        for (std::size_t i = 0; i < 12; i++)
        {
            if ((current_config & 1 << i) != 0) {
                active_group.push_back(i + 1);
            }
        }
        mptr->ApplyControl(active_group);

        if (current_config >= 1 << 12) {
            exit(0);
        }

//        mptr->ApplyControl({1, 2, 3, 4, 5, 6, 7, 8, 9});

        count++;

        // Simulate 10ms
        for (int i = 0; i < 20; ++i)
        {
            dSpaceCollide(space, 0, &NearCallback);
            mptr->UpdateSpringForces();
            dWorldStep(world, 0.0005);
            dJointGroupEmpty(contactgroup);
//            std::cout << "finished simulation step " << i << std::endl;
        }
    }
    mptr->Draw(); // draw the arm

    auto node_positions = mptr->getPositions();


    std::cout << "active_springs: ";
    for (int i : active_group) {
        std::cout << i << " ";
    }

    std::cout << " node positions:";
    for (auto node_pos : node_positions) {
        std::cout << "(" << node_pos(0) << ", " << node_pos(1) << ", " << node_pos(2) << ") ";
    }
    std::cout << std::endl;

//    // draw the sphere
//    const dReal *pos = dBodyGetPosition(bSphere);
//    const dReal *R = dBodyGetRotation(bSphere);
//    dsSetColor(0, 1, 0);
//    dsDrawSphere(pos, R, r);

    // Draw box/anchor for mesh
    dMatrix3 Rbox;
    dRSetIdentity(Rbox);
    dReal origin[3];    
    origin[0] = -0.01;
    origin[1] = 2.01;
    origin[2] = 0.1;
    dReal sides[3];
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.2;
    dsSetColor(0.4, 0.4, 0.4);  // set the color of the "foundation" box
    dsDrawBox(origin, Rbox, sides);
}

//Try looking for dAllocateODEDataForThread in include/ode/odeinit.h.

int main(int argc, char **argv)
{
    // Drawstuff setup
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &SimStart;
    fn.step = &SimStep;
    fn.stop = 0;
    fn.command = 0;
    fn.path_to_textures = "/usr/local/include/drawstuff/textures";
    dsSetSphereQuality(3);

    // World setup
    dInitODE2(dAllocateFlagBasicData);
    world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetDamping(world, 0.1, 0.1);

    space = dHashSpaceCreate (0);
    dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // Create mesh
    Mesh mesh(world, space);
//    mesh.Init("mesh.xml");

    double x_spacing = 0.035;
    double diagonal_spacing = x_spacing * std::sqrt(2);
    double super_diagonal_spacing = std::sqrt(x_spacing * x_spacing + diagonal_spacing * diagonal_spacing);


    std::vector<spring_type> input_springs;
    // Nodes 1 through 4 are fixed, so there is no point in adding springs between them
//    input_springs.push_back(spring_type(1, 2, 6000, x_spacing));
//    input_springs.push_back(spring_type(1, 3, 6000, x_spacing));
//    input_springs.push_back(spring_type(2, 4, 6000, x_spacing));
//    input_springs.push_back(spring_type(3, 4, 6000, x_spacing));

    input_springs.push_back(spring_type(1, 5, 6000, x_spacing));
    input_springs.push_back(spring_type(2, 5, 6000, diagonal_spacing));
    input_springs.push_back(spring_type(3, 5, 6000, diagonal_spacing));
//    input_springs.push_back(spring_type(4, 5, 6000, super_diagonal_spacing));

    input_springs.push_back(spring_type(1, 6, 6000, diagonal_spacing));
    input_springs.push_back(spring_type(2, 6, 6000, x_spacing));
    input_springs.push_back(spring_type(3, 6, 6000, super_diagonal_spacing));
    input_springs.push_back(spring_type(4, 6, 6000, diagonal_spacing));

    input_springs.push_back(spring_type(1, 7, 6000, diagonal_spacing));
//    input_springs.push_back(spring_type(2, 7, 6000, super_diagonal_spacing));
    input_springs.push_back(spring_type(3, 7, 6000, x_spacing));
    input_springs.push_back(spring_type(4, 7, 6000, diagonal_spacing));

//    input_springs.push_back(spring_type(1, 8, 6000, super_diagonal_spacing));
    input_springs.push_back(spring_type(2, 8, 6000, diagonal_spacing));
    input_springs.push_back(spring_type(3, 8, 6000, diagonal_spacing));
    input_springs.push_back(spring_type(4, 8, 6000, x_spacing));

    input_springs.push_back(spring_type(5, 6, 6000, x_spacing));
    input_springs.push_back(spring_type(5, 7, 6000, x_spacing));
    input_springs.push_back(spring_type(6, 8, 6000, x_spacing));
    input_springs.push_back(spring_type(7, 8, 6000, x_spacing));
    input_springs.push_back(spring_type(6, 7, 6000, diagonal_spacing));
    input_springs.push_back(spring_type(5, 8, 6000, diagonal_spacing));


//    input_springs.push_back(spring_type(1, 4, 0.15, 0.04949));
//    input_springs.push_back(spring_type(1, 5, 0.15, 0.035));
//    input_springs.push_back(spring_type(1, 7, 0.15, 0.0699));
    mesh.Init(input_springs);
    mptr = &mesh;


//    dMass mass;
//    bSphere = dBodyCreate(world);
//    dGeomID geom = dCreateSphere(space, r);
//    dMassSetSphereTotal(&mass, 0.01, r);
//    dBodySetMass(bSphere, &mass);
//    dGeomSetBody(geom, bSphere);
//    dBodySetPosition(bSphere, 0.045, 2-r - 0.005, r);

    // Run simulation
    count = 0;
    dsSimulationLoop(argc, argv, 1400, 1000, &fn);

    // Clean up
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}

