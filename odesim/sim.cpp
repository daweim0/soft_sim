#include <iostream>
#include <cmath>
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

std::pair<std::vector<node_type>, std::vector<spring_type>> genTriangle();
std::pair<std::vector<node_type>, std::vector<spring_type>> genCube();
std::pair<std::vector<node_type>, std::vector<spring_type>> genHPC();


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


        unsigned long current_config = count;
        for (std::size_t i = 0; i < mptr->n_groups(); i++)
        {
            if ((current_config & 1 << i) != 0) {
                active_group.push_back(i + 1);
            }

//            if (rand() / (1.0 * RAND_MAX) > 0.75)
//                active_group.push_back(i+1);

        }
        mptr->ApplyControl(active_group);

        if (current_config >= 1 << mptr->n_groups()) {
            exit(0);
        }

        for(int rep = 0; rep  < 200; rep++) {
            auto pre_node_positions = mptr->getPositions();
            // Simulate 10ms
            for (int i = 0; i < 10; ++i) {
                dSpaceCollide(space, 0, &NearCallback);
                mptr->UpdateSpringForces();
                dWorldStep(world, 0.0005);
                dJointGroupEmpty(contactgroup);
//            std::cout << "finished simulation step " << i << std::endl;
            }
            auto post_node_positions = mptr->getPositions();
            auto post_node_vels = mptr->getLinearVel();
            double total_dist = 0;
            for (int i = 0; i < pre_node_positions.size(); i++) {
                total_dist += std::abs((pre_node_positions.at(i) - post_node_positions.at(i)).norm());
                total_dist += std::abs(post_node_vels.at(i).norm()) * 0.05;  // wieght this less than position since position changes much less
            }
            if (total_dist < 0.0001) {
                count++;
                break;
            }
        }
    }
    mptr->Draw(); // draw the arm


    auto node_positions = mptr->getPositions();
    auto node_quaternions = mptr->getQuaternions();

//    std::cout << "active_springs: ";
//    for (int i : active_group) {
//        std::cout << i << " ";
//    }

//    std::cout << "[";
    std::cout << "[" << count << "], ";

    for (int i = 0; i < node_positions.size(); i++) {
        auto node_pos = node_positions.at(i);
        auto node_rot = node_quaternions.at(i);

        if (i != 0)
            std::cout << ", ";

        std::cout << "[" << node_pos(0) << ", " << node_pos(1) << ", " << node_pos(2) << ", ";

        double x = node_rot.x() / sqrt(1-node_rot.w() * node_rot.w());
        double y = node_rot.y() / sqrt(1-node_rot.w() * node_rot.w());
        double z = node_rot.z() / sqrt(1-node_rot.w() * node_rot.w());

        std::cout << x << ", " << y << ", " << z << "]";

    }

    std::cout << "," << std::endl;

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
//    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetDamping(world, 0.1, 0.1);

    space = dHashSpaceCreate (0);
    dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    // Create mesh
    Mesh mesh(world, space);
//    mesh.Init("mesh.xml");


/*
 * For a cube
 */
//    auto input_mesh = genCube();
//    mesh.Init_from_vector(input_mesh.first, input_mesh.second);
//    mptr = &mesh;

/*
 * For a close-packed-hexagonal-something
 */
//    auto input_mesh = genHPC();
//    mesh.Init_from_vector(input_mesh.first, input_mesh.second);
//    mptr = &mesh;


/*
 * For a triangular pyramid
 */
    auto triangle_mesh = genTriangle();
    mesh.Init_from_vector(triangle_mesh.first, triangle_mesh.second);
    mptr = &mesh;


/*
 * Big red ball
 */

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

std::pair<std::vector<node_type>, std::vector<spring_type>> genCube() {
    double x_spacing = 0.035;
    double diagonal_spacing = x_spacing * std::sqrt(2);
    double super_diagonal_spacing = sqrt(x_spacing * x_spacing + diagonal_spacing * diagonal_spacing);


    std::vector<node_type> input_nodes;
    for (int x = 0; x < 2; x++) {
        for (int z = 0; z < 2; z++) {
                for (int y = 0; y < 2; y++){
                bool fixed = x == 0;
                input_nodes.emplace_back(std::make_tuple(x * x_spacing, y * x_spacing, z * x_spacing, fixed));
            }
        }
    }

    std::vector<spring_type> input_springs;

    bool controllable = true;

    // these springs go straight across
    input_springs.emplace_back(spring_type(1, 5, 6000, x_spacing, controllable));
    input_springs.emplace_back(spring_type(2, 6, 6000, x_spacing, controllable));
    input_springs.emplace_back(spring_type(3, 7, 6000, x_spacing, controllable));
    input_springs.emplace_back(spring_type(4, 8, 6000, x_spacing, controllable));


    input_springs.emplace_back(spring_type(1, 6, 6000, 0, controllable));
    input_springs.emplace_back(spring_type(1, 7, 6000, 0, controllable));

    input_springs.emplace_back(spring_type(2, 5, 6000, 0, controllable));
    input_springs.emplace_back(spring_type(2, 8, 6000, 0, controllable));

    input_springs.emplace_back(spring_type(3, 5, 6000, 0, controllable));
    input_springs.emplace_back(spring_type(3, 8, 6000, 0, controllable));

    input_springs.emplace_back(spring_type(4, 6, 6000, 0, controllable));
    input_springs.emplace_back(spring_type(4, 7, 6000, 0, controllable));


    // these springs link the output face
    input_springs.emplace_back(spring_type(5, 6, 6000, x_spacing, false));
    input_springs.emplace_back(spring_type(6, 8, 6000, diagonal_spacing, false));
    input_springs.emplace_back(spring_type(8, 7, 6000, x_spacing, false));
    input_springs.emplace_back(spring_type(7, 5, 6000, diagonal_spacing, false));
    input_springs.emplace_back(spring_type(6, 7, 6000, x_spacing, false));
    input_springs.emplace_back(spring_type(5, 8, 6000, x_spacing, false));

    return std::make_pair(input_nodes, input_springs);
}

// make a close-packed hexagon structure
std::pair<std::vector<node_type>, std::vector<spring_type>> genHPC() {
    double x_spacing = 0.035;

    std::vector<node_type> position_list;
    std::vector<spring_type> input_springs;

    // make the input face
    double layer_1_x = 0;
    position_list.emplace_back(std::make_tuple(layer_1_x, 0, 0, true));
    position_list.emplace_back(std::make_tuple(layer_1_x, x_spacing, 0, true));
    position_list.emplace_back(std::make_tuple(layer_1_x, x_spacing / 2, x_spacing * sqrt(3/4.), true));


    // make the middle layer
    double face_center_z = x_spacing / 2.;
    double face_center_y = tan(30 * 3.14159 / 180) * face_center_z;
    double dist_to_face_center = sqrt(pow(face_center_y , 2) + pow(face_center_z, 2));
    double layer_2_x = sqrt(pow(x_spacing, 2) - pow(dist_to_face_center, 2));

    double hexagon_y_offset = x_spacing / 2;
    double hexagon_z_offset = sqrt(pow(x_spacing, 2) - pow(hexagon_y_offset, 2));

    // center point
    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y, face_center_z, false));
    input_springs.emplace_back(spring_type(1, 4, 6000, 0, true));
    input_springs.emplace_back(spring_type(2, 4, 6000, 0, true));
    input_springs.emplace_back(spring_type(3, 4, 6000, 0, true));

    // surrounding points
    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y - hexagon_y_offset * 2, face_center_z, false));  // horizontal

    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y - hexagon_y_offset, face_center_z - hexagon_z_offset, false));
    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y + hexagon_y_offset, face_center_z - hexagon_z_offset, false));

    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y + hexagon_y_offset * 2, face_center_z, false)); // horizontal

    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y + hexagon_y_offset, face_center_z + hexagon_z_offset, false));
    position_list.emplace_back(std::make_tuple(layer_2_x, face_center_y - hexagon_y_offset, face_center_z + hexagon_z_offset, false));


    // connect the input face to the outer nodes of the output face
    input_springs.emplace_back(spring_type(1, 5, 6000, 0, true));
    input_springs.emplace_back(spring_type(1, 6, 6000, 0, true));
    input_springs.emplace_back(spring_type(2, 7, 6000, 0, true));
    input_springs.emplace_back(spring_type(2, 8, 6000, 0, true));
    input_springs.emplace_back(spring_type(3, 9, 6000, 0, true));
    input_springs.emplace_back(spring_type(3, 10, 6000, 0, true));

    // add some cross bracing between the layers
//    input_springs.emplace_back(spring_type(1, 7, 6000, 0, true));
//    input_springs.emplace_back(spring_type(3, 8, 6000, 0, true));


    // connect the output face to itself
    input_springs.emplace_back(spring_type(4, 5, 6000, 0, false));
    input_springs.emplace_back(spring_type(4, 6, 6000, 0, false));
    input_springs.emplace_back(spring_type(4, 7, 6000, 0, false));
    input_springs.emplace_back(spring_type(4, 8, 6000, 0, false));
    input_springs.emplace_back(spring_type(4, 9, 6000, 0, false));
    input_springs.emplace_back(spring_type(4, 10, 6000, 0, false));

    input_springs.emplace_back(spring_type(5, 6, 6000, 0, false));
    input_springs.emplace_back(spring_type(6, 7, 6000, 0, false));
    input_springs.emplace_back(spring_type(7, 8, 6000, 0, false));
    input_springs.emplace_back(spring_type(8, 9, 6000, 0, false));
    input_springs.emplace_back(spring_type(9, 10, 6000, 0, false));
    input_springs.emplace_back(spring_type(10, 5, 6000, 0, false));


//    // make the output face
//    double layer_3_x = layer_2_x * 2;
//    position_list.emplace_back(std::make_tuple(layer_3_x, 0, 0, false));
//    position_list.emplace_back(std::make_tuple(layer_3_x, x_spacing, 0, false));
//    position_list.emplace_back(std::make_tuple(layer_3_x, x_spacing / 2, x_spacing * sqrt(3/4.), false));
//
//    // connect output face to center point
//    input_springs.emplace_back(spring_type(4, 5, 6000, x_spacing, true));
//    input_springs.emplace_back(spring_type(4, 6, 6000, x_spacing, true));
//    input_springs.emplace_back(spring_type(4, 7, 6000, x_spacing, true));
//    // connect output face
//    input_springs.emplace_back(spring_type(5, 6, 6000, x_spacing, false));
//    input_springs.emplace_back(spring_type(6, 7, 6000, x_spacing, false));
//    input_springs.emplace_back(spring_type(7, 5, 6000, x_spacing, false));




    return std::make_pair(position_list, input_springs);
}

std::pair<std::vector<node_type>, std::vector<spring_type>> genTriangle() {
    double x_spacing = 0.035;


    std::vector<node_type> position_list;
    position_list.emplace_back(std::make_tuple(0, 0, 0, true));
    position_list.emplace_back(std::make_tuple(0, x_spacing * 1, 0, true));
    position_list.emplace_back(std::make_tuple(0, x_spacing * 1./2, x_spacing * sqrt(2)/2, true));
    position_list.emplace_back(std::make_tuple(x_spacing, x_spacing * 1./4, x_spacing * sqrt(2)/4, false));  // this math is extremely approximate


    std::vector<spring_type> input_springs;
    // Nodes 1 through 3 are fixed, so there is no point in adding springs between them
    input_springs.emplace_back(spring_type(1, 4, 6000, x_spacing, true));
    input_springs.emplace_back(spring_type(2, 4, 6000, x_spacing, true));
    input_springs.emplace_back(spring_type(3, 4, 6000, x_spacing, true));

    return std::make_pair(position_list, input_springs);
}

