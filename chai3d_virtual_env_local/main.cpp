////==============================================================================
///*
//Software License Agreement (BSD License)
//Copyright (c) 2003-2016, CHAI3D.
//(www.chai3d.org)
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
//* Redistributions of source code must retain the above copyright
//notice, this list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above
//copyright notice, this list of conditions and the following
//disclaimer in the documentation and/or other materials provided
//with the distribution.
//
//* Neither the name of CHAI3D nor the names of its contributors may
//be used to endorse or promote products derived from this software
//without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.
//
//\author    <http://www.chai3d.org>
//\author    Francois Conti
//\version   3.1.0 $Rev: 1869 $
//*/
////===========================================================================
///*
////===========================================================================
//edited for the Computational Haptics Laboratory SS 2010,
//Lehrstuhl fuer Medientechnik, Technische Universitaet Muenchen, Germany
//last revision : 02.07.2010
//last revision for sockets: 21.04.2021
////===========================================================================

//===========================================================================
#include "chai3d.h"
#include "chai3d.h"

#include <boost/asio.hpp>

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

#include <thread>
#include <fstream>
#include <iostream>
#include <map>
#include <utility>
#include <list>
#include <mutex>

//---------------------------------------------------------------------------
#include "CODE.h"
//---------------------------------------------------------------------------
#include "Udp_core/Udp_server.h"
#include "Udp_core/Udp_client.h"
#include "Udp_core/Endpoint_info_and_msg.h"

using namespace chai3d;
using namespace  std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------
// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------


// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W = 512;
const int WINDOW_SIZE_H = 512;

// mouse menu options (right button)
const int OPTION_FULLSCREEN = 1;
const int OPTION_WINDOWDISPLAY = 2;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cDirectionalLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;
// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticD;

// a virtual tool representing the haptic device in the scene
//cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

// maximum stiffness to be used with virtual objects in scene
double stiffnessMax;

// status of the main simulation haptics loop
bool simulationRunning = false;

// ODE world
cODEWorld* ODEWorld;

// ODE object for cube world
/*
cODEGenericBody* ODEBody0=nullptr;
cODEGenericBody* ODEBody1=nullptr;
cODEGenericBody* ODEBody2=nullptr;

cODEGenericBody* ODEGPlane0=nullptr;
cODEGenericBody* ODEGPlane1=nullptr;
cODEGenericBody* ODEGPlane2=nullptr;
cODEGenericBody* ODEGPlane3=nullptr;
cODEGenericBody* ODEGPlane4=nullptr;
cODEGenericBody* ODEGPlane5=nullptr;
*/

// object for 1d demo world
cMesh *box=nullptr;
cShapeSphere* sphere0=nullptr;
cShapeSphere* sphere1=nullptr;
cShapeLine* demoWorldLine=nullptr;
cShapeCylinder* cylinder=nullptr;
cLevel* demoWorldLevel=nullptr;
cScope* demoWorldScope=nullptr;


//Label
cFontPtr font = NEW_CFONTCALIBRI20();
cLabel* newLabel=nullptr;

//Cursor
cShapeSphere* newCursor=nullptr;
cToolCursor* tool=nullptr;

// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool simulationFinished = false;

cVector3d CubeCenter;
int modelparasize;
cVector3d LeaderPos;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(Udp_server &my_server,bool &is_simulation_running);

// main haptics loop
void UDPServerThread(Udp_server &my_server,bool &is_simulation_running);

// creates a cube mesh
void createCube(cMesh* a_mesh, double a_size);
void createCube(cMesh* a_mesh,double x,double y,double z);

//the root world is set as a global variable
//for the convenience of the callback function of GLUT
//this should be optimized for large object, but it is okay in small objects like this
void createCubeWorld();
void create1DDigitalTwinWorld();
void create1DDigitalTwinWorldV2();





struct ClientStruct {
    boost::asio::ip::udp::endpoint client_endpoint;		//IP and port of the socket that sent the data
    boost::asio::ip::udp::endpoint answer_endpoint;		//IP and port of the socket to which data should be sent
    cVector3d Position;

    // force update to be transmitted
    cVector3d Force;

    // corresponding timestamp
    double time_stamp;

    cVector3d Velocity;
    cToolCursor* tool;
    cShapeSphere* cursor;

    cVector3d CubeCen;
    bool isRemote = false ;
    cVector3d LeaderPosition;
    // a haptic device handler
    cHapticDeviceHandler* handler;
    cGenericHapticDevicePtr hapticDevice;

    double UpdateTime;
};

//pointer to client structure
vector<ClientStruct*> Clients;
ClientStruct* cClient;
mutex cClient_lock_guard;

char hostname[NI_MAXHOST];
char servInfo[NI_MAXSERV];


// switch coding method
//bool enable_db_coding = false;	// linear predictor runs by default


// perceptual deadband coding
// cVector3d ch_perceptualDBcoding(const cVector3d& rawForce, const double& db_parameter, bool& transmit);
// bool firstTimeHere = true;


// linear predictor
//cVector3d ch_perceptualLinearPred(const cVector3d& rawForce, const double& db_parameter, bool& transmit, double& update_timestamp);
//unsigned int initial_samples_marker = 0;

int string_split(string input,vector<string> & output,string &&delimiter);
//===========================================================================
/*
DEMO:    ODE_cubic.cpp

This example illustrates the use of the ODE framework for simulating
haptic interaction with dynamic bodies. In this scene we create 3
cubic meshes that we individually attach to ODE bodies. Haptic interactions
are computer by using the finger-proxy haptic model and forces are
propagated to the ODE representation.
*/
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf("\n");
    printf("-----------------------------------\n");
    printf("CHAI 3D\n");
    printf("Demo: 40-ODE-cubic\n");
    printf("Copyright 2003-2009\n");
    printf("-----------------------------------\n");
    printf("\n\n");
    printf("Instructions:\n\n");
    printf("- Use haptic device and user switch to manipulate cubes \n");
    printf("\n\n");
    printf("Keyboard Options:\n\n");
    printf("[1] - Enable gravity\n");
    printf("[2] - Disable gravity\n");
    printf("[3] - Perceptual deadband coding\n");
    printf("[4] - Linear predictor\n");
    printf("[x] - Exit application\n");
    printf("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

    proxyRadius = 0.03;

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CH lab - Haptic UDP Server");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }
    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(0.0, 0.0, 0.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and oriente the camera
    camera->set(cVector3d(9.0, 0.0, 3.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 30.0);

    // create a light source and attach it to the camera
    light = new cDirectionalLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setLocalPos(cVector3d(2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam
    light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);



    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    stiffnessMax = 10;//info.m_maxForceStiffness / workspaceScaleFactor;

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
    world->addChild(graspLine);
    graspLine->m_colorPointA.set(1.0, 1.0, 1.0);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    //createCubeWorld();
    //create1DDigitalTwinWorld();
    create1DDigitalTwinWorldV2();

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create thread for haptic rendering and virtual env. server
    Udp_server my_server(12306);

    thread udpServerThread(UDPServerThread,std::ref(my_server),std::ref(simulationRunning));
    udpServerThread.detach();

    thread hapticRenderThread(updateHaptics,std::ref(my_server),std::ref(simulationRunning));
    hapticRenderThread.detach();



    // old type CHAI3D threads
    /*
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    cThread* serverThread = new cThread();
    serverThread->start(UDPServerThread, CTHREAD_PRIORITY_GRAPHICS);
    */

    // start the main graphics rendering loop
    // the main thread ends here and will only respond with callback functions
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

/////////////////////////////////////////////////////////// world creation ///////////////////////////////////////////////////////
// this is the current world creation function in use
void create1DDigitalTwinWorldV2(){
    cMesh* ground_mesh=new cMesh;

    int ground_vertices[4];

    double ground_level=-3;

    double ground_size=3;

    /////////////////////////////////////////// ground ///////////////////////////////////////////////////////
    ground_vertices[0] = ground_mesh->newVertex(-ground_size, ground_size, -0.0);
    ground_vertices[1] = ground_mesh->newVertex(-ground_size, -ground_size, -0.0);
    ground_vertices[2] = ground_mesh->newVertex(ground_size, -ground_size, -0.0);
    ground_vertices[3] = ground_mesh->newVertex(ground_size, ground_size, -0.0);

    ground_mesh->newTriangle(ground_vertices[0], ground_vertices[1], ground_vertices[2]);
    ground_mesh->newTriangle(ground_vertices[0], ground_vertices[2], ground_vertices[3]);

    ground_mesh->m_material->setBlueAqua();
    ground_mesh->setLocalPos(0.0,0.0,ground_level);

    world->addChild(ground_mesh);
    /////////////////////////////////////////// box ///////////////////////////////////////////////////////
    //replace the cylinder with A box
    box= new cMesh();
    createCube(box,2.5,2.0,2.5);

    cMaterial box_material;
    box_material.m_ambient.set(0.8, 0.1, 0.4);
    box_material.m_diffuse.set(1.0, 0.15, 0.5);
    box_material.m_specular.set(1.0, 0.2, 0.8);
    box_material.setStiffness(0.5 * stiffnessMax);
    box_material.setDynamicFriction(0.8);
    box_material.setStaticFriction(0.8);
    box->setMaterial(box_material);



    world->addChild(box);

    box->setLocalPos(-0.7,0.0,ground_level+1.25);
    //potential rotation HERE
    box->computeBoundaryBox(true);
    box->createAABBCollisionDetector(true);

    box->m_material->setYellowDarkKhaki();

    box->createEffectSurface();



    /////////////////////////////////////////// GUI //////////////////////////////////////////////////////////

    demoWorldLevel = new cLevel();
    camera->m_frontLayer->addChild(demoWorldLevel);
    demoWorldLevel->rotateWidgetDeg(-90);
    demoWorldLevel->setRange(-0.5, 0.6);
    demoWorldLevel->setWidth(40);
    demoWorldLevel->setNumIncrements(100);
    demoWorldLevel->setSingleIncrementDisplay(true);
    demoWorldLevel->setTransparencyLevel(0.5);

    // create a scope to plot haptic device position data
    demoWorldScope = new cScope();
    camera->m_frontLayer->addChild(demoWorldScope);
    demoWorldScope->setSize(400, 100);
    demoWorldScope->setRange(0.0, 5.0);
    demoWorldScope->setSignalEnabled(true, false, false, false);
    demoWorldScope->setShowPanel(false);
    demoWorldScope->m_colorSignal0.setRedCrimson();

}

//////////////////////////////////////////// GLUT and CHAI3D Utility Functions /////////////////////////////////////////


void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    // option 1:
    if (key == '1')
    {
        // enable gravity
        if(ODEWorld)
            ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
        printf("Gravity ON \n");
    }

    // option 2:
    if (key == '2')
    {
        // disable gravity
        if(ODEWorld)
            ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        printf("Gravity OFF \n");
    }

}




void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    //  tool->stop();
}

//////////////////////////////////////////// Thread Tasks //////////////////////////////////////////////////////////////
// this is a standard GLUT call back which renders a frame of the application
void updateGraphics(void)
{
    // for every connected client, create a label for them
    vector<ClientStruct*>::iterator iter;
    for (iter = Clients.begin(); iter != Clients.end(); iter++) {

        // label for the client tool in the shared VE
        newLabel->setText(hostname);
        newLabel->setLocalPos((newCursor->getGlobalPos().y()) * 500 + displayW / 2, (newCursor->getGlobalPos().z()) * 500 + displayH / 2, (newCursor->getGlobalPos().x()));

    }

    // render world
    camera->renderView(displayW, displayH);

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

// haptics updating thread task
// this task should not be blocked by communication and other IO waiting, and should be executed at around 1kHz
void updateHaptics(Udp_server &my_server, bool &is_simulation_running)
{

    vector<ClientStruct*>::iterator it;

    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    int seq=0;
    // main haptic simulation loop
    while (simulationRunning)
    {
        // sequence number for UDP packet ordering
        if(seq<=1000000)
            seq++;
        else
            seq=0;

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // lock the shared object - client list, as it is updated by server thread
        cClient_lock_guard.lock();
        // for every connected thread, calculate the position of the cursor and prepare the feedback message
        for (it = Clients.begin(); it != Clients.end(); it++) {
            cClient = (*it);

            // the tool belonging to the current client
            cToolCursor* tool = cClient->tool;

            // read the remote "device" position
            tool->setDeviceGlobalPos(35 * (cClient->Position)); //WorkspaceScaleFactor = 35;

            // compute interaction forces for this tool
            tool->computeInteractionForces();

            // update the position of the cursor belonging to the client
            cClient->cursor->setLocalPos(tool->getDeviceGlobalPos());
            cClient->cursor->setShowEnabled(false);
            cClient->cursor->setRadius(0.8);
            cClient->tool->setRadius(0.8);

            // calculate the feedback force
            cGenericObject* object = tool->m_hapticPoint->m_algorithmFingerProxy->m_collisionEvents[0]->m_object;
            if (object != nullptr)
            {
                // check if object is attached to an external ODE parent
                // for now we do not use the ODE, so the following code has no effect
                cGenericType* externalParent = object->getParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != nullptr)
                {
                    // get position of tool
                    cVector3d pos = tool->m_hapticPoint->m_algorithmFingerProxy->m_collisionEvents[0]->m_globalPos;

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = tool->getDeviceGlobalForce();

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    ODEobject->addExternalForceAtPoint(tmpfrc, pos);

                }

            }

            // DB parameter
            double k = 0.2;

            // should this force packet be transmitted?
            bool transmit = true;



            ///////////////////////////////////// Manually update env. /////////////////////////////////////////////////////////////
            simClock.stop();
            double timeInterval = simClock.getCurrentTimeSeconds();

            simClock.reset();
            simClock.start();

            // update the position of the box
            if(tool->isInContact(box)){
                double pushing_force=tool->getDeviceGlobalForce().y();

                const double pushing_friction_k=0.5;
//                double pos_y=box->getLocalPos().y();
                double pos_y = CubeCenter.y();

                //update position on y axis
                pos_y= cClamp(pos_y-pushing_friction_k*timeInterval*pushing_force,-0.5,0.5);
//                cout<<"CubeCenter:"<<pos_y<<endl;

                box->setLocalPos(CubeCenter.x() ,pos_y,CubeCenter.z());


//                box->setLocalPos(box->getLocalPos().x() ,pos_y,box->getLocalPos().z());

            }

            ///////////////////////////////////////////// send reaction force ///////////////////////////////////////////////////////////
            cClient->Force = tool->getDeviceGlobalForce();
            if(!tool->isInContact(box))
                cClient->Force.set(0.0,0.0,0.0);

            if (transmit){
                if (cClient->isRemote == true){

                    // assemble msg

                    ostringstream msg;
                    msg<<"seq:"<<seq<<"\n";
                    msg<<"timestamp:"<<cClient->time_stamp<<"\n";
                    msg<<"position"<<cClient->Position.x()<<","<<cClient->Position.y()<<","<<cClient->Position.z()<<"\n";

                    // add the msg to the server sending request queue
                    my_server.add_snd_request_to_one_client(Endpoint_info_and_msg(cClient->answer_endpoint,msg.str()));



                }
                else{
                    ostringstream msg;
                    msg<<"seq:"<<seq<<"\n";
                    msg<<"timestamp:"<<cClient->time_stamp<<"\n";
                    msg<<"force:"<<cClient->Force.x()<<","<<cClient->Force.y()<<","<<cClient->Force.z()<<"\n";

                    // add the msg to the server sending request queue
                    my_server.add_snd_request_to_one_client(Endpoint_info_and_msg(cClient->answer_endpoint,msg.str()));
                }



            }
                // debug code
            else{
                int stop_here = 1;
            }




        }
        // unlock the client list
        cClient_lock_guard.unlock();

        // the main loop runs at at most 1kHz
        this_thread::sleep_for(chrono::milliseconds(1));


        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.00001, 0.001);

        // update simulation
        if(ODEWorld)
            ODEWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

// this thread cooperates with the server, parsing the received msg and create new client structure for every newly connected
void UDPServerThread(Udp_server &my_server,bool &is_simulation_running) {
    cout << "UDP thread start..." << endl;

    // start the cycle clock
    cPrecisionClock netClock;
    netClock.start(true);

    while(is_simulation_running){

        // take out new msg received by the server
        list<Endpoint_info_and_msg> new_msg_with_comm_info_list;
        my_server.get_msgs_from_all_client(new_msg_with_comm_info_list);

        // if no new msg is found, skip this cycle
        if(new_msg_with_comm_info_list.empty())
        {
            this_thread::yield();
            continue;
        }

        for(auto &it:new_msg_with_comm_info_list)
        {
            // get msg and endpoint information
            string msg=it.msg;
            bool is_new_client=true;
            Asio_endpoint &cur_endpoint=it.endpoint;

            // find if the client has be registered or not
            vector<ClientStruct*> ::iterator iter;
            for ( iter=Clients.begin();iter!=Clients.end();iter++){
                if((*iter)->answer_endpoint == cur_endpoint){
                    is_new_client = false;
                    break;
                }
            }
            // for new client, create new cursor in the virtual env.
            if(is_new_client){
                cout << "New Client " << cur_endpoint.address().to_string()<<":"<<cur_endpoint.port() << endl;
                //Create a new client

                ClientStruct *cClient = new ClientStruct();

                cClient->client_endpoint = cur_endpoint;
                cClient->answer_endpoint = cur_endpoint;

                cClient->Force *= 0;
                cClient->Position *= 0;
                cClient->Velocity *= 0;
                cClient->UpdateTime = netClock.getCurrentTimeSeconds();

                //the world is also shared between the thread so it is also possible to be raced
                //since the lock shall be competed by 2 threads only when a new client come
                //this lock will not be competed very often
                cClient_lock_guard.lock();
                // create a 3D tool and add it to the world
                tool = new cToolCursor(world);
                tool->setShowEnabled(true);

                world->addChild(tool);
                tool->setWorkspaceRadius(1.3);

                // define a radius for the tool (graphical display)
                tool->setRadius(proxyRadius);

                // hide the device sphere. only show proxy.
                tool->m_hapticPoint->m_sphereGoal->setShowEnabled(true);
                tool->m_hapticPoint->m_sphereProxy->setShowEnabled(true);
                tool->setWorkspaceRadius(1.3);
                tool->setWorkspaceScaleFactor(35);	// workspace scale factor for the phantom omni device

                // set the physical readius of the proxy.
                tool->m_hapticPoint->m_algorithmFingerProxy->setProxyRadius(proxyRadius);
                tool->m_hapticPoint->m_algorithmFingerProxy->m_collisionSettings.m_checkForNearestCollisionOnly = true;
                tool->m_hapticPoint->m_algorithmFingerProxy->m_useDynamicProxy = true;


                cClient->tool = tool;

                //HapticDevice
                /*
                handler = new cHapticDeviceHandler();
                // get access to the first available haptic device found
                handler->getDevice(hapticD, 0);
                // retrieve information about the current haptic device
                cHapticDeviceInfo hapticDeviceInfo = hapticD->getSpecifications();
                hapticD->setEnableGripperUserSwitch(true);
                cClient->hapticDevice = hapticD;
                */

                newCursor = new cShapeSphere(proxyRadius);
                world->addChild(newCursor);
                cClient->cursor = newCursor;

                // label for the client tool in the shared VE

                newLabel = new cLabel(font);
                camera->m_frontLayer->addChild(newLabel);
                newLabel->m_fontColor.set(1.0, 1.0, 1.0);


                Clients.push_back(cClient);
                cClient_lock_guard.unlock();
                cout<<"new client cursor init done."<<endl;
            } else {
                //Update the client's data

                try{
                    ClientStruct *curClient = *iter;

                    //parsing the msg
                    vector<string> tokens;
//                    cout<<"msg"<<msg<<endl;
                    string_split(msg,tokens,"\n");
                    string dataline=tokens[2];

                    string modelpara = tokens[3];
//                    cout<<"modelpara:"<<modelpara<<endl;
//                    cout<<"modelpara:"<<modelpara.size()<<endl;

                    modelparasize = modelpara.size();

                    size_t pos=0;
                    vector<string> data;
                    if((pos=dataline.find(':'))!=string::npos)
                    {
                        dataline.erase(0,pos+1);
                        string_split(dataline,data,",");
                    }

                    if (modelparasize ==0) {
                        cVector3d newPosition(stod(data[0]), stod(data[1]), stod(data[2]));
                        curClient->Position = newPosition;
                        LeaderPos = newPosition;
                    }
                    else{
                        curClient->Position = LeaderPos;
                        size_t pos1 = 0;
                        vector<string> MP;
                        if ((pos1 = modelpara.find(':')) != string::npos) {
                            modelpara.erase(0, pos1 + 1);
                            string_split(modelpara, MP, ",");
                        }

                        cVector3d newCenter(stod(MP[0]), stod(MP[1]), stod(MP[2]));
//                        curClient->CubeCen = newCenter;
                        curClient->isRemote = true;
                        CubeCenter = newCenter;
                    }



                    double now = netClock.getCurrentTimeSeconds();
                    curClient->UpdateTime = now;
                    //render data
                }catch (exception e){cerr<<"parse error: "<<e.what()<<endl;}
            }
        }

    }
}
////////////////////////////////////////////  END: Thread Tasks //////////////////////////////////////////////////////////////


void createCube(cMesh* a_mesh, double x,double y,double z){
    const double half_x = x / 2.0;
    const double half_y=y/2.0;
    const double half_z=z/2.0;

    int vertices[6][6];

    // face -x
    vertices[0][0] = a_mesh->newVertex(-half_x, half_y, -half_z);
    vertices[0][1] = a_mesh->newVertex(-half_x, -half_y, -half_z);
    vertices[0][2] = a_mesh->newVertex(-half_x, -half_y, half_z);
    vertices[0][3] = a_mesh->newVertex(-half_x, half_y, half_z);

    // face +x
    vertices[1][0] = a_mesh->newVertex(half_x, -half_y, -half_z);
    vertices[1][1] = a_mesh->newVertex(half_x, half_y, -half_z);
    vertices[1][2] = a_mesh->newVertex(half_x, half_y, half_z);
    vertices[1][3] = a_mesh->newVertex(half_x, -half_y, half_z);

    // face -y
    vertices[2][0] = a_mesh->newVertex(-half_x, -half_y, -half_z);
    vertices[2][1] = a_mesh->newVertex(half_x, -half_y, -half_z);
    vertices[2][2] = a_mesh->newVertex(half_x, -half_y, half_z);
    vertices[2][3] = a_mesh->newVertex(-half_x, -half_y, half_z);

    // face +y
    vertices[3][0] = a_mesh->newVertex(half_x, half_y, -half_z);
    vertices[3][1] = a_mesh->newVertex(-half_x, half_y, -half_z);
    vertices[3][2] = a_mesh->newVertex(-half_x, half_y, half_z);
    vertices[3][3] = a_mesh->newVertex(half_x, half_y, half_z);

    // face -z
    vertices[4][0] = a_mesh->newVertex(-half_x, -half_y, -half_z);
    vertices[4][1] = a_mesh->newVertex(-half_x, half_y, -half_z);
    vertices[4][2] = a_mesh->newVertex(half_x, half_y, -half_z);
    vertices[4][3] = a_mesh->newVertex(half_x, -half_y, -half_z);

    // face +z
    vertices[5][0] = a_mesh->newVertex(half_x, -half_y, half_z);
    vertices[5][1] = a_mesh->newVertex(half_x, half_y, half_z);
    vertices[5][2] = a_mesh->newVertex(-half_x, half_y, half_z);
    vertices[5][3] = a_mesh->newVertex(-half_x, -half_y, half_z);

    // create triangles
    for (int i = 0; i<6; i++)
    {
        a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
        a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material->m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material->m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material->m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material->m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();

    // compute collision detection algorithm
    a_mesh->createAABBCollisionDetector(1.01 * proxyRadius);
}

void createCube(cMesh* a_mesh, double a_size){
    const double HALFSIZE = a_size / 2.0;
    int vertices[6][6];

    // face -x
    vertices[0][0] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, -HALFSIZE);
    vertices[0][1] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[0][2] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, HALFSIZE);
    vertices[0][3] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, HALFSIZE);

    // face +x
    vertices[1][0] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[1][1] = a_mesh->newVertex(HALFSIZE, HALFSIZE, -HALFSIZE);
    vertices[1][2] = a_mesh->newVertex(HALFSIZE, HALFSIZE, HALFSIZE);
    vertices[1][3] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, HALFSIZE);

    // face -y
    vertices[2][0] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[2][1] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[2][2] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, HALFSIZE);
    vertices[2][3] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, HALFSIZE);

    // face +y
    vertices[3][0] = a_mesh->newVertex(HALFSIZE, HALFSIZE, -HALFSIZE);
    vertices[3][1] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, -HALFSIZE);
    vertices[3][2] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, HALFSIZE);
    vertices[3][3] = a_mesh->newVertex(HALFSIZE, HALFSIZE, HALFSIZE);

    // face -z
    vertices[4][0] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[4][1] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, -HALFSIZE);
    vertices[4][2] = a_mesh->newVertex(HALFSIZE, HALFSIZE, -HALFSIZE);
    vertices[4][3] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, -HALFSIZE);

    // face +z
    vertices[5][0] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, HALFSIZE);
    vertices[5][1] = a_mesh->newVertex(HALFSIZE, HALFSIZE, HALFSIZE);
    vertices[5][2] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, HALFSIZE);
    vertices[5][3] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, HALFSIZE);

    // create triangles
    for (int i = 0; i<6; i++)
    {
        a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
        a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material->m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material->m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material->m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material->m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();

    // compute collision detection algorithm
    a_mesh->createAABBCollisionDetector(1.01 * proxyRadius);
}


int string_split(string input,vector<string> & output,string &&delimiter)
{
    try{
        size_t pos=0;
        std::string token;
        while((pos=input.find(delimiter))!=string::npos)
        {
            //cout<<"[string split]"<<input<<endl;
            token=input.substr(0,pos);
            output.push_back(token);
            input.erase(0,pos+delimiter.length());
        }
        output.push_back(input);
    }catch(exception e){cout<<"string_split error:"<<e.what()<<endl;}
    //cout<<"split done"<<endl;
    return (int)output.size();
}