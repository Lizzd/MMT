//
// Created by zican on 22.08.22.
//

#include "Falcon_robot.h"

using namespace chai3d;
using namespace std;

Falcon_robot::Falcon_robot(int argc, char* argv[]): I_Robot(3) {

    initialize_glut_interface(argc,argv);
    initialize_chai3d_world(argc,argv);

    cHapticDeviceHandler* hapticDeviceHandler=new cHapticDeviceHandler();
    cout<<"find "<<hapticDeviceHandler->getNumDevices()<<" devices"<<endl;

    hapticDeviceHandler->getDevice(hapticMasterDevice,0);
    hapticMasterDevice->open();
    hapticMasterDevice->calibrate(true);
    cHapticDeviceInfo info = hapticMasterDevice->getSpecifications();
    cout<<info.m_modelName<<":";
    cout<<"open haptic device done"<<endl;

    cout<<"INIT: initializing device, move the device around"<<endl;

    int initSuccessFlag=false;
    cVector3d lastPos;
    hapticMasterDevice->getPosition(lastPos);
    cVector3d currentPos;
    int diffDataCnt=0;
    double eps=0.00001;

    while(!initSuccessFlag)
    {
        hapticMasterDevice->getPosition(currentPos);
        cVector3d diffPos=lastPos-currentPos;
        if(std::abs(diffPos.x())>eps
           || std::abs(diffPos.y())>eps
           || std::abs(diffPos.z())>eps)
        {
            diffDataCnt++;
        }
        if(diffDataCnt>100) initSuccessFlag=true;
    }

    cout<<"initialization done"<<endl;

}

bool Falcon_robot::get_cartesian_position(Eigen::Vector3d &output_position)
{
    cVector3d pos;
    hapticMasterDevice->getPosition(pos);
    output_position= Falcon_robot::chai3d_vec_to_eigen(pos);
    return true;
}

void Falcon_robot::initialize_chai3d_world(int argc, char **argv) {
    //////////////////////////////////////////////////////////////////////////
    // Create a Ground
    //////////////////////////////////////////////////////////////////////////

    // create mesh to model ground surface
    cMesh* ground = new cMesh();
    //world->addChild(ground);

    // create 4 vertices (one at each corner)
    double groundSize = 2.0;
    int vertices0 = ground->newVertex(-groundSize, -groundSize, 0.0);
    int vertices1 = ground->newVertex(groundSize, -groundSize, 0.0);
    int vertices2 = ground->newVertex(groundSize, groundSize, 0.0);
    int vertices3 = ground->newVertex(-groundSize, groundSize, 0.0);

    // compose surface with 2 triangles
    ground->newTriangle(vertices0, vertices1, vertices2);
    ground->newTriangle(vertices0, vertices2, vertices3);

    // compute surface normals
    ground->computeAllNormals();

    // position ground at the right level
    ground->setLocalPos(0.0, 0.0, -1.0);

}

void Falcon_robot::initialize_glut_interface(int argc, char **argv) {

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------
    const int WINDOW_SIZE_W = 512;
    const int WINDOW_SIZE_H = 512;
    cStereoMode stereoMode = C_STEREO_DISABLED;
    // initialize GLUT
    glutInit(&argc, argv);

    glutDisplayFunc(Falcon_robot::updateGraphics);

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
    glutHideWindow();

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif
    world = new cWorld();
}