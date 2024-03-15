#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "myrobot.h"
#include "mycamera.h"

// only when simulate on windows
//#define _WIN64

using namespace cnoid;
using namespace cnoid::vnoid;

class VnoidSampleController : public SimpleController{
public:
	MyRobot*  robot;
    MyCamera* camera;
    FkSolver* fk_solver;
    Joystick joystick;

    vector<Vector3> ground_rectangle;
    bool PreButtonState;
    int count;

public:
    virtual bool configure(SimpleControllerConfig* config){
        return true;
    }

	virtual bool initialize(SimpleControllerIO* io){
        camera = new MyCamera();
        camera->Init(io);
        count = 0;

		robot = new MyRobot();
		robot->Init(io);

		return true;
	}

	virtual bool control()	{
        joystick.readCurrentState();
        bool ButtonState = joystick.getButtonState(Joystick::A_BUTTON);
        if (ButtonState && !PreButtonState) {
            robot->points_convex.clear();
            printf("push A_BUTTON\n");
            camera->GroundScan(robot->points_convex);
            robot->compStairStep = true;
            //ground_rectangle = fk_solver->FootToGroundFK(robot);
            //int i;
            //for(Vector3& p : ground_rectangle){
            //    printf("id%d: %lf, %lf, %lf\n", i, p.x(), p.y(), p.z());
            //    i++;
            //}
        }
        PreButtonState = ButtonState;
        
		robot->Control();
        count++;
		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidSampleController)
