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
    Joystick joystick;
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
            #ifdef _WIN64
            OPD("push A_BUTTON\n");
            #else
            printf("push A_BUTTON\n");
            #endif
            camera->GroundScan();
        }
        PreButtonState = ButtonState;
        
		robot->Control();
        count++;
		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidSampleController)
