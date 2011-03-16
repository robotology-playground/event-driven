// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License fo
 r more details
 */

/**
 * @file vAlignerThread.cpp
 * @brief Implementation of the thread (see header vaThread.h)
 */

#include <iCub/vAlignerThread.h>
#include <cstring>
#include <cassert>
#include <yarp/math/SVD.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace yarp::math;
using namespace std;

#define THRATE 30
#define SHIFTCONST 100
#define VERTSHIFT 34

/************************************************************************/
bool getCamPrj(const string &configFile, const string &type, Matrix **Prj)
{
    *Prj=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

        if (parType.size())
        {
            if (parType.check("w") && parType.check("h") &&
                parType.check("fx") && parType.check("fy"))
            {
                // we suppose that the center distorsion is already compensated
                double cx = parType.find("w").asDouble() / 2.0;
                double cy = parType.find("h").asDouble() / 2.0;
                double fx = parType.find("fx").asDouble();
                double fy = parType.find("fy").asDouble();

                Matrix K=eye(3,3);
                Matrix Pi=zeros(3,4);

                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy; 
                
                Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                *Prj=new Matrix;
                **Prj=K*Pi;

                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
    }

    return false;
}

/**********************************************************************************/



inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

/**********************************************************************************/

inline void copy_8u_C3R(ImageOf<PixelRgb>* src, ImageOf<PixelRgb>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

/**********************************************************************************/

vAlignerThread::vAlignerThread(string _configFile) : RateThread(THRATE) {
    resized = false;
    count = 0;
    leftDragonImage = 0;
    rightDragonImage = 0;
    leftEventImage = 0;
    rightEventImage = 0;
    shiftValue = 20;
    configFile =  _configFile;
}

vAlignerThread::~vAlignerThread() {
    if(leftDragonImage!=0) {
        delete leftDragonImage;
    }
    if(rightDragonImage!=0) {
        delete leftDragonImage;
    }
    if(tmpMono!=0) {
        delete tmp;
    }
    if(tmp!=0) {
        delete tmp;
    }
    if(leftEventImage!=0) {
        delete leftEventImage;
    }
    if(rightDragonImage!=0) {
        delete leftEventImage;
    }    
}

bool vAlignerThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    leftDragonPort.open(getName("/leftDragon:i").c_str());
    rightDragonPort.open(getName("/rightDragon:i").c_str());
    leftDragonPort.open(getName("/leftDvs:i").c_str());
    rightDragonPort.open(getName("/rightDvs:i").c_str());
    vergencePort.open(getName("/vergence:i").c_str());

    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;

    //initilisation of the torso
    string torsoPort, headPort;
  
    torsoPort = "/" + robotName + "/torso";
    headPort = "/" + robotName + "/head";

    optionsTorso.put("device", "remote_controlboard");
    optionsTorso.put("local", "/localTorso");
    optionsTorso.put("remote", torsoPort.c_str() );

    robotTorso = new PolyDriver(optionsTorso);

    if (!robotTorso->isValid()) {
        printf("Cannot connect to robot torso\n");
    }
    robotTorso->view(encTorso);
    if ( encTorso==NULL) {
        printf("Cannot get interface to robot torso\n");
        robotTorso->close();
    }

    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", "/localhead");
    optionsHead.put("remote", headPort.c_str());

    robotHead = new PolyDriver (optionsHead);

    if (!robotHead->isValid()){
        printf("cannot connect to robot head\n");
    }
    robotHead->view(encHead);
    if (encHead == NULL) {
        printf("cannot get interface to the head\n");
        robotHead->close();
    }

    // initilisation of the reference to the left and right eyes
    rightEye = new iCubEye("right");
    leftEye = new iCubEye("left");

    for (int i = 0; i < 8; i++ ){
        leftEye->releaseLink(i);
        rightEye->releaseLink(i);
    }
    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj=*PrjL;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of left %f %f %f", Prj(0,0),Prj(1,1),Prj(2,2));
    }
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_RIGHT",&PrjR)) {
        Matrix &Prj=*PrjR;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of right %f %f %f", Prj(0,0),Prj(1,1),Prj(2,2));
    }

	chainRightEye=rightEye->asChain();
  	chainLeftEye =leftEye->asChain();

    return true;
}


void vAlignerThread::setRobotname(string str) {
    robotName = str;
    printf("robotname: %s \n", robotName.c_str());
}

void vAlignerThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}


std::string vAlignerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void vAlignerThread::resize(int widthp, int heightp) {
    width = widthp;
    height = heightp;
    leftDragonImage=new ImageOf<PixelRgb>;
    leftDragonImage->resize(width, height);
    rightDragonImage=new ImageOf<PixelRgb>;
    rightDragonImage->resize(width, height);
    leftEventImage = new ImageOf<PixelMono>;
    leftEventImage->resize(width, height);
    rightEventImage = new ImageOf<PixelMono>;
    rightEventImage->resize(width, height);
}

void vAlignerThread::run() {
    count++;
    if(outPort.getOutputCount()) {
        if(leftDragonPort.getInputCount()) {
             tmp = leftDragonPort.read(false);
             if(tmp!=0) {
                 if(!resized) {
                    resized = true;
                    resize(tmp->width(), tmp->height());
                 }
                 else {
                    ImageOf<yarp::sig::PixelRgb>& outputImage=outPort.prepare();
                    outputImage.resize(width, height);
                    copy_8u_C3R(tmp,leftDragonImage);
                 }
             }
        }
        if(rightDragonPort.getInputCount()) {
             tmp = rightDragonPort.read(false);
             if(tmp!=0) {
                 if(!resized) {
                    resized = true;
                    resize(tmp->width(), tmp->height());
                 }
                 else {
                    copy_8u_C3R(tmp,rightDragonImage);
                 }
             }
        }

        if(leftEventPort.getInputCount()) {
             tmpMono = leftEventPort.read(false);
             if(tmpMono!=0) {
                 printf("copying the leftEventImage \n");
                 copy_8u_C1R(tmpMono,leftEventImage);
             }
        }

        if(rightEventPort.getInputCount()) {
             tmpMono = rightEventPort.read(false);
             if(tmpMono!=0) {
                copy_8u_C1R(tmpMono,rightEventImage);
             }
        }
        
        ImageOf<yarp::sig::PixelRgb>& outputImage=outPort.prepare();
        if(resized) {
            Vector angles(3);
            bool b = igaze->getAngles(angles);
            printf("azim %f, elevation %f, vergence %f \n",angles[0],angles[1],angles[2]);
            double vergence = (angles[2] * 3.14) / 180;
            double version = (angles[0] * 3.14) / 180;
            outputImage.resize(width + shiftValue, height + VERTSHIFT);
                 
            shift(shiftValue, *leftEventImage, *leftEventImage, outputImage);
            outPort.write();

        }
    }
}

void vAlignerThread::shift(int shift,ImageOf<PixelMono> leftEvent,
                           ImageOf<PixelMono> rightEvent, ImageOf<PixelRgb>& outImage) {
    unsigned char* pRightDragon = rightDragonImage->getRawImage();
    unsigned char* pLeftDragon = leftDragonImage->getRawImage();
    unsigned char* pRightEvent = rightEventImage->getRawImage();
    unsigned char* pLeftEvent = leftEventImage->getRawImage();
    
    int padding = leftDragonImage->getPadding();
    int paddingOut = outImage.getPadding();
    
    unsigned char* pOutput=outImage.getRawImage();
    double centerX=width + shift;
    double centerY=height;
    if(shift >= 0) {
        int row = 0;
        while (row < height + VERTSHIFT) {
            if(row < VERTSHIFT ){

                int col = 0;
                while(col < width + shift) {
                    if(col < width) {
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                    }
                    else if(col >= width) {
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                            
                    }
                    col++;
                }
                //padding
                pLeftDragon += padding;
                pOutput += paddingOut;
            }
            
            else if((row >= VERTSHIFT)&&(row < height)) {
                int col = 0;

                while(col < width + shift) {
                    //pRight += shift*3;
                    if (col < shift) {
                        //d=sqrt( (row - centerY) * (row - centerY) 
                        //    + (col - centerX) * (col - centerX));
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                        *pOutput++ = (unsigned char) *pLeftDragon *0.5; pLeftDragon++;
                    }
                    else if((col >= shift)&&(col < width)) {
                        //d=sqrt( (row - centerY) * (row - centerY) 
                        //    + (col - centerX) * (col - centerX));
                        if((col > 160 - 60)&&(col < 160 + 60) && (row > 120 -60) && (row<120 +60)) {
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.25 + *pRightDragon *0.25));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.25 + *pRightDragon *0.25));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                        }
                        else {
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++; pRightDragon++; pOutput++;
                            *pOutput=(unsigned char) floor( (*pLeftDragon *0.5 + *pRightDragon *0.5));
                            pLeftDragon++;pRightDragon++;pOutput++;
                        }
                    }
                    else if((col >= width)&&(col < width + shift)){
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                    }
                    col++;
                }
                //padding
                pLeftDragon += padding;
                pRightDragon += padding;
                pOutput += paddingOut;
            }            
            else if ((row >= height)&&(row < height + VERTSHIFT)) {
                int col = 0;

                while(col < width + shift) {
                    if(col < shift ) {
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                        *pOutput++ = (unsigned char) 0;
                    }
                    else {
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                        *pOutput++ = (unsigned char) *pRightDragon *0.5; pRightDragon++;
                    }
                    col++;
                }                
                //padding
                pRightDragon += padding;
                pOutput += paddingOut;
            }
            row ++;
        }
    }
}


void vAlignerThread::interrupt() {
    outPort.interrupt();
    leftEventPort.interrupt();
    rightEventPort.interrupt();
    leftDragonPort.interrupt();
    rightDragonPort.interrupt();
    vergencePort.interrupt();
}

void vAlignerThread::threadRelease() {
    delete clientGazeCtrl;
    outPort.close();
    leftEventPort.close();
    rightEventPort.close();
    leftDragonPort.close();
    rightDragonPort.close();
    vergencePort.close();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------


