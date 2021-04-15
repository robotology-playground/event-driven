/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           valentina.vasco@iit.it
 *           chiara.bartolozzi@iit.it
 *           massimiliano.iacono@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __vDraw__
#define __vDraw__

#include <event-driven/api.h>
#include <event-driven/all.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace ev {

/**
 * @brief The vDraw class is the base class from which all vDrawers should
 * inherit. It contains the draw and getTag functions which must be overloaded,
 * and the sensor size for reference.
 */
class vDraw {

protected:

    int Xlimit;
    int Ylimit;
    unsigned int display_window;
    unsigned int max_window;
    bool flip;

    cv::Vec3b aqua {151, 174, 6};
    cv::Vec3b violet  {180, 10, 155};
    cv::Vec3b orange {9, 111, 255};
    cv::Vec3b lime   {9, 250, 222};
    cv::Vec3b white{255, 255, 255};
    cv::Vec3b black  {0,   0,   0};
    cv::Vec3b red  {0,   0,   255};

public:

    vDraw() : Xlimit(304), Ylimit(240), flip(false)
    {
        display_window = 0.1*ev::vtsHelper::vtsscaler;
        max_window = 0.5*ev::vtsHelper::vtsscaler;
    }

    virtual ~vDraw() {}

    ///
    /// \brief setLimits sets the maximum possible values of the position of
    /// events that could be drawn (mostly governed by the sensor size).
    /// \param Xlimit is the maximum x value (width)
    /// \param Ylimit is the maximium y value (height)
    ///
    void setRetinaLimits(int Xlimit, int Ylimit)
    {
        this->Xlimit = Xlimit;
        this->Ylimit = Ylimit;
    }

    void setTemporalLimits(unsigned int display_window,
                           unsigned int max_window)
    {
        this->display_window = display_window;
        this->max_window = max_window;
    }

    void setFlip(bool flip)
    {
        this->flip = flip;
    }

    virtual void initialise() {}

    virtual void resetImage(cv::Mat &image)
    {
        if(image.empty())
            image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    ///
    /// \brief draw takes an image and overlays the new visualisation textures
    /// \param canvas is the image which may or may not yet exist
    /// \param eSet is the set of events which could possibly be drawn
    ///
    virtual void draw(cv::Mat &canvas, const ev::vQueue &eSet, int vTime) = 0;

    ///
    /// \brief getTag returns the unique code for this drawing method. The
    /// arguments given on the command line must match this code exactly
    /// \return the tag code
    ///
    virtual std::string getDrawType() = 0;

    virtual std::string getEventType() = 0;


};

class EV_API isoDraw : public vDraw {

protected:

    //angles
    double thetaY;
    double thetaX;
    double CY, SY;
    double CX, SX;

    double ts_to_axis;
    int Zlimit;
    int imagewidth;
    int imageheight;
    int imagexshift;
    int imageyshift;

    //private functions
    inline void pttr(int &x, int &y, int &z) {
        // we want a negative rotation around the y axis (yaw)
        // a positive rotation around the x axis (pitch) (no roll)
        // the z should always be negative values.
        // the points need to be shifted across by negligble amount
        // the points need to be shifted up by (x = max, y = 0, ts = 0 rotation)

        int xmod = x*CY + z*SY + 0.5; // +0.5 rounds rather than floor
        int ymod = y*CX - SX*(-x*SY + z*CY) + 0.5;
        int zmod = y*SX + CX*(-x*SY + z*CY) + 0.5;
        x = xmod; y = ymod; z = zmod;
    }

    //image with warped square drawn
    cv::Mat baseimage;

public:

    void initialise();

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API addressDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API imuDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API rasterDraw : public vDraw {

protected:

    float time_scaler;
    unsigned int num_neurons;

public:

    virtual void initialise()
    {
        Xlimit = 1024;
        Ylimit = 1024;

        num_neurons = 1;
        time_scaler = (double) Xlimit / max_window;
    }

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API cochleaDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API flowDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API interestDraw : public vDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API clusterDraw : public vDraw {

protected:

    std::map<int, ev::event<ev::GaussianAE>> persistance;
    int stagnantCount;

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API isoInterestDraw : public isoDraw {

public:

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};


} //namespace ev::

#endif


