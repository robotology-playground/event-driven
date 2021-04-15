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

#ifdef WIN32
#define EXPORT //warning, positional define. This should be define before including the vDraw.h
#endif
#include "event-driven/vDraw.h"

namespace ev {

// ISO (AE) //
// ======== //

const std::string isoDraw::drawtype = "ISO";

std::string isoDraw::getDrawType()
{
    return isoDraw::drawtype;
}

std::string isoDraw::getEventType()
{
    return AddressEvent::tag;
}


void isoDraw::initialise()
{
    Zlimit = Xlimit * 3;
    ts_to_axis = (double)Zlimit / max_window;

    thetaX = 20 * 3.14 / 180.0;  //PITCH
    thetaY = 40 * 3.14 / 180.0;  //YAW

    CY = cos(thetaY); SY = sin(thetaY);
    CX = cos(thetaX); SX = sin(thetaX);

    //the following calculations make the assumption of a negative yaw and
    //a positive pitch
    int x, y, z;
    int maxx = 0, maxy = 0, miny = Ylimit, minx = Xlimit;
    for(int xi = 0; xi <= Xlimit; xi+=Xlimit) {
        for(int yi = 0; yi <= Ylimit; yi+=Ylimit) {
            for(int zi = 0; zi <= Zlimit; zi+=Zlimit) {
                x = xi; y = yi; z = zi; pttr(x, y, z);
                maxx = std::max(maxx, x);
                maxy = std::max(maxy, y);
                minx = std::min(minx, x);
                miny = std::min(miny, y);
            }
        }
    }


    imagexshift = -minx + 10;
    imageyshift = -miny + 10;

    imagewidth = maxx + imagexshift + 10;
    imageheight = maxy + imageyshift + 10;

    baseimage = cv::Mat(imageheight, imagewidth, CV_8UC3);
    baseimage.setTo(0);


    //cv::putText(baseimage, std::string("X"), cv::Point(100, 100), 1, 0.5, CV_RGB(0, 0, 0));

    cv::Scalar invertedtextc = CV_RGB(125, 125, 125);
    cv::Vec3b invertedaxisc = cv::Vec3b(255, 255, 255);
    cv::Vec3b invertedframec = cv::Vec3b(125, 125, 125);

    for(int xi = 0; xi < Xlimit; xi++) {
        x = xi; y = 0; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        x = xi; y = Ylimit; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        if(xi == Xlimit / 2) {
            cv::putText(baseimage, std::string("x"), cv::Point(x-10, y+10),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }
    }

    for(int yi = 0; yi <= Ylimit; yi++) {
        x = 0; y = yi; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        if(yi == Ylimit / 2) {
            cv::putText(baseimage, std::string("y"), cv::Point(x-10, y+10),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }
        x = Xlimit; y = yi; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

    }

    unsigned int tsi;
    for(tsi = 0; tsi < (unsigned int)(Zlimit*0.3); tsi++) {

        x = Xlimit; y = Ylimit; z = tsi; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

        if(tsi == (unsigned int)(Zlimit *0.15)) {
            cv::putText(baseimage, std::string("t"), cv::Point(x, y+12),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }

    }

    for(int i = 0; i < 14; i++) {

        x = Xlimit-i/2; y = Ylimit; z = tsi-i; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

        x = Xlimit+i/2; y = Ylimit; z = tsi-i; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
    }

    for(tsi = vtsHelper::vtsscaler / 10.0;
        tsi < max_window;
        tsi += vtsHelper::vtsscaler / 10.0) {

        int zc = tsi * ts_to_axis + 0.5;

        for(int xi = 0; xi < Xlimit; xi++) {
            x = xi; y = 0; z = zc; pttr(x, y, z);
            y += imageyshift; x += imagexshift;
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;
            x = xi; y = Ylimit; z = zc; pttr(x, y, z);
            y += imageyshift; x += imagexshift;
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;
        }

        for(int yi = 0; yi <= Ylimit; yi++) {
            x = 0; y = yi; z = zc; pttr(x, y, z);
            y += imageyshift; x += imagexshift;
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;
            x = Xlimit; y = yi; z = zc; pttr(x, y, z);
            y += imageyshift; x += imagexshift;
            baseimage.at<cv::Vec3b>(y, x) = invertedframec;

        }

    }

    yInfo() << "Finished setting up ISO draw";



}

void isoDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    auto light_violet = (white - (white - violet) * 0.25);
    auto light_aqua = (white - (white - aqua) * 0.25);

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    //if() return;
    if(!eSet.empty() && vTime < 0)
        vTime = eSet.back()->stamp;

    int skip = 1 + eSet.size() / 100000;

    for(int i = eSet.size() - 1; i >= 0; i -= skip) {

        AE *aep = read_as<AE>(eSet[i]);

        //transform values
        int dt = vTime - aep->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > max_window) continue;
        dt = dt * ts_to_axis + 0.5;
        int px = aep->x;
        int py = aep->y;
        if(flip) {
            px = Xlimit - 1 - px;
            py = Ylimit - 1 - py;
        }
        int pz = dt;
        pttr(px, py, pz);
        px += imagexshift;
        py += imageyshift;

        if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight) {
            continue;
        }

        if(!aep->polarity) {
            isoimage.at<cv::Vec3b>(py, px) = light_violet;
        } else {
            isoimage.at<cv::Vec3b>(py, px) = light_aqua;
        }
    }

    if (image.type() == CV_8UC1)
        cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
    if(!image.empty()) {
        for(int y = 0; y < image.rows; y++) {
            for(int x = 0; x < image.cols; x++) {
                cv::Vec3b &pixel = image.at<cv::Vec3b>(y, x);

                if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255) {

                    int px = x, py = y, pz = 0; pttr(px, py, pz);
                    px += imagexshift;
                    py += imageyshift;
                    if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight)
                        continue;

                    isoimage.at<cv::Vec3b>(py, px) = pixel;
                }
            }
        }
    }

    image = isoimage - baseimage;

}

// ISO (LAE) //
// ========= //

const std::string isoInterestDraw::drawtype = "ISO-INT";

std::string isoInterestDraw::getDrawType()
{
    return isoInterestDraw::drawtype;
}

std::string isoInterestDraw::getEventType()
{
    return LabelledAE::tag;
}

void isoInterestDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    if(eSet.empty()) return;
    if(vTime < 0) vTime = eSet.back()->stamp;

    int skip = 1 + eSet.size() / 50000;

    int r = 1;
    cv::Scalar c1 = CV_RGB(255, 0, 0);
    cv::Scalar c2 = CV_RGB(0, 255, 255);

    //ev::vQueue::const_iterator qi;
    //for(qi = eSet.begin(); qi != eSet.end(); qi += skip) {
    for(int i = eSet.size() - 1; i >= 0; i -= skip) {

        LabelledAE *cep = read_as<LabelledAE>(eSet[i]);

        //transform values
        int dt = vTime - cep->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > max_window) continue;
        dt = dt * ts_to_axis + 0.5;
        int px = cep->x;
        int py = cep->y;
        if(flip) {
            px = Xlimit - 1 - px;
            py = Ylimit - 1 - py;
        }
        int pz = dt;
        pttr(px, py, pz);
        px += imagexshift;
        py += imageyshift;

        if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight) {
            continue;
        }

        cv::Point centr(px, py);
        if(cep->ID == 1)
            cv::circle(image, centr, r, c1);
        else
            cv::circle(image, centr, r, c2);
    }

    if(!image.empty()) {
        for(int y = 0; y < image.rows; y++) {
            for(int x = 0; x < image.cols; x++) {
                cv::Vec3b &pixel = image.at<cv::Vec3b>(y, x);

                if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255) {

                    int px = x, py = y;
                    if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight)
                        continue;

                    isoimage.at<cv::Vec3b>(py, px) = pixel;
                }
            }
        }
    }

    image = isoimage - baseimage;

}

} //namespace ev::
