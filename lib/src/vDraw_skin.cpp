/*
 *   Copyright (C) 2020 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           luca.gagliardi@iit.it
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
#define EXPORT //warning, positional define. This should be define before including the vDrawSkin.h
#endif
#include "event-driven/vDrawSkin.h"
#include "event-driven/vDraw.h"

namespace ev {

// SKIN DRAW //
// ========= //

const std::string skinDraw::drawtype = "SKIN";

std::string skinDraw::getDrawType()
{
    return skinDraw::drawtype;
}

std::string skinDraw::getEventType()
{
    return SkinEvent::tag;
}

void skinDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{


    //int radius = 5;

    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = is_event<SkinEvent>(*qi);

        int index = aep->taxel;


        if(tmap.pos.find(index) != tmap.pos.end()){
            //yInfo() << "\n Index " << index <<"  mapped to   "<< std :: get<0>(tmap[index]) << std :: get<1>(tmap[index]) << "\n";
            int x = tmap.xoffset + tmap.scaling* std :: get<0>(tmap.pos[index]);
            int y =  tmap.yoffset + tmap.scaling* std :: get<1>(tmap.pos[index]);
            cv::Point centr(x, y);
            if(!aep->polarity)
            {
                cv::circle(image, centr, radius, aqua, cv::FILLED, cv::LINE_AA);
            }
            else
            {
                cv::circle(image, centr, radius, violet, cv::FILLED, cv::LINE_AA);
            }
        }
        else
        {
           // yWarning() << "\n Index " << index <<"not mapped! \n";
        }


    }
}

// SKIN SAMPLE DRAW //
// ================ //

const std::string skinsampleDraw::drawtype = "SAMPLE";

std::string skinsampleDraw::getDrawType()
{
    return skinsampleDraw::drawtype;
}

std::string skinsampleDraw::getEventType()
{
    return SkinSample::tag;
}

void skinsampleDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    int radius_min = radius;
    int radius_max = 6*radius;

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;


        auto aep = as_event<SkinSample>(*qi);
        if(!aep) {
            yError() << "data corruption";
            continue;
        }


        int index = aep->taxel;


        if(tmap.pos.find(index) != tmap.pos.end()){
            // yInfo() << "\n Index = " << index ;
            // std :: cout << "\n Index " << index <<"mapped to "<< x << y << "\n";
            if(aep->value > noise){
                int x = tmap.xoffset + tmap.scaling* std :: get<0>(tmap.pos[index]);
                int y =  tmap.yoffset +tmap.scaling* std :: get<1>(tmap.pos[index]);
                cv::Point centr(x, y);

                int radius = radius_min + (radius_max-radius_min)* aep->value /max_value;
                if(radius>radius_max){
                    radius = radius_max;
                }

                cv::circle(image, centr, radius, red, 1, cv::LINE_AA);
            }
        }
        else{
            //yWarning() << "\n Index " << index <<"not mapped! \n";
            //std :: cout << "\n Index " << index <<"not mapped! \n";
        }
    }
}



// SKIN TAXEL DRAW //
// ================ //

const std::string taxelsampleDraw :: drawtype = "TAXEL_SAMPLE";

std::string  taxelsampleDraw :: getDrawType()
{
    return taxelsampleDraw::drawtype;
}

std::string  taxelsampleDraw ::getEventType()
{
    return SkinSample::tag;
}
//--------------------------------



void taxelsampleDraw :: draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    if(eSet.empty()) return;
    ev::vQueue::const_reverse_iterator qi;
    // static float T =0;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > display_window) break;

        auto aep = is_event<SkinSample>(*qi);

        int noise = 2500;

        int index = aep->taxel;

        float sample = aep->value/float(noise)*20;

        if(index ==163){

            cv::Point dot(10 + dt*vtsHelper::tsscaler*500, Ylimit - sample);

            cv::circle(image, dot, 1, red, cv::FILLED);
        }

    }

}

const std::string taxeleventDraw :: drawtype = "TAXEL_EVENT";

std::string  taxeleventDraw :: getDrawType()
{
    return taxeleventDraw::drawtype;
}

std::string  taxeleventDraw ::getEventType()
{
    return SkinEvent::tag;
}
//--------------------------------

void taxeleventDraw :: draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    if(eSet.empty()) return;
    ev::vQueue::const_reverse_iterator qi;

    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        int dt = eSet.back()->stamp - (*qi)->stamp; // start with newest event
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
       // if((unsigned int)dt > display_window) break;


        auto aep = is_event<SkinEvent>(*qi);

        int index = aep->taxel;



        if(index ==163){

            cv::Point dot(10 + dt*vtsHelper::tsscaler*500, 10);

            if(!aep->polarity)
            {
                cv::circle(image, dot, 1, aqua, cv::FILLED);
            }
            else
            {
                cv::circle(image, dot, 1, violet, cv::FILLED);
            }

        }
    }

}

// ISO DRAW SKIN //
// ======== //

const std::string isoDrawSkin::drawtype = "ISOSKIN";

std::string isoDrawSkin::getDrawType()
{
    return isoDrawSkin::drawtype;
}

std::string isoDrawSkin::getEventType()
{
    return SkinEvent::tag;
}



void isoDrawSkin::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    //auto light_violet = (white - (white - violet) * 0.25);
    //auto light_aqua = (white - (white - aqua) * 0.25);

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    //if() return;
    //if(eSet.empty()) return;

    //ev::vQueue::const_reverse_iterator qi;

    if(!eSet.empty() && vTime < 0)
        vTime = eSet.back()->stamp;

    int skip = 1 + eSet.size() / 100000;

    for(int i = eSet.size() - 1; i >= 0; i -= skip) {

        //transform valueswhite
        auto aep = is_event<SkinEvent>(eSet[i]);
        int dt = vTime-aep->stamp;

        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if((unsigned int)dt > max_window) continue;
        dt = dt * ts_to_axis + 0.5;

        int index = aep->taxel;
        int px = tmap.xoffset + tmap.scaling* std :: get<0>(tmap.pos[index]);
        int py =  tmap.yoffset + tmap.scaling* std :: get<1>(tmap.pos[index]);
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
            //isoimage.at<cv::Vec3b>(py, px) = light_violet;
            cv::Point center(px,py);
            cv::circle(isoimage, center, radius, aqua, cv::FILLED, cv::LINE_AA);
        } else {
            //  isoimage.at<cv::Vec3b>(py, px) = light_aqua;
            cv::Point center(px,py);
            cv::circle(isoimage, center, radius, violet,cv::FILLED, cv::LINE_AA);
        }
    }

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

}
