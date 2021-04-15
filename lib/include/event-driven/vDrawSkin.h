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

#ifndef __VDRAWSKIN__
#define __VDRAWSKIN__

#include <string>
#include <opencv2/opencv.hpp>
#include <list>
#include <yarp/os/all.h>
#include "event-driven/vDraw.h"

namespace ev {

class PAD {
protected:

    const double DEG2RAD=M_PI/180.0;
    int nTaxels;

    double *dX{nullptr},*dY{nullptr};

    inline std::list<unsigned int> vectorofIntEqualto(const std::vector<int> vec, const int val)
    {
        std::list<unsigned int> res;
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (vec[i]==val)
            {
                res.push_back(i);
            }
        }
        return res;
    }

    std::map<int, std::vector<double> > data;

public:

    int xoffset{0},yoffset{0};
    int scaling{4};
    int radius{6};
    int noise{2500};
    int max_value{15000};//good for default flat skin patch

    ~PAD()
    {
        if(dX) {
            delete dX;
            dX = nullptr;
        }

        if(dY) {
            delete dY;
            dY = nullptr;
        }

    }

    std::map<int, std::list<unsigned int> > repr2TaxelList;
    virtual void initRepresentativeTaxels(std::vector<int> taxelMap) = 0;
    virtual int* getID(const int index) const = 0;
    virtual std::tuple<double,double> makeMap( int ID) const = 0;
    void get_data(int id,double x,double y, double orientation,double gain, int mirror,int layoutNum){
        data.insert(std::make_pair(id,std::vector<double> {x,y,orientation,gain,double(mirror)}));
    };

};


class Triangle_10pad : public PAD {

private:


    int* getID(const int index)const {

        static int result[3] = {0, 0, 0};
        int triangleID = 0,taxelInTriangle = 0;
        bool success = 0;


        for(auto mapit=repr2TaxelList.begin();mapit!=repr2TaxelList.end();mapit++){
            auto listIntriangle= mapit->second;
            auto it = std::find(listIntriangle.begin(), listIntriangle.end(), index);
            if(it != listIntriangle.end()){
                triangleID = mapit->first;
                //taxelInTriangle = std :: distance(listIntriangle.begin(),it);
                taxelInTriangle = index-16*(triangleID-3)/12;
                success =1;
                break;
            }
        }

        if (triangleID==-1){
            for(auto mapit=repr2TaxelList.begin();mapit!=repr2TaxelList.end();mapit++){
                auto listIntriangle= mapit->second;
                auto it = std::find(listIntriangle.begin(), listIntriangle.end(), index-1);
                if(it != listIntriangle.end()){
                    triangleID = mapit->first;
                    break;
                }
            }
            if(taxelInTriangle%2==0)taxelInTriangle=6;
            else taxelInTriangle = 10;
        }

        triangleID = (triangleID-3)/12;
        result[0] = triangleID;
        result[1] = taxelInTriangle;
        result[2] = success;
        return result;
    };


public:

    Triangle_10pad(){

        nTaxels=12;

        xoffset = -200;
        yoffset = 600;


        dX = new double[nTaxels];
        dY = new double[nTaxels];

        dX[8]=-128.62; dY[8]=222.83;
        dX[10]= 0; dY[10]=296.55;
        dX[ 9]= 0; dY[ 9]=445.54;
        dX[ 11]= 128.62; dY[ 11]=222.83;
        dX[ 0]= 257.2; dY[ 0]=0;
        dX[ 6]= -256.3; dY[ 6]=-147.64;
        dX[ 1]=  385.83; dY[1]=-222.83;
        dX[ 2]=128.62; dY[ 2]=-222.83;
        dX[ 3]=0.0; dY[ 3]=0.0;
        dX[ 4]=-128.62; dY[ 4]=-222.83;
        dX[ 5]=-385.83; dY[ 5]=-222.83;
        dX[ 7]=-257.2; dY[ 7]=0.0;
    };

    void initRepresentativeTaxels(std::vector<int> taxelMap){
        //yarp::os::RecursiveLockGuard rlg(recursive_mutex);
        std::list<int> mapp(taxelMap.begin(), taxelMap.end());
        mapp.sort();
        mapp.unique();
        double s=0;

        size_t mappsize = mapp.size();
        for (size_t i = 0; i < mappsize; i++)
        {
            repr2TaxelList[mapp.front()]=vectorofIntEqualto(taxelMap,mapp.front());
            mapp.pop_front();
        }
        //assign correct spacing between addresses: every 12 5 empty addresses
        for (auto it =repr2TaxelList.begin();it!=repr2TaxelList.end();it++){
            for (auto it2 =  it->second.begin(); it2 !=it->second.end(); it2++)
            {
                if(it->first==-1){
                    s+=1;
                    *it2 = 10000; //dummy value to avoid problems. The algorithm below works only for contiguous triangles in assigning a correct value
                    //*it2 = (ceil(s/2)-1) *4;//heat taxels are reassigned to be within triangle (es. traingle ID 0--> 6 and 10, ID = 1 --> 22,26)
                }
                else{
                    int triangleID = (it->first-3)/12;
                    *it2 += triangleID *4;
                }
            }

        }
        // return true;
    }




    std::tuple<double,double> makeMap(int index)const{

        // ID=(ID-3)/12;
        int ID = getID(index)[0];
        int taxelIntriangle =getID(index)[1];

        std::vector<double> info = data.find(ID)->second;
        const double scale=1/40.0;

        double cx = info[0];
        double cy = info[1];
        double th = info[2];
        //double gain = info[3];
        int lrMirror = int (info[4]);


        const double CST=cos(DEG2RAD*(th+0));
        const double SNT=sin(DEG2RAD*(th+0));

        double x=scale*dX[taxelIntriangle];
        double y=scale*dY[taxelIntriangle];
        if (lrMirror==1) x=-x;

        double u=cx+CST*x-SNT*y;
        double v=cy+SNT*x+CST*y;
        if ((taxelIntriangle!=6) && (taxelIntriangle!=10)) return std :: make_tuple(abs(u), v);
        else return std :: make_tuple(-1000,-1000);//dummy location
    };

};



class Right_hand : public PAD{
private:
    int palmShift;
    //hand must be a wrapper for fingers and palm. Indeed the calibration file is unique
public:
    Right_hand(int shift){
        xoffset = 352;
        yoffset = 400;
        scaling =6;
        palmShift = shift;
        noise = 2300;
        max_value = 10000;

        dX = new double[48];
        dY = new double[48];
        std::cout<< "\nInitialise hand"<<std::endl;
    };

    void initRepresentativeTaxels(std::vector<int> taxelMap){//correct only for 10 pad triangle (?)
        //yarp::os::RecursiveLockGuard rlg(recursive_mutex);
        std::list<int> mapp(taxelMap.begin(), taxelMap.end());
        mapp.sort();
        mapp.unique();

        size_t mappsize = mapp.size();
        for (size_t i = 0; i < mappsize; i++)
        {
            repr2TaxelList[mapp.front()]=vectorofIntEqualto(taxelMap,mapp.front());
            mapp.pop_front();
        }

        int s=0,t=-1;

        for (auto it =repr2TaxelList.begin();it!=repr2TaxelList.end();it++){
            for (auto it2 =  it->second.begin(); it2 !=it->second.end(); it2++){
                if(it->first ==-1)*it2=10000;//dummy value ;

                if(it->first>60){
                    //if(s==5) s+=1;/ skip 6th

                    if (t==3){
                        if(s==7)s+=1;
                    }
                    else{
                        if(s==11) s+=1;//skip 12th except the 4th triangle where it skips the 8th
                    }
                    if(s%12==0) {t+=1;s=0;}

                    *it2 =128+s+ t*16;
                    s+=1;

                }
                else{
                    int triangleID = (it->first-3)/12;
                    *it2 += triangleID *4;
                }
            }

        }
    }

    int* getID(const int index) const{

        static int result[3] = {0, 0, 0};
        int triangleID = 0,taxelInTriangle = 0;
        bool success =0;
        //std::cout<<"\n HERE"<<index<<std::endl;

        for(auto mapit=repr2TaxelList.begin();mapit!=repr2TaxelList.end();mapit++){
            auto listIntriangle= mapit->second;
            auto it = std::find(listIntriangle.begin(), listIntriangle.end(), index);
            if(it != listIntriangle.end()){

                triangleID = mapit->first;

                if (triangleID==-1){
                    //working in progress

                    triangleID =8; //palm ID, heat pads should be only in the palm

                    taxelInTriangle = std :: distance(listIntriangle.begin(),it);

                    //taxelInTriangle = (taxelInTriangle%4)*12+5;//5,17,29,41
                    taxelInTriangle = (taxelInTriangle%4 +1)*12-1; //11,23,35,43
                    if(taxelInTriangle==47) taxelInTriangle=43;


                }
                else if(triangleID<60){
                    //yWarning("Finger");
                    triangleID = (triangleID-9)/12;
                    taxelInTriangle = std :: distance(listIntriangle.begin(),it);
                }
                else{
                    //yWarning("Palm");

                    taxelInTriangle = (*it-128)-4*((*it-128)/16);

                    triangleID = 8;

                }
                success =1;
                break;
            }
        }

        result[0] = triangleID;
        result[1] = taxelInTriangle;
        result[2] = success;
        return result;
    };
    std::tuple<double,double> makeMap(int index)const{

        int ID = getID(index)[0];
        int taxelIntriangle =getID(index)[1];

        std::vector<double> info = data.find(ID)->second;

        double cx = info[0];
        double cy = info[1];
        double th = info[2];
        //double gain = info[3];
        int lrMirror = int (info[4]);
        const double CST=cos(DEG2RAD*th);
        const double SNT=sin(DEG2RAD*th);


        if(ID<=4){
            // yInfo("\n Quest for finger map \n");
            const double scale=2.7/15.3;


            dX[11]=-41.0; dY[11]=10.0;
            dX[0]= 15.0; dY[0]=10.0;
            dX[1]= 15.0; dY[1]=35.0;
            dX[2]= 41.0; dY[2]=35.0;
            dX[3]= 30.0; dY[3]=64.0;
            dX[4]= 11.0; dY[4]=58.0;
            dX[5]=  0.0; dY[5]=82.0;
            dX[6]=-11.0; dY[6]=58.0;
            dX[7]=-30.0; dY[7]=64.0;
            dX[8]=-41.0; dY[8]=35.0;
            dX[9]=-15.0; dY[9]=35.0;
            dX[10]=-15.0; dY[10]=10.0;


            // std :: cout << "HERE"<<std ::endl;
            double x=scale*dX[taxelIntriangle];
            double y=scale*dY[taxelIntriangle];

            double u=cx+CST*x-SNT*y;
            double v=cy+SNT*x+CST*y;


            return std :: make_tuple(u,v);

        }
        else if(ID==8){
            // yInfo("\n Quest for palm map \n");

            const double scale=1.2;

            dX[27]=1.5;  dY[27]=6.5;
            dX[26]=6.5;  dY[26]=6;
            dX[25]=11.5; dY[25]=6;
            dX[24]=16.5; dY[24]=6;
            dX[31]=21.5; dY[31]=6;
            dX[29]=6.5;  dY[29]=1;
            dX[28]=11.5; dY[28]=1;
            dX[32]=16.5; dY[32]=1;
            dX[33]=21.5; dY[33]=1;
            dX[35]=9.5;  dY[35]=-2; //thermal_pad
            dX[30]=14.5; dY[30]=-3.5;
            dX[34]=21.5; dY[34]=-4;


            dX[6]=27;   dY[6]=6;
            dX[3]=32;   dY[3]=6;
            dX[2]=37;   dY[2]=6;
            dX[1]=42;   dY[1]=5.5;
            dX[0]=47;   dY[0]=4.5;
            dX[11]=51.7; dY[11]=4; //thermal_pad
            dX[7]=27;    dY[7]=1;
            dX[8]=32;    dY[8]=1;
            dX[4]=37;    dY[4]=1;
            dX[5]=42;   dY[5]=0;
            dX[9]=27;    dY[9]=-3.5;
            dX[10]=32;    dY[10]=-3.5;


            dX[16]=37.5;    dY[16]=-4.5;
            dX[15]=42;     dY[15]=-5.5;
            dX[14]=46.5;    dY[14]=-8;
            dX[20]=27;    dY[20]=-9;
            dX[21]=32;    dY[21]=-9;
            dX[17]=37;    dY[17]=-9;
            dX[19]=42;    dY[19]=-10.5;
            dX[22]=38;    dY[22]=-14;
            dX[18]=43;    dY[18]=-16;
            dX[13]=47;    dY[13]=-13;
            dX[12]=47.5;    dY[12]=-18;
            dX[23]=43.5;    dY[23]=-20; //thermal_pad

            dX[46]=33;    dY[46]=-14.5;
            dX[47]=28;    dY[47]=-14.5;
            dX[36]=28;    dY[36]=-19.5;
            dX[42]=33;    dY[42]=-19.5;
            dX[45]=38;    dY[45]=-19.5;
            dX[37]=28;    dY[37]=-24.5;
            dX[38]=33;    dY[38]=-24.5;
            dX[41]=38;    dY[41]=-24.5;
            dX[44]=43;    dY[44]=-26;
            dX[39]=35;    dY[39]=-29;
            dX[40]=40;    dY[40]=-29.5;
            dX[43]=37;    dY[43]=-32.5; //thermal pad



            double x=scale*dX[taxelIntriangle];
            double y=scale*dY[taxelIntriangle];

            //std :: cout<< "\nHERE"<<std::endl;

            if (lrMirror==1) x=-x;

            double u=cx+CST*x-SNT*y;
            double v=cy+SNT*x+CST*y;

            if ((taxelIntriangle!=43) && (taxelIntriangle!=23) && (taxelIntriangle!=11) && (taxelIntriangle!=35) )return std :: make_tuple(abs(u)+palmShift,v);
            //if ((taxelIntriangle!=41) && (taxelIntriangle!=29) && (taxelIntriangle!=17) && (taxelIntriangle!=5) )return std :: make_tuple(abs(u)+palmShift,v);
            else return std :: make_tuple(-1000,-1000);//dummy location
        }

        else{
            yWarning("\n Unexpected ID");
            return std :: make_tuple(-10000,-10000);
        }






    };
};


class Left_hand : public PAD{
private:
    int palmShift;
    //hand must be a wrapper for fingers and palm. Indeed the calibration file is unique
public:
    Left_hand(int shift){
        xoffset = 352;
        yoffset = 400;
        scaling =6;
        palmShift = shift;
        noise = 2300;
        max_value = 10000;

        dX = new double[48];
        dY = new double[48];
        std::cout<< "\nInitialise hand"<<std::endl;
    };

    void initRepresentativeTaxels(std::vector<int> taxelMap){//correct only for 10 pad triangle (?)
        //yarp::os::RecursiveLockGuard rlg(recursive_mutex);
        std::list<int> mapp(taxelMap.begin(), taxelMap.end());
        mapp.sort();
        mapp.unique();

        size_t mappsize = mapp.size();
        for (size_t i = 0; i < mappsize; i++)
        {
            repr2TaxelList[mapp.front()]=vectorofIntEqualto(taxelMap,mapp.front());
            mapp.pop_front();
        }


        int s=0,t=-1;

        for (auto it =repr2TaxelList.begin();it!=repr2TaxelList.end();it++){
            for (auto it2 =  it->second.begin(); it2 !=it->second.end(); it2++){
                if(it->first ==-1)*it2=10000;//dummy value ;

                if(it->first>60){
                    //if(s==5) s+=1;/ skip 6th

                    if (t==3){
                        if(s==7)s+=1;
                    }
                    else{
                        if(s==11) s+=1;//skip 12th except the 4th triangle where it skips the 8th
                    }
                    if(s%12==0) {t+=1;s=0;}

                    *it2 =128+s+ t*16;
                    s+=1;

                }
                else{
                    int triangleID = (it->first-3)/12;
                    *it2 += triangleID *4;
                }
            }

        }
    }

    int* getID(const int index) const{

        static int result[3] = {0, 0, 0};
        int triangleID = 0,taxelInTriangle = 0;
        bool success = 0;
        //std::cout<<"\n HERE"<<index<<std::endl;

        for(auto mapit=repr2TaxelList.begin();mapit!=repr2TaxelList.end();mapit++){
            auto listIntriangle= mapit->second;
            auto it = std::find(listIntriangle.begin(), listIntriangle.end(), index);
            if(it != listIntriangle.end()){

                triangleID = mapit->first;

                if (triangleID==-1){
                    //working in progress

                    triangleID =8; //palm ID, heat pads should be only in the palm

                    taxelInTriangle = std :: distance(listIntriangle.begin(),it);

                    //taxelInTriangle = (taxelInTriangle%4)*12+5;//5,17,29,41
                    taxelInTriangle = (taxelInTriangle%4 +1)*12-1; //11,23,35,43
                    if(taxelInTriangle==47) taxelInTriangle=43;


                }
                else if(triangleID<60){
                    //yWarning("Finger");
                    triangleID = (triangleID-9)/12;
                    taxelInTriangle = std :: distance(listIntriangle.begin(),it);
                }
                else{
                    //yWarning("Palm");

                    taxelInTriangle = (*it-128)-4*((*it-128)/16);

                    triangleID = 8;

                }
                success =1;
                break;
            }
        }

        result[0] = triangleID;
        result[1] = taxelInTriangle;
        result[2] = success;
        return result;
    };

    std::tuple<double,double> makeMap(int index)const{

        int ID = getID(index)[0];
        int taxelIntriangle =getID(index)[1];

        std::vector<double> info = data.find(ID)->second;

        double cx = info[0];
        double cy = info[1];
        double th = info[2];
        //double gain = info[3];
        int lrMirror = int (info[4]);
        const double CST=cos(DEG2RAD*th);
        const double SNT=sin(DEG2RAD*th);


        if(ID<=4){
            // yInfo("\n Quest for finger map \n");
            const double scale=2.7/15.3;

            dX[0]= 41.0; dY[0]=10.0;
            dX[1]= 15.0; dY[1]=10.0;
            dX[2]= 15.0; dY[2]=35.0;
            dX[3]= 41.0; dY[3]=35.0;
            dX[4]= 30.0; dY[4]=64.0;
            dX[5]= 11.0; dY[5]=58.0;
            dX[6]=  0.0; dY[6]=82.0;
            dX[7]=-11.0; dY[7]=58.0;
            dX[8]=-30.0; dY[8]=64.0;
            dX[9]=-41.0; dY[9]=35.0;
            dX[10]=-15.0; dY[10]=35.0;
            dX[11]=-15.0; dY[11]=10.0;




            // std :: cout << "HERE"<<std ::endl;
            double x=scale*dX[taxelIntriangle];
            double y=scale*dY[taxelIntriangle];

            double u=cx+CST*x-SNT*y;
            double v=cy+SNT*x+CST*y;


            return std :: make_tuple(u,v);

        }
        else if(ID==8){
            // yInfo("\n Quest for palm map \n");

            const double scale=1.2;

            dX[29]=1.5;  dY[29]=6.5;
            dX[28]=6.5;  dY[28]=6;
            dX[30]=11.5; dY[30]=6;
            dX[31]=16.5; dY[31]=6;
            dX[33]=21.5; dY[33]=6;
            dX[27]=6.5;  dY[27]=1;
            dX[26]=11.5; dY[26]=1;
            dX[32]=16.5; dY[32]=1;
            dX[34]=21.5; dY[34]=1;
            dX[35]=9.5;  dY[35]=-2; //thermal_pad
            dX[25]=14.5; dY[25]=-3.5;
            dX[24]=21.5; dY[24]=-4;


            dX[6]=27;   dY[6]=6;
            dX[7]=32;   dY[7]=6;
            dX[8]=37;   dY[8]=6;
            dX[9]=42;   dY[9]=5.5;
            dX[10]=47;   dY[10]=4.5;
            dX[11]=51.7; dY[11]=4; //thermal_pad
            dX[3]=27;    dY[3]=1;
            dX[1]=32;    dY[1]=1;
            dX[4]=37;    dY[4]=1;
            dX[5]=42;   dY[5]=0;
            dX[2]=27;    dY[2]=-3.5;
            dX[0]=32;    dY[0]=-3.5;


            dX[17]=37.5;    dY[17]=-4.5;
            dX[20]=42;     dY[20]=-5.5;
            dX[21]=46.5;    dY[21]=-8;
            dX[15]=27;    dY[15]=-9;
            dX[14]=32;    dY[14]=-9;
            dX[16]=37;    dY[16]=-9;
            dX[19]=42;    dY[19]=-10.5;

            dX[13]=38;    dY[13]=-14;
            dX[18]=43;    dY[18]=-16;
            dX[22]=47;    dY[22]=-13;
            dX[12]=47.5;    dY[12]=-18;
            dX[23]=43.5;    dY[23]=-20; //thermal_pad

            dX[45]=28;    dY[45]=-19.5;
            dX[42]=33;    dY[42]=-19.5;
            dX[36]=38;    dY[36]=-19.5;
            dX[44]=28;    dY[44]=-24.5;
            dX[40]=33;    dY[40]=-24.5;
            dX[38]=38;    dY[38]=-24.5;
            dX[37]=43;    dY[37]=-26;
            dX[41]=35;    dY[41]=-29;
            dX[39]=40;    dY[39]=-29.5;
            dX[43]=37;    dY[43]=-32.5; //thermal pad
            dX[47]=33;    dY[47]=-14.5;
            dX[46]=28;    dY[46]=-14.5;



            double x=scale*dX[taxelIntriangle];
            double y=scale*dY[taxelIntriangle];

            //std :: cout<< "\nHERE"<<std::endl;

            if (lrMirror==1) x=-x;

            double u=cx+CST*x-SNT*y;
            double v=cy+SNT*x+CST*y;

            if ((taxelIntriangle!=43) && (taxelIntriangle!=23) && (taxelIntriangle!=11) && (taxelIntriangle!=35) )return std :: make_tuple(abs(u)+palmShift,v);
            //    if ((taxelIntriangle!=41) && (taxelIntriangle!=29) && (taxelIntriangle!=17) && (taxelIntriangle!=5) )return std :: make_tuple(abs(u)+palmShift,v);
            else return std :: make_tuple(-1000,-1000);//dummy location
        }

        else{
            yWarning("\n Unexpected ID");
            return std :: make_tuple(-10000,-10000);
        }






    };
};

//////////////////////////////////////////////




class loadMap {
private:

    std::map<int,std::tuple<double, double>> default_pos={{0,{24, 9}},{1,{24, 7}},{2,{23, 8}},{3,{23, 10}},{4,{22, 9}},{5,{21, 10}},{7,{22, 11}},{8,{23, 12}},{9,{24, 13}},{11,{24, 11}},{16,{22, 17}},{17,{23, 16}},
                                                          {18,{22, 15}},{19,{21, 16}},{20,{21, 14}},{21,{20, 13}},{23,{20, 15}},{24,{20, 17}},{25,{20, 19}},{27,{21, 18}},{32,{24, 21}},{33,{24, 19}},
                                                          {34,{23, 20}},{35,{23, 22}},{36,{22, 21}},{37,{21, 22}},{39,{22, 23}},{40,{23, 24}},{41,{24, 25}},{43,{24, 23}},{48,{2, 11}},{49,{2, 13}},
                                                          {50,{3, 12}},{51,{3, 10}},{52,{4, 11}},{53,{5, 10}},{55,{4, 9}},{56,{3, 8}},{57,{2, 7}},{59,{2, 9}},{64,{4, 3}},{65,{3, 4}},
                                                          {66,{4, 5}},{67,{5, 4}},{68,{5, 6}},{69,{6, 7}},{71,{6, 5}},{72,{6, 3}},{73,{6, 1}},{75,{5, 2}},{80,{8, 5}},{81,{8, 7}},
                                                          {82,{9, 6}},{83,{9, 4}},{84,{10, 5}},{85,{11, 4}},{87,{10, 3}},{88,{9, 2}},{89,{8, 1}},{91,{8, 3}},{96,{5, 18}},{97,{6, 19}},
                                                          {98,{6, 17}},{99,{5, 16}},{100,{6, 15}},{101,{6, 13}},{103,{5, 14}},{104,{4, 15}},{105,{3, 16}},{107,{4, 17}},{112,{4, 23}},{113,{5, 22}},
                                                          {114,{4, 21}},{115,{3, 22}},{116,{3, 20}},{117,{2, 19}},{119,{2, 21}},{120,{2, 23}},{121,{2, 25}},{123,{3, 24}},{128,{11, 24}},{129,{12, 25}},
                                                          {130,{12, 23}},{131,{11, 22}},{132,{12, 21}},{133,{12, 19}},{135,{11, 20}},{136,{10, 21}},{137,{9, 22}},{139,{10, 23}},{144,{8, 17}},{145,{8, 19}},
                                                          {146,{9, 18}},{147,{9, 16}},{148,{10, 17}},{149,{11, 16}},{151,{10, 15}},{152,{9, 14}},{153,{8, 13}},{155,{8, 15}},{160,{10, 9}},{161,{9, 10}},
                                                          {162,{10, 11}},{163,{11, 10}},{164,{11, 12}},{165,{12, 13}},{167,{12, 11}},{168,{12, 9}},{169,{12, 7}},{171,{11, 8}},{176,{15, 8}},{177,{14, 7}},
                                                          {178,{14, 9}},{179,{15, 10}},{180,{14, 11}},{181,{14, 13}},{183,{15, 12}},{184,{16, 11}},{185,{17, 10}},{187,{16, 9}},{192,{17, 18}},{193,{18, 19}},
                                                          {194,{18, 17}},{195,{17, 16}},{196,{18, 15}},{197,{18, 13}},{199,{17, 14}},{200,{16, 15}},{201,{15, 16}},{203,{16, 17}},{208,{16, 23}},{209,{17, 22}},
                                                          {210,{16, 21}},{211,{15, 22}},{212,{15, 20}},{213,{14, 19}},{215,{14, 21}},{216,{14, 23}},{217,{14, 25}},{219,{15, 24}},{224,{16, 3}},{225,{15, 4}},
                                                          {226,{16, 5}},{227,{17, 4}},{228,{17, 6}},{229,{18, 7}},{231,{18, 5}},{232,{18, 3}},{233,{18, 1}},{235,{17, 2}},{240,{21, 2}},{241,{20, 1}},
                                                          {242,{20, 3}},{243,{21, 4}},{244,{20, 5}},{245,{20, 7}},{247,{21, 6}},{248,{22, 5}},{249,{23, 4}},{251,{22, 3}}};

public:

    std::map<int,std::tuple<double, double>> pos {};

    int xoffset,yoffset,scaling,radius,noise,max_value;


    loadMap(){

        noise = 2500;
        max_value = 15000;

        xoffset = 80;
        //yoffset = 120;
        yoffset = 680;
        scaling=20;
        radius =10;

    };

    void initialise(std::string bodypart){

        std::vector<int> taxel2Repr;
        yarp::os::ResourceFinder rf1, rf2;
        std :: string filename1,filename2;

        int check=0;

        if (bodypart == "arm"){
            Triangle_10pad skin;

            filename1 = "skinGui/right_forearm_V2.ini";
            filename2 = "positions/right_forearm_V2.txt";

            rf2.setVerbose(true);
            rf2.setDefaultContext("skinGui");            //overridden by --context parameter
            rf2.setDefaultConfigFile(filename2); //overridden by --from parameter
            rf2.configure(0,NULL);


            if (rf2.check("taxel2Repr")){
                check+=1;
                std :: cout << "Calibration data";
                yarp::os::Bottle calibration_data = *(rf2.find("taxel2Repr").asList());
                for (size_t i = 0; i < calibration_data.size(); i++){
                    taxel2Repr.push_back(calibration_data.get(i).asInt());
                }
                skin.initRepresentativeTaxels(taxel2Repr);
            }

            rf1.setVerbose(true);
            rf1.setDefaultContext("skinGui");            //overridden by --context parameter
            rf1.setDefaultConfigFile(filename1); //overridden by --from parameter
            rf1.configure(0,NULL);


            if (rf1.check("SENSORS")){
                check+=1;
                std:: cout<< "\n Sensors data"<<std::endl;
                yarp::os::Bottle &ini_data = rf1.findGroup("SENSORS");

                for (size_t i = 1; i < (ini_data.size()); i++){

                    yarp::os::Bottle sensorConfig(ini_data.get(i).toString());

                    //std::string type(sensorConfig.get(0).asString()); useless since forearm has always same type and hand has a precise numbering for palm and fingers.

                    int id=sensorConfig.get(1).asInt();
                    double x=sensorConfig.get(2).asDouble();
                    double y=sensorConfig.get(3).asDouble();
                    double orientation=sensorConfig.get(4).asDouble();
                    double gain=sensorConfig.get(5).asDouble();
                    int    mirror=sensorConfig.get(6).asInt();
                    int    layoutNum=sensorConfig.get(7).asInt();

                    skin.get_data(id,x,y,orientation,gain,mirror,layoutNum);

                }
            }


            if(check==2){
                yInfo("Building taxel map from file");
                for (auto it =skin.repr2TaxelList.begin();it!=skin.repr2TaxelList.end();it++){
                    for (auto it2 =  it->second.begin(); it2 !=it->second.end(); it2++){
                        pos.insert(std::pair<int,std::tuple<double, double>>(*it2,skin.makeMap(*it2)));
                    }
                }
                xoffset=skin.xoffset;
                yoffset=skin.yoffset;
                scaling =skin.scaling;
                radius = skin.radius;
                // for (auto it =pos.begin();it!=pos.end();it++){
                // std:: cout <<"\n Address "<< it->first <<"\t Coordinate ="<< std :: get<0>(it->second)<<"   "<<std :: get<1>(it->second)<<std::endl;
                // }
            }
            else{
                yWarning("Unable to build the taxel map from file");
                pos = default_pos;
            }
        }

        if(bodypart == "rightHand"){
            Right_hand handR = Right_hand(-40);

            filename1 = "skinGui/right_hand_V2_1.ini";
            filename2 = "positions/right_hand_V2_1.txt";

            rf2.setVerbose(false);
            rf2.setDefaultContext("skinGui");            //overridden by --context parameter
            rf2.setDefaultConfigFile(filename2); //overridden by --from parameter
            rf2.configure(0,NULL);
            rf2.setVerbose(true);

            if (rf2.check("taxel2Repr")){
                check+=1;
                std :: cout << "Calibration data";
                yarp::os::Bottle calibration_data = *(rf2.find("taxel2Repr").asList());
                for (size_t i = 0; i < calibration_data.size(); i++){
                    taxel2Repr.push_back(calibration_data.get(i).asInt());
                }
                handR.initRepresentativeTaxels(taxel2Repr);
            }

            rf1.setVerbose(false);
            rf1.setDefaultContext("skinGui");            //overridden by --context parameter
            rf1.setDefaultConfigFile(filename1); //overridden by --from parameter
            rf1.configure(0,NULL);
            rf1.setVerbose(true);

            if (rf1.check("SENSORS")){
                check+=1;
                std:: cout<< "\n Sensors data"<<std::endl;
                yarp::os::Bottle &ini_data = rf1.findGroup("SENSORS");

                for (size_t i = 1; i < (ini_data.size()); i++){

                    yarp::os::Bottle sensorConfig(ini_data.get(i).toString());

                    //std::string type(sensorConfig.get(0).asString()); useless since forearm has always same type and hand has a precise numbering for palm and fingers.

                    int id=sensorConfig.get(1).asInt();
                    double x=sensorConfig.get(2).asDouble();
                    double y=sensorConfig.get(3).asDouble();
                    double orientation=sensorConfig.get(4).asDouble();
                    double gain=sensorConfig.get(5).asDouble();
                    int    mirror=sensorConfig.get(6).asInt();
                    int    layoutNum=sensorConfig.get(7).asInt();

                    handR.get_data(id,x,y,orientation,gain,mirror,layoutNum);
                }
            }
            if(check==2){
                yInfo("Building taxel map from file");
                for (auto it =handR.repr2TaxelList.begin();it!=handR.repr2TaxelList.end();it++){
                    for (auto it2 =  it->second.begin(); it2 !=it->second.end(); it2++){
                        pos.insert(std::pair<int,std::tuple<double, double>>(*it2,handR.makeMap(*it2)));
                    }
                }
                xoffset=handR.xoffset;
                yoffset=handR.yoffset;
                scaling =handR.scaling;
                radius = handR.radius;
                noise = handR.noise;
                max_value = handR.max_value;
            }
            else {
                yWarning("Unable to build the taxel map from file");
                pos = default_pos;
            }
        }

        if(bodypart == "leftHand"){

            Left_hand handL = Left_hand(-5);


            filename1 = "skinGui/left_hand_V2_1.ini";
            filename2 = "positions/left_hand_V2_1.txt";

            rf2.setVerbose(false);
            rf2.setDefaultContext("skinGui");            //overridden by --context parameter
            rf2.setDefaultConfigFile(filename2); //overridden by --from parameter
            rf2.configure(0,NULL);
            rf2.setVerbose(true);

            if (rf2.check("taxel2Repr")){
                check+=1;
                std :: cout << "Calibration data";
                yarp::os::Bottle calibration_data = *(rf2.find("taxel2Repr").asList());
                for (size_t i = 0; i < calibration_data.size(); i++){
                    taxel2Repr.push_back(calibration_data.get(i).asInt());
                }
                handL.initRepresentativeTaxels(taxel2Repr);
            }

            rf1.setVerbose(false);
            rf1.setDefaultContext("skinGui");            //overridden by --context parameter
            rf1.setDefaultConfigFile(filename1); //overridden by --from parameter
            rf1.configure(0,NULL);
            rf1.setVerbose(true);

            if (rf1.check("SENSORS")){
                check+=1;
                std:: cout<< "\n Sensors data"<<std::endl;
                yarp::os::Bottle &ini_data = rf1.findGroup("SENSORS");

                for (size_t i = 1; i < (ini_data.size()); i++){

                    yarp::os::Bottle sensorConfig(ini_data.get(i).toString());

                    //std::string type(sensorConfig.get(0).asString()); useless since forearm has always same type and hand has a precise numbering for palm and fingers.

                    int id=sensorConfig.get(1).asInt();
                    double x=sensorConfig.get(2).asDouble();
                    double y=sensorConfig.get(3).asDouble();
                    double orientation=sensorConfig.get(4).asDouble();
                    double gain=sensorConfig.get(5).asDouble();
                    int    mirror=sensorConfig.get(6).asInt();
                    int    layoutNum=sensorConfig.get(7).asInt();

                    handL.get_data(id,x,y,orientation,gain,mirror,layoutNum);
                }
            }
            if(check==2){
                yInfo("Building taxel map from file");
                for (auto it =handL.repr2TaxelList.begin();it!=handL.repr2TaxelList.end();it++){
                    for (auto it2 =  it->second.begin(); it2 !=it->second.end(); it2++){
                        pos.insert(std::pair<int,std::tuple<double, double>>(*it2,handL.makeMap(*it2)));
                    }
                }
                xoffset=handL.xoffset;
                yoffset=handL.yoffset;
                scaling =handL.scaling;
                radius = handL.radius;
                noise = handL.noise;
                max_value = handL.max_value;
            }
            else {
                yWarning("Unable to build the taxel map from file");
                pos = default_pos;
            }
        }

    };

};

class EV_API isoDrawSkin : public isoDraw {

private:

    loadMap tmap;

    int radius;

public:

    void initialise(){


        Xlimit = 800;
        Ylimit = 800;

        isoDraw :: initialise();
        tmap.initialise("leftHand");

        radius = tmap.radius-1;

    };

    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();
};




class EV_API skinDraw : public vDraw {

private:

    loadMap tmap;
    cv::Mat baseimage;

    int radius;

public:

    void initialise(){

        Xlimit = 800;
        Ylimit = 800;

        baseimage = cv::Mat(cv::Size(Xlimit, Ylimit), CV_8UC3);
        baseimage.setTo(255);
        tmap.initialise("leftHand");
        radius = tmap.radius;

        int x;
        int y;

        yInfo("scaling = %d", tmap.scaling );
        yInfo("xoffset = %d", tmap.xoffset );
        yInfo("yoffset =%d", tmap.yoffset );

        for (auto it = tmap.pos.begin(); it != tmap.pos.end(); it++)
        {
            x = tmap.xoffset + tmap.scaling* std :: get<0>(it->second);
            y =  tmap.yoffset +tmap.scaling* std :: get<1>(it->second);
            cv::Point centr_all(x, y);
            cv::circle(baseimage, centr_all, radius, black,1, cv::LINE_AA);
        }

    };



    virtual void resetImage(cv::Mat &image)
    {
        baseimage.copyTo(image);
    }
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API skinsampleDraw : public vDraw {


private:

    loadMap tmap;
    cv::Mat baseimage;
    int radius,noise,max_value;

public:
    void initialise(){

        Xlimit = 800;
        Ylimit = 800;

        baseimage = cv::Mat(cv::Size(Xlimit, Ylimit), CV_8UC3);
        baseimage.setTo(255);

        tmap.initialise("leftHand");

        radius = tmap.radius;
        noise = tmap.noise;
        max_value=tmap.max_value;

        int x;
        int y;


        for (auto it = tmap.pos.begin(); it != tmap.pos.end(); it++)
        {
            x = tmap.xoffset + tmap.scaling* std :: get<0>(it->second);
            y =  tmap.yoffset +tmap.scaling* std :: get<1>(it->second);
            cv::Point centr_all(x, y);
            cv::circle(baseimage, centr_all, radius, black,1, cv::LINE_AA);
        }
    };



    virtual void resetImage(cv::Mat &image){
        baseimage.copyTo(image);
    };
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};


class EV_API taxelsampleDraw : public vDraw {

public:
    void resetImage(cv::Mat &image){
        if(image.empty()) {
            image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
            image.setTo(0);
        }
    };
    void initialise(){

        Xlimit = 200;
        Ylimit = 300;

    };
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

class EV_API taxeleventDraw : public vDraw {

public:
    virtual void resetImage(cv::Mat &image){
        if(image.empty()) {
            image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
            image.setTo(0);
        }
    };
    void initialise(){

        Xlimit = 200;
        Ylimit = 300;

    };
    static const std::string drawtype;
    virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
    virtual std::string getDrawType();
    virtual std::string getEventType();

};

} //namespace ev::

#endif
