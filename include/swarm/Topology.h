#ifndef Topology_H
#define Topology_H

#define setbit(x,y)  x|=(1<<y);
#define clrbit(x,y)  x&=~(1<<y)
#define reversebit(x,y)  x^=(1<<y)
#define getbit(x,y)   ((x) >> (y)&1)

#include <Eigen/Eigen>
#include <boost/array.hpp> 
#include <math.h>
#include <math_utils.h>
#include <drone_msg/Message.h>
#include <drone_msg/ControlCommand.h>
#include <drone_msg/SwarmCommand.h>
#include <drone_msg/DroneState.h>
#include <drone_msg/PositionReference.h>
#include <drone_msg/AttitudeReference.h>
#include <drone_msg/ControlOutput.h>
#include <drone_msg/LogMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


class Topology{
    public:

    Topology (const Eigen::Matrix<bool, 8, 8>& topo)
        :Topo(topo)
    {}

    Topology (boost::array<uint8_t,8U> topo){
        int i, j;
        for(i = 0;i  < 8; i++){
            for(j = 0; j < 8; j++){
                Topo(i, j) = Topo(j, i) = getbit(topo[i],j);
            }
        }
    }

    Topology (){
        Topo.fill(1);
    }

    void connect(const int i, const int j){
        Topo(i, j) = Topo(j,i) = 1;
    };

    void disconnect(const int i, const int j){
        Topo(i, j) = Topo(j,i) = 0;
    };

    void Fully_connected(){
        Topo.fill(1);
    };

    void Ring_connected(){
        Topo.fill(0);
        for(int i = 0;i  < 8; i++){
                int j = (i+1)%8;
                Topo(i, j) = Topo(j,i) = 1;
            }
    }

    bool get(const int i, const int j){
        return Topo(i,j);
    }

   boost::array<uint8_t,8U>  get_topo_int(){
        int i,j;
        boost::array<uint8_t,8U> topo;
        for(i = 0;i  < 8; i++){
            for(j = 0; j < 8; j++){
                if(Topo(i, j) ){
                    setbit(topo[i],j);
                    setbit(topo[j],i);
                }else{
                    clrbit(topo[i],j);
                    clrbit(topo[j],i);
                }
            }
        }
        return topo;
    }

    void copy_topo_int(boost::array<uint8_t,8U> topo){
        int i, j;
        for(i = 0;i  < 8; i++){
            for(j = 0; j < 8; j++){
                Topo(i, j) = Topo(j, i) = getbit(topo[i],j);
            }
        }
    }

    private:
    Eigen::Matrix<bool, 8, 8> Topo;
};

#endif
