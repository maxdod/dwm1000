#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "localizer_dwm1001/Anchor.h"
#include "std_msgs/Float64MultiArray.h"
#include "localizer_dwm1001/askPosition.h"
#include <cmath>
struct point 
{
    double x,y;
};

float norm (point p) // get the norm of a vector
{
    return pow(pow(p.x,2)+pow(p.y,2),.5);
}
point finalPose;
point trilateration(point point1, point point2, point point3, double r1, double r2, double r3) {
    point resultPose;
    //unit vector in a direction from point1 to point 2
    double p2p1Distance = pow(pow(point2.x-point1.x,2) + pow(point2.y-   point1.y,2),0.5);
    point ex = {(point2.x-point1.x)/p2p1Distance, (point2.y-point1.y)/p2p1Distance};
    point aux = {point3.x-point1.x,point3.y-point1.y};
    //signed magnitude of the x component
    double i = ex.x * aux.x + ex.y * aux.y;
    //the unit vector in the y direction. 
    point aux2 = { point3.x-point1.x-i*ex.x, point3.y-point1.y-i*ex.y};
    point ey = { aux2.x / norm (aux2), aux2.y / norm (aux2) };
    //the signed magnitude of the y component
    double j = ey.x * aux.x + ey.y * aux.y;
    //coordinates
    double x = (pow(r1,2) - pow(r2,2) + pow(p2p1Distance,2))/ (2 * p2p1Distance);
    double y = (pow(r1,2) - pow(r3,2) + pow(i,2) + pow(j,2))/(2*j) - i*x/j;
    //result coordinates
    double finalX = point1.x+ x*ex.x + y*ey.x;
    double finalY = point1.y+ x*ex.y + y*ey.y;
    resultPose.x = finalX;
    resultPose.y = finalY;
    return resultPose;
}
double sgn(double n)
{
  if (n > 0 ) return 1.0;
  if (n < 0) return -1.0;
  return 0;
}
int ciclo=0;
using namespace message_filters;
ros::Publisher posizione;
typedef sync_policies::ApproximateTime<localizer_dwm1001::Anchor, localizer_dwm1001::Anchor, localizer_dwm1001::Anchor> MySyncPolicy;
typedef Synchronizer<MySyncPolicy> Sync;
boost::shared_ptr<Sync> sync_;
bool getPos(localizer_dwm1001::askPosition::Request  &req,
         localizer_dwm1001::askPosition::Response &res)
{
  res.posX = finalPose.x;
  res.posY = finalPose.y;
  //res.posX = 4.0;
  //res.posY = 3.0;
  return true;
}
void callback(const localizer_dwm1001::AnchorConstPtr& pos1, const localizer_dwm1001::AnchorConstPtr& pos2,
              const localizer_dwm1001::AnchorConstPtr& pos3)
{
  double x=0,y=0;
  double r1 =  pos1->distanceFromTag;
  double r2 =  pos2->distanceFromTag;
  double r3 =  pos3->distanceFromTag;
  std::cout << pos1->distanceFromTag << " " <<  pos2->distanceFromTag << " " << pos3->distanceFromTag <<std::endl;
  
  point p1 = {0.0,0.0};
  point p2 = {0.57,0.0};
  point p3 = {0.27, -0.27};
  
  
    finalPose = trilateration(p1,p2,p3,r1,r2,r3);
    std::cout<<"X:::  "<<finalPose.x<<std::endl;
    std::cout<<"Y:::  "<<finalPose.y<<std::endl; 
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizza_server");

  ros::NodeHandle nh;
  message_filters::Subscriber<localizer_dwm1001::Anchor> anch1_sub(nh, "/tag0/anchor0", 100);
  message_filters::Subscriber<localizer_dwm1001::Anchor> anch2_sub(nh, "/tag1/anchor0", 100);
  message_filters::Subscriber<localizer_dwm1001::Anchor> anch3_sub(nh, "/tag2/anchor0", 100);
  //posizione = nh.advertise<std_msgs::Float64MultiArray>("/dwm1000/pos", 1000);
  ros::ServiceServer service = nh.advertiseService("getPosition", getPos);
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.reset(new Sync(MySyncPolicy(300), anch1_sub, anch2_sub,anch3_sub));
  sync_->registerCallback(boost::bind(&callback, _1, _2, _3));
  
  ros::spin();

  return 0;
}