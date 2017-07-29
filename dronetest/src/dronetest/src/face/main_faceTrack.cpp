#include "faceTrack.h"


//-------------------------------Main Function------------------------------------------
int main(int argc, char **argv){

ros::init(argc,argv,"node");

faceTrack face;

ros::spin();                 

return 0;
}
