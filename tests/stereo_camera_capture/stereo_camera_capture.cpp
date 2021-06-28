#include "Camera.hpp"

using namespace std;
using namespace cv;

int main(void){

  int status;

  sv::Camera cam_left;
  sv::Camera cam_right;

  status = cam_left.open(0);

  if(status){
    return -1;
  }

  status = cam_right.open(2);

  if(status){
    return -1;
  }

  while (true){
      Mat frame_left, frame_right;
      Mat frame_out;
      int status;

      status = cam_left.read(frame_left);

      if(status){
        continue;
      }

      status = cam_right.read(frame_right);

      if(status){
        continue;
      }

      hconcat(frame_left, frame_right, frame_out);

      imshow("img",frame_out);
      waitKey(1);
  }

  return 0;
}
