#include "Camera.hpp"

using namespace std;
using namespace cv;

int main(void){

  int status;

  sv::Camera cam;

  status = cam.open(0);

  if(status){
    return -1;
  }

  while (true){
      Mat frame;
      int status;

      status = cam.read(frame);

      if(status){
        continue;
      }

      imshow("img",frame);
      waitKey(1);
  }

  return 0;
}
