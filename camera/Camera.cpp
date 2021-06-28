
#include "Camera.hpp"
#include <thread>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace sv;
using namespace cv;

#ifdef DEBUG
  #define dbg_user_info(x)   cout << "INFO: " << x << endl
  #define dbg_user_err(x)   cout << "ERR: " << x << endl
#else
  #define dbg_user_info(x)
  #define dbg_user_err(x)
#endif

Camera::Camera(){
  frames_dropped = 0;
}

Camera::~Camera(){
  if(capture.isOpened()) {
    capture.release();
  }
}

int Camera::open(string pipeline){

  capture.open(pipeline, CAP_GSTREAMER);

  if(!capture.isOpened()) {
      dbg_user_err("Unable to open camera");
      return -1;
  }

  capture_thread = thread(&Camera::_update, this);

  return 0;
}

int Camera::open(int camera_index){

  capture.open(camera_index, CAP_ANY);

  if(!capture.isOpened()) {
      dbg_user_err("Unable to open camera");
      return -1;
  }

  capture_thread = thread(&Camera::_update, this);

  return 0;
}

int Camera::read(Mat &frame){

  if(!capture.isOpened()) {
    dbg_user_err("camera closed");
    return -1;
  }


  //a simple way to check if thread has been created
  if(!capture_thread.joinable()){
    dbg_user_err("thread not running");
    return -1;
  }

  //scoped lock for thread safety
  lock_guard<mutex> lock(frames_queue_mutex);

  if(frames_queue.empty()){
    return -1;
  }

  frame = frames_queue.front().clone();
  frames_queue.pop();

  return 0;
}

void Camera::_update(void){

  while(true){

    Mat frame;
    bool grabbed;

    grabbed = capture.read(frame);

    //check if got a new frame
    if(!grabbed){
      continue;
    }

    //locking for thread safety
    lock_guard<mutex> lock(frames_queue_mutex);
    //check if queue is full, if it is full dequeue one element
    if(frames_queue.size() >= Camera::CAMERA_MAX_QUEUE_SIZE){
      frames_dropped++;
      dbg_user_info("Dropping camera frame | total: " + to_string(frames_dropped));
      frames_queue.pop();
    }

    frames_queue.push(frame);

  }

}
