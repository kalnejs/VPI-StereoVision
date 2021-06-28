#include <string>
#include <thread>
#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

namespace sv{

using namespace std;
using namespace cv;

class Camera {

  const int CAMERA_MAX_QUEUE_SIZE = 4;

  public:
    Camera();
    ~Camera();

    static string gstreamer_pipeline (int id, int mode, int capture_width,
                                      int capture_height, int display_width,
                                      int display_height, int framerate,
                                      int flip_method) {
      return "nvarguscamerasrc sensor-id="+to_string(id)+
        " sensor-mode="+to_string(mode)+
        " ! video/x-raw(memory:NVMM), width=(int)"+to_string(capture_width)+
        ", height=(int)"+to_string(capture_height)+
        ", format=(string)NV12, framerate=(fraction)"+to_string(framerate)+
        "/1 ! nvvidconv flip-method="+to_string(flip_method)+
        " ! video/x-raw, width=(int)"+to_string(display_width)+
        ", height=(int)" +to_string(display_height)+
        ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    int open(string pipeline);
    int open(int camera_index);
    int start(void);
    int read(Mat &frame);

  private:
    VideoCapture capture;
    queue<Mat> frames_queue;
    mutex frames_queue_mutex;
    int frames_dropped;
    thread capture_thread;
    void _update(void);
};

}
