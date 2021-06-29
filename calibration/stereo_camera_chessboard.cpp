#include "Camera.hpp"

#include <ctime>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

#ifdef DEBUG
  #define dbg_print(x) cout << "DEBUG: " << x << endl;
#else
  #define dbg_print(x)
#endif

static int capture_stereo_frames(sv::Camera &left,
                                  sv::Camera &right,
                                  Mat &out_left,
                                  Mat &out_right,
                                  bool preview = false
                                )
{

  int status;
  Mat color_left, color_right;


  status = left.read(color_left);

  if(status){
    return -1;
  }

  status = right.read(color_right);

  if(status){
    return -1;;
  }

  cvtColor(color_left, out_left, COLOR_BGR2GRAY);
  cvtColor(color_right, out_right, COLOR_BGR2GRAY);

  if(preview){
    Mat frame;
    hconcat(out_left,out_right,frame);
    imshow("preview",frame);
    waitKey(1);
  }

  return 0;
}

static void calc_board_corner_positions(Size board_size,
                                        float square_size,
                                        vector<Point3f>& obj)
{
  for (int i = 0; i < board_size.height; i++){
    for (int j = 0; j < board_size.width; j++){
      obj.push_back(Point3f(j*square_size, i*square_size, 0));
      dbg_print("x: " +to_string(j*square_size)+
                " y: "+to_string(i*square_size));
    }
  }
}

static void capture_stereo_chessboard(Size chessboard_size,
                                      float square_size,
                                      Mat left, Mat right,
                                      vector<vector<Point3f>> &obj_points,
                                      vector<vector<Point2f>> &img_points_left,
                                      vector<vector<Point2f>> &img_points_right,
                                      bool preview = true
                                            ){


  bool found_left, found_right;
  vector<Point2f> corners_left, corners_right;

  found_left = findChessboardCorners(left,
                                chessboard_size,
                                corners_left,
                                CALIB_CB_ADAPTIVE_THRESH +
                                CALIB_CB_NORMALIZE_IMAGE +
                                CALIB_CB_FAST_CHECK);

  found_right = findChessboardCorners(right,
                                chessboard_size,
                                corners_right,
                                CALIB_CB_ADAPTIVE_THRESH +
                                CALIB_CB_NORMALIZE_IMAGE +
                                CALIB_CB_FAST_CHECK);

  if(found_left && found_right){

    cornerSubPix(left, corners_left, Size(11, 11), Size(-1, -1),
                  TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));



    cornerSubPix(right, corners_right, Size(11, 11), Size(-1, -1),
                  TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    //check if checkerboard is steady for several seconds
    static float p_point_first_x, p_point_last_x = 0;
    static long long int p_time = 0;
    static int saved_objs = 0;
    float delta_first, delta_last;


    delta_first = p_point_first_x - corners_left.front().x;
    delta_last = p_point_last_x - corners_left.back().x;

    //dbg_print(delta_first);

    if(abs(delta_first) > 5 && abs(delta_last) > 5 && (time(0) - p_time) > 2){
        dbg_print("Saving " + to_string((time(0) - p_time)));
        p_time = time(0);
        p_point_first_x = corners_left.front().x;
        p_point_last_x = corners_left.back().x;

        saved_objs++;
    }

    if(preview){

      Mat colot_left_frame, colot_right_frame;
      Mat concat_frame;

      cvtColor(left, colot_left_frame, COLOR_GRAY2BGR);
      cvtColor(right, colot_right_frame, COLOR_GRAY2BGR);

      drawChessboardCorners(colot_left_frame, chessboard_size,
                            Mat(corners_left), found_left);
      drawChessboardCorners(colot_right_frame, chessboard_size,
                            Mat(corners_right), found_right);


      hconcat(colot_left_frame, colot_right_frame, concat_frame);

      putText(concat_frame,
          "Saved: " + to_string(saved_objs),
          Point(10,40), // Coordinates
          FONT_HERSHEY_SIMPLEX, // Font
          1.0, // Scale. 2.0 = 2x bigger
          Scalar(0,255,0), // BGR Color
          2, // Line Thickness (Optional)
          LINE_AA); // Anti-alias (Optional, see version note)

      imshow("preview",concat_frame);
      waitKey(1);
    }

    // vector<Point3f> obj;
    // calc_board_corner_positions(chessboard_size, square_size, obj);
    //
    // obj_points.push_back(obj);
    //
    // img_points_left.push_back(corners_left);
    // img_points_right.push_back(corners_left);




    // update_img_obj_points(chessboard_size,
    //                       chessboard_square_size,
    //                       corners_left,
    //                       obj_points_right,
    //                       img_point_right);
  }

}

static void update_img_obj_points(Size boardSize,
                                      float squareSize,
                                      vector<Point2f> corners,
                                      vector<vector<Point3f>>& obj_points,
                                      vector<vector<Point2f>>& img_points)
{

  vector<Point3f> obj;

  for (int i = 0; i < boardSize.height; i++){
    for (int j = 0; j < boardSize.width; j++){
      obj.push_back(Point3f(j*squareSize, i*squareSize, 0));
      dbg_print("x: " +to_string(j*squareSize)+
                " y: "+to_string(i*squareSize));
    }
  }

  img_points.push_back(corners);
  obj_points.push_back(obj);

}

int main(void){

  sv::Camera cam_left, cam_right;
  //internal number of corners in the chessboard
  Size chessboard_size(9,6);
  float chessboard_square_size = 23;
  vector<vector<Point3f>> obj_points_left(1), obj_points_right(1);
  vector<vector<Point2f>> img_point_left, img_point_right;

  if(cam_left.open(0)){
    return -1;
  };

  if(cam_right.open(2)){
    return -1;
  };


  while (true){

    Mat grey_left, grey_right;
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> img_points_left, img_points_right;

    if(capture_stereo_frames(cam_left, cam_right, grey_left, grey_right, false)){
      continue;
    }

    capture_stereo_chessboard(chessboard_size,
                              chessboard_square_size,
                              grey_left, grey_right,
                              object_points,
                              img_points_left,
                              img_points_right,
                              true);

    //
    // pattern_found = findChessboardCorners(grey_left,
    //                                       chessboard_size,
    //                                       corners_left,
    //                                       CALIB_CB_ADAPTIVE_THRESH +
    //                                       CALIB_CB_NORMALIZE_IMAGE +
    //                                       CALIB_CB_FAST_CHECK);
    //
    // if(pattern_found){
    //   cornerSubPix(grey_left, corners_left, Size(11, 11), Size(-1, -1),
    //                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    //
    //   update_img_obj_points(chessboard_size,
    //                         chessboard_square_size,
    //                         corners_left,
    //                         obj_points_left,
    //                         img_point_left);
    // }
    //
    // drawChessboardCorners(color_left, chessboard_size,
    //                       Mat(corners_left), pattern_found);
    //
    //
    // pattern_found = findChessboardCorners(grey_right,
    //                                       chessboard_size,
    //                                       corners_right,
    //                                       CALIB_CB_ADAPTIVE_THRESH +
    //                                       CALIB_CB_NORMALIZE_IMAGE +
    //                                       CALIB_CB_FAST_CHECK);
    //
    // if(pattern_found){
    //   cornerSubPix(grey_right, corners_right, Size(11, 11), Size(-1, -1),
    //                 TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    //
    //   update_img_obj_points(chessboard_size,
    //                         chessboard_square_size,
    //                         corners_right,
    //                         obj_points_right,
    //                         img_point_right);
    // }
    //
    // drawChessboardCorners(color_right, chessboard_size,
    //                       Mat(corners_right), pattern_found);
    //
    // hconcat(color_left, color_right, frame_out);
    //
    //
    // // objectPoints[0][chessboard_size.width - 1].x = objectPoints[0][0].x + grid_width;
    // // newObjPoints = objectPoints[0];
    //
    // // objectPoints.resize(imagePoints.size(),objectPoints[0]);
    // imshow("stereo_calibration_preview",frame_out);
    //
    // waitKey(1);
  }

  return 0;
}
