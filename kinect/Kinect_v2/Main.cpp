#include <iostream>
#include <opencv2/core/core.hpp>
//#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include "KinectBase.hpp"
#include "Global.hpp"

const static std::string WINDOWNAME = "The Kinect";	//

void onMouse(int event, int x, int y, int flags, void *param = NULL)
{
  m_event = event;
  m_x = x;
  m_y = y;
}

int main()
{
  std::unique_ptr<KinectSensor> kinect(new KinectSensor());

  cv::setUseOptimized(true);//最適化

  cv::Mat depthBufferMat(KINECT_HEIGHT, KINECT_WIDTH, CV_16UC1);				//depthの取得データ
  cv::Mat colorBufferMat(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);	//color取得データ
  cv::Mat coordinateMat(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC4);				//depth->color

  cv::namedWindow(WINDOWNAME);				//画面
  cv::setMouseCallback(WINDOWNAME, onMouse);	//マウスイベント

  int mode = 1;//描画モード

  while (1){
    bool depthFlag, colorFlag;//各データの取得フラグ

    //モード共通の処理
    depthFlag = kinect->updateDepth(&depthBufferMat);
    colorFlag = kinect->updateColor(&colorBufferMat);

    //モード別の処理
    switch (mode) {
      case 1://depth表示
        if (depthFlag) {
          kinect->drawDepth(depthBufferMat, WINDOWNAME);	//
        }
        break;
      case 2://color表示
        if (colorFlag)
          kinect->drawColor(colorBufferMat, WINDOWNAME);	//
        break;
      case 3://depth->color
        if (depthFlag&&colorFlag) {
          kinect->create_rgbd(depthBufferMat, colorBufferMat, &coordinateMat);
          cv::imshow(WINDOWNAME, coordinateMat);
        }
        break;
      default:
        break;
    }

    //フレームの解放
    kinect->frameRelease();

    //描画モードの切り替え
    int key = cv::waitKey(1);
    if (key >= '0' && key <= '9') mode = key - '0';
    else if (key == 'q') break;
  }

  //終了処理
  cv::destroyAllWindows();

  return 0;
}
