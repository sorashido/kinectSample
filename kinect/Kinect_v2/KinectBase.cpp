#include <iostream>
#include <Windows.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <kinect.h>
#include "KinectBase.hpp"
#include "Global.hpp"

static HRESULT hResult;	//結果の判定
static IKinectSensor* pSensor=nullptr;//センサー
//depth
static IDepthFrameSource* pDepthSource=nullptr;
static IDepthFrameReader* pDepthReader=nullptr;
static IDepthFrame *pDepthFrame = nullptr;
//color
static IColorFrameSource *pColorSource = nullptr;
static IColorFrameReader *pColorReader = nullptr;
static IColorFrame *pColorFrame = nullptr;

static ICoordinateMapper* pCoordinateMapper = nullptr;//座標変換

  template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
  if (pInterfaceToRelease != NULL){
    pInterfaceToRelease->Release();
    pInterfaceToRelease = NULL;
  }
}
void KinectSensor::frameRelease()
{
  //次のフレームを獲得するために開放
  SafeRelease(pDepthFrame);
  SafeRelease(pColorFrame);
}

int KinectSensor::initialize()
{
  /*Sensor*/
  hResult = S_OK;

  hResult = GetDefaultKinectSensor(&pSensor);
  if (FAILED(hResult)){
    std::cerr << "Error:GetDefaultKinectSensor" << std::endl;
    return 0;
  }
  hResult = pSensor->Open();
  if (FAILED(hResult)){
    std::cerr << "Error:Open()" << std::endl;
    return 0;
  }
  /*color and depth*/
  //Source
  hResult = pSensor->get_DepthFrameSource(&pDepthSource);
  if (FAILED(hResult)){
    std::cerr << "Error:get_DepthFrameSource()" << std::endl;
    return 0;
  }
  hResult = pSensor->get_ColorFrameSource(&pColorSource);
  if (FAILED(hResult)){
    std::cerr << "Error:get_ColorFrameSource()" << std::endl;
    return 0;
  }
  //Reader
  hResult = pDepthSource->OpenReader(&pDepthReader);  //
  if (FAILED(hResult)){
    std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
    return 0;
  }
  hResult = pColorSource->OpenReader(&pColorReader);  //
  if (FAILED(hResult)){
    std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
    return 0;
  }

  /*座標変換用*/
  HRESULT hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
  if (FAILED(hResult)) {
    std::cerr << "ERROR" << std::endl;
    return 0;
  }
  return 1;
}

//depthの更新
bool KinectSensor::updateDepth(cv::Mat* buffer/*, cv::Mat* depth*/)
{
  static  unsigned int bufferSize = KINECT_WIDTH*KINECT_HEIGHT*sizeof(unsigned short);
  hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
  if (SUCCEEDED(hResult)){
    hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast< UINT16**>(&buffer->data));
    if (SUCCEEDED(hResult))return true;
  }
  return false;
}

//カラーの更新
bool KinectSensor::updateColor(cv::Mat*buffer)
{
  static unsigned int bufferSize = KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT * 4 * sizeof(unsigned char);//

  hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
  if (SUCCEEDED(hResult)){
    hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>((*buffer).data), ColorImageFormat::ColorImageFormat_Bgra);
    if (SUCCEEDED(hResult))return true;
  }
  return false;
}

//depth画像の表示
void KinectSensor::drawDepth(cv::Mat& drawDepthMap, const std::string& winname)
{
  //データが入っていないなら何もしない
  if (!drawDepthMap.data)return;

  cv::Mat depthTmp;//depthをカラーでの描画に変換する際の中間Mat
  drawDepthMap.convertTo(depthTmp, CV_8U, -255.0f / 8000.0f, 255.0f);

  //カラーに
  cv::Mat paintImg;//描画用
  cv::cvtColor(depthTmp, paintImg, CV_GRAY2BGR);

  //描画文字
  char str[64];
  sprintf_s(str, "%4d,%4d,%4d,%4d", drawDepthMap.at<short>(m_y, m_x),(int)worldPoint.x,(int)worldPoint.y,(int)worldPoint.z);
  cv::putText(paintImg, str, cv::Point(m_x, m_y), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 2, CV_AA);

  //描画
  cv::imshow(winname, paintImg);
}

//カラー画像の表示
void KinectSensor::drawColor(cv::Mat& drawColorMap, const std::string& winname)
{
  //データに何も入っていないなら更新しない
  if (!drawColorMap.data)return;

  //描画用
  cv::Mat paintImg(KINECT_COLOR_HEIGHT/2, KINECT_COLOR_WIDTH/2, CV_8UC4);

  cv::resize(drawColorMap, paintImg, cv::Size(), 0.5, 0.5);//リサイズ
  cv::imshow(winname,paintImg);	//表示
}

int KinectSensor::depthToWorld(
    const cv::Point3d &imgPoint,
    cv::Point3d &worldPoint)
{
  DepthSpacePoint depthPoint;
  CameraSpacePoint cameraSpacePoint;

  depthPoint.X = imgPoint.x;
  depthPoint.Y = imgPoint.y;
  hResult = pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, imgPoint.z, &cameraSpacePoint);

  worldPoint.x = cameraSpacePoint.Z*1e+3;
  worldPoint.y = cameraSpacePoint.X*1e+3;
  worldPoint.z = cameraSpacePoint.Y*1e+3;

  return 0;
}

int KinectSensor::worldToDepth(
    const cv::Point3d& worldPoint,
    cv::Point3d& imgPoint)
{
  DepthSpacePoint depthPoint;
  CameraSpacePoint cameraSpacePoint;

  cameraSpacePoint.X = worldPoint.y*1e-3;
  cameraSpacePoint.Y = worldPoint.z*1e-3;
  cameraSpacePoint.Z = worldPoint.x*1e-3;

  hResult = pCoordinateMapper->MapCameraPointToDepthSpace(cameraSpacePoint, &depthPoint);

  imgPoint.x = depthPoint.X;
  imgPoint.y = depthPoint.Y;

  return 0;
}

//depth->color
void KinectSensor::create_rgbd(cv::Mat& depth_im, cv::Mat& rgb_im, cv::Mat* rgbd_im)
{
  // Depth座標系に対応するカラー座標系の一覧を取得する
  std::vector<ColorSpacePoint> colorSpace(KINECT_WIDTH * KINECT_HEIGHT);
  pCoordinateMapper->MapDepthFrameToColorSpace(
      KINECT_WIDTH*KINECT_HEIGHT, (UINT16*)depth_im.data, colorSpace.size(), &colorSpace[0]);

  for (int y = 0; y < KINECT_HEIGHT; y++) {
    for (int x = 0; x < KINECT_WIDTH; x++) {
      if (depth_im.at<short>(y, x)>0) {
        ColorSpacePoint colorPoint = colorSpace[y*KINECT_WIDTH + x];
        int colorX = (int)colorPoint.X;
        int colorY = (int)colorPoint.Y;
        if ((colorX >= 0) && (colorX < KINECT_COLOR_WIDTH) && (colorY >= 0) && (colorY < KINECT_COLOR_HEIGHT)) {
          rgbd_im->at<cv::Vec4b>(y, x) = rgb_im.at<cv::Vec4b>(colorY, colorX);
        }
      }
      else {
        rgbd_im->at<cv::Vec4b>(y, x) = 0;
      }
    }
  }
}
