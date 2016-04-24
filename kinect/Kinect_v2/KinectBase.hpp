#ifndef _KINECT_
#define _KINECT_

#include <opencv2/core/core.hpp>
#include<kinect.h>

const int KINECT_HEIGHT = 424;
const int KINECT_WIDTH = 512;
const int KINECT_COLOR_WIDTH = 1920;
const int KINECT_COLOR_HEIGHT = 1080;

class KinectSensor{
public:
	KinectSensor(){ initialize(); }
	~KinectSensor(){}

	//更新 whileの先頭とかで行う
	//trueなら更新成功、falseなら失敗
	bool updateDepth(cv::Mat* bufferMat/*, cv::Mat* depthMat*/);
	bool updateColor(cv::Mat* colorMat);

	//描画処理を行う
	void drawDepth(cv::Mat& drawDepthMap, const std::string& winname);
	void drawColor(cv::Mat& drawColorMap, const std::string&winname);

	//次フレームを手に入れるために開放
	void frameRelease();

	//colorの座標系をdepthに合わせ,rgb+depthの画像を得る
	void create_rgbd(cv::Mat& depth_im, cv::Mat& rgb_im, cv::Mat* rgbd_im);

	//depth座標->カメラ座標へ
	int depthToWorld(
		const cv::Point3d &imgPoint,
		cv::Point3d &worldPoint);
	//カメラ座標->depth座標へ
	int worldToDepth(
		const cv::Point3d &worldPoint,
		cv::Point3d &imgPoint);
private:
	//初期化
	int initialize();
};

#endif
