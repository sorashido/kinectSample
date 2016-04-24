#include <opencv2/core/core.hpp>
#include "Global.hpp"

//なるべく使わないように設計するつもりだけど、便利なので取りあえず用意

//マウスイベント
int m_x = 0, m_y = 0, m_event = 0;

//世界座標
//新しいブランチ
cv::Point3d worldPoint = { 0 };