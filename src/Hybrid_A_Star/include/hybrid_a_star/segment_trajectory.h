#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
//把混合A*规划出的路径 在方向变化点进行分割 依次为前进路段和后退路径。。。

// 判断当前向量和前一个向量的夹角是否改变
bool has_direction_changed(const vector<double>& pre_prev, const vector<double>& prev, const vector<double>& curr);

// 将初始轨迹分割成多个段落
vector<pair<vector<double>, int>> segment_trajectory(const vector<vector<double>>& trajectory);

// 拟合多项式函数
// x: 自变量数据
// y: 因变量数据
// order: 多项式次数
// coeffs: 返回拟合多项式系数的向量
// n: 使用的数据点数量
void polyfitBack(vector<double> x, vector<double> y, int order, VectorXd &coeffs, int n);

// 在最后一个点后面添加新点
void addPointsBack(vector<double> &x, vector<double> &y, double interval, int num_points, VectorXd coeffs);

// 在现有轨迹的基础上用order次多项式拟合,在最后一个点后面添加间隔距离为interval的num_points个新点
// n指定用于计算多项式拟合系数的数据点数量
// direction = 1 向后添加点
// direction = -1 向前添加点
void addTrendPoints(vector<double> &x, vector<double> &y, int order, double interval, int num_points, int n, int direction);

