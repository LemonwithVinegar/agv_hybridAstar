#include"hybrid_a_star/segment_trajectory.h"


void polyfitFront(vector<double> x, vector<double> y, int order, VectorXd &coeffs, int n) {
    MatrixXd X(n, order+1);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= order; j++) {
            X(i,j) = pow(x[i], j);
        }
    }

    coeffs = (X.transpose() * X).inverse() * X.transpose() * Map<VectorXd>(y.data(), n);
}

void addPointsFront(vector<double> &x, vector<double> &y, double interval, int num_points, VectorXd coeffs) {
    for (int i = 1; i <= num_points; i++) {
        double new_x = x.front() - interval;
        double new_y = 0;

        for (int j = 0; j < coeffs.size(); j++) {
            new_y += coeffs[j] * pow(new_x, j);
        }

        x.insert(x.begin(), new_x);
        y.insert(y.begin(), new_y);
    }
}


void polyfitBack(vector<double> x, vector<double> y, int order, VectorXd &coeffs, int n) {
    MatrixXd X(n, order+1);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= order; j++) {
            X(i,j) = pow(x[x.size()-n+i], j);
        }
    }

    coeffs = (X.transpose() * X).inverse() * X.transpose() * Map<VectorXd>(y.data() + y.size() - n, n);
}




// 在最后一个点后面添加新点
void addPointsBack(vector<double> &x, vector<double> &y, double interval, int num_points, VectorXd coeffs) {
    for (int i = 1; i <= num_points; i++) {
        double new_x = x.back() + interval;
        double new_y = 0;

        for (int j = 0; j < coeffs.size(); j++) {
            new_y += coeffs[j] * pow(new_x, j);
        }

        x.push_back(new_x);
        y.push_back(new_y);
    }
}

void addTrendPoints(vector<double> &x, vector<double> &y, int order, double interval, int num_points, int n, int direction) {
    VectorXd coeffs;

    n = n > x.size() ? x.size() : n;
    if (direction == 1) { // 向后添加点
        polyfitBack(x, y, order, coeffs, n);
        addPointsBack(x, y, interval, num_points, coeffs);
    }
    else if (direction == -1) { // 向前添加点
        polyfitFront(x, y, order, coeffs, n);
        addPointsFront(x, y, interval, num_points, coeffs);
    }
}



// 判断当前向量和前一个向量的夹角是否改变
bool has_direction_changed(const vector<double>& pre_prev, const vector<double>& prev, const vector<double>& curr) {
    double v1x = prev[0] - pre_prev[0];
    double v1y = prev[1] - pre_prev[1];
    double v2x = curr[0] - prev[0];
    double v2y = curr[1] - prev[1];
    double dot_product = v1x * v2x + v1y * v2y;
    double length1 = sqrt(v1x * v1x + v1y * v1y);
    double length2 = sqrt(v2x * v2x + v2y * v2y);
    double cosine = dot_product / (length1 * length2);
    if (cosine < 0) {
        return true;
    }
    return false;
}

// 将初始轨迹分割成多个段落
vector<pair<vector<double>, int>> segment_trajectory(const vector<vector<double>>& trajectory) {
    vector<pair<vector<double>, int>> result;
    bool prev_direction = has_direction_changed(trajectory[0], trajectory[1], trajectory[2]);
    int segment_start_index = 0;
    bool segment_direction = prev_direction;
    vector<pair<int, bool>> segments;
    int changed_count = 0;
  
    for (int i = 2; i < trajectory.size(); i++) {
        bool curr_direction = has_direction_changed(trajectory[i - 2], trajectory[i - 1], trajectory[i]);

        if (curr_direction) {
            segments.push_back(make_pair(segment_start_index, segment_direction));
            changed_count++;
            segment_start_index = i -1;
            segment_direction = curr_direction;
        }

        // prev_direction = curr_direction;

        if (i == trajectory.size() - 1) {
            segments.push_back(make_pair(segment_start_index, segment_direction));
        }
    }
    // 将所有段落提取出来，并记录方向
    for (int i = 0; i < segments.size(); i++) {
        int segment_start;
        int segment_end;
        segment_start = segments[i].first;
        segment_end = (i == segments.size() - 1) ? trajectory.size() - 1 : segments[i + 1].first;
        // 获取当前段落的方向
        bool segment_direction = segments[i].second;
        vector<double> segment;
        vector<double> x;
        vector<double> y;
        for (int j = segment_start; j <= segment_end; j++) {
            segment.push_back(trajectory[j][0]);
            segment.push_back(trajectory[j][1]);
            x.push_back(trajectory[j][0]);
            y.push_back(trajectory[j][1]);
        }
        int direction;
        if (i %2 == 0) {
            direction = 1;
        } else {
            direction = -1;
        }

        /***********************************************/
        //在每一段轨迹点后面拟合添加几个符合趋势的点 
        // 在现有轨迹的基础上用二次多项式拟合,在最后一个点后面添加间隔距离为interval的num_points个新点
        // n指定用于计算多项式拟合系数的数据点数量
        // int n = 5;
        // double interval = 0.2;
        // int num_points = 1; //在轨迹点后面加入点的数量
        // addTrendPoints(x, y, 2, interval, num_points, n, 1);
        // polyfit_add_points_end(x, y, 2, interval, num_points, n);
        // for (int i = x.size() - num_points; i < x.size(); i++) {
        //     // cout << "(" << x[i] << ", " << y[i] << ")";
        //     // if (i < x.size()-1) {
        //     //     cout << ", ";
        //     // }
        //     segment.push_back(x[i]);
        //     segment.push_back(y[i]);

        // }
        /*******************************************************/

        // 在第一段轨迹的最开头插入一个点
        // addTrendPoints(x, y, 2, 0.3, 1, 5, -1);

        // if (i == 0) {
        //     segment.insert(segment.begin(), y[0]);
        //     segment.insert(segment.begin(), x[0]);
        // }
        // 将当前段落的方向和轨迹点存储为一个 pair，并将该 pair 存储到结果 vector 中
        result.push_back(make_pair(segment, direction));
    }
    // 返回所有段落及其方向的结果
    return result;
}

// int main() {
//     vector<vector<double>> trajectory = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}};
//     // vector<vector<double>> trajectory = {{0, 0}, {1, 1}, {2, 2}, {3, 1}, {4, 1}, {4, 2}, {5, 2}, {6, 2},{7,2}};
//     vector<pair<vector<double>, int>> segments = segment_trajectory(trajectory);

//     // 输出分段后的结果
//     for (int i = 0; i < segments.size(); i++) {
//         cout << "Segment " << i + 1 << ": ";
//         for (int j = 0; j < segments[i].first.size(); j += 2) {
//             cout << "(" << segments[i].first[j] << ", " << segments[i].first[j + 1] << ") ";
//         }
//         cout << "Direction: " << segments[i].second << endl;
//     }

//     return 0;
// }