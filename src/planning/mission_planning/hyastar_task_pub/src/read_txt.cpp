#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ros/package.h>

using namespace std;
//测试时使用
struct RoadAttribute{                      //道路属性结构体


    float velocity;

    float road_width;

    float aeb_front;
    float aeb_back;
    float aeb_left;
    float aeb_right;

    float detect_front;
    float detect_back;
    float detect_left;
    float detect_right;

    float start_s;
    float end_s;
};

vector<RoadAttribute> readDataFromFile(const string& filename)
{
    ifstream file(filename); // 打开文件
    vector<RoadAttribute> road_atr_vec;
    
    if (file.is_open())
    {
        string line;
        getline(file, line); // 跳过第一行

        while (getline(file, line)) // 逐行读取文件
        {
            stringstream ss(line);
            float v, w, f, b, l, r, df, db, dl, dr, s, e;
            if (ss >> v >> w >> f >> b >> l >> r >> df >> db >> dl >> dr >> s >> e) // 分离每行数据
            {
                RoadAttribute road_atr;
                road_atr.velocity = v;
                road_atr.road_width = w;
                road_atr.aeb_front = f;
                road_atr.aeb_back = b;
                road_atr.aeb_left = l;
                road_atr.aeb_right = r;
                road_atr.start_s = s;
                road_atr.end_s = e;
                road_atr_vec.push_back(road_atr);
            }
            else
            {
                cout << "Failed to parse line: " << line << endl;
            }
        }

        // 处理完数据后关闭文件
        file.close();
    }
    else
    {
        cout << "Failed to open file: " << filename << endl;
    }
    
    return road_atr_vec;
}

int main()
{
    string filepath = ros::package::getPath("your_package_name") + "/config/RoadAttri.txt";
    vector<RoadAttribute> road_atr_vec = readDataFromFile(filepath);
    
    // 输出读取的数据
    for (int i = 0; i < road_atr_vec.size(); i++)
cout << "velocity: " << road_atr_vec[i].velocity << ", width: " << road_atr_vec[i].road_width << ", front: " << road_atr_vec[i].aeb_front << ", back: " << road_atr_vec[i].aeb_back << ", left: " << road_atr_vec[i].aeb_left << ", right: " << road_atr_vec[i].aeb_right << ", start_s: " << road_atr_vec[i].start_s << ", end_s: " << road_atr_vec[i].end_s << endl;    {
        
    }
    
    return 0;
}

