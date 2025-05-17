//2020-06-18 调整18km/h以下的关联阈值，使得低速情况下目标不反复横跳，但是在35km/h以上存在目标关联不上的情况
#include "data_association.h"
// #include "kalman.h"
#include <math.h>
// int flag_img = 0;
// std::vector < target>Z_k_img;
// std::vector < target>Z_k_lidar;

//单个目标
target::target(){
    vx = 0.0;         //目标x方向速度，m/s
    vy = 0.0;         //目标y方向速度,m/s
    v = 0.0;          //目标速度，km/h
    wide = 0.0;        //目标宽
    deep = 0.0;      //目标深度
    height = 0.0;     //目标高度
    x_pos = 0.0;       //目标x坐标
    y_pos = 0.0;       //目标y坐标
    s_pos = 0.0;       //目标s坐标
    d_pos = 0.0;       //目标d坐标
    tar_associated = 0;            //预测目标 未被观测到
    // observed_flag = 0;           //观测值未使用了,当前未被使用的观测值就是新的目标或者杂波
    // flag_appear = 0;               //目标出现
}

target::~target(){}
target_pre_his::target_pre_his(){ 
    id = 0; 
    flag_new_target = 0;
    //kalman滤波参数初值
    A << 1, 0, 0.1, 0,			//状态转移矩阵
         0, 1, 0, 0.1,
         0, 0, 1, 0,
         0, 0, 0, 1;
    B << 0, 0, 0, 0;            //控制量
    H << 1, 0, 0, 0,			//观测矩阵
         0, 1, 0, 0;
    Q << MatrixXd::Identity(4,4)*(1e-4);   //预测过程噪声协方差
    R << MatrixXd::Identity(2,2)*(1e-3);   //测量过程噪声协方差
}
target_pre_his::~target_pre_his(){}


Data_association::Data_association()
{
    frame_count = 0;
    current_time = 0.0;
    first_flag = 0;
    fout2.open("/home/luck2024/Desktop/object_pos.txt");
    fout1.open("/home/luck2024/Desktop/origin.txt");
    fout2<<"帧数:     "<<"id     "<<"size     "<<"tar_associated     "<<"flag_new_target      "<<"宽度     "<<"深度    "<<"高度    "<<"x_pos     "<<"y_pos     "<<"vy     "<<"v(km/h)     "<<"delta_time     "<<std::endl;
    fout1<<"帧数:    "<<"x_pos     "<<"y_pos     "<<"wide     "<<"deep     "<<"height     "<<std::endl;
}
Data_association::~ Data_association(){}

// //图像数据
// void Data_association::set_raw_ImageObject(const custom_msgs::ImageObjectArrayConstPtr& msgin)
// {
//     curruentimagearraydata = *msgin;//这里LidarRawObjectArray 相当于是一个类，类里面有objs的对象，求长度的话就objs.size();
//     Z_k_img.clear();
//     for(uint i = 0;i < curruentimagearraydata.objs.size();i++)
//     {
//         target te1;
//         te1.length = curruentimagearraydata.objs[i].lwh.x;
//         te1.wide = curruentimagearraydata.objs[i].lwh.y;
//         te1.x_pos = curruentimagearraydata.objs[i].x_pos;
//         te1.y_pos = curruentimagearraydata.objs[i].y_pos;
//         Z_k_img.push_back(te1);
//     }
// }

//雷达数据保存
void Data_association::set_raw_LidarRawObject(const custom_msgs::LidarRawObjectArrayConstPtr& msgin)
{
    curruentlidararraydata = *msgin;//这里LidarRawObjectArray 相当于是一个类，类里面有objs的对象，求长度的话就objs.size();
    Z_k.clear();
    ++frame_count;
    // std::cout<<"--------------------"<<"帧数: "<<frame_count<<"----------------------"<<std::endl;
    //保存雷达数据,只保存了雷达的x,y,宽度,8个点
    for(uint i = 0;i < curruentlidararraydata.objs.size();i++)
    {
        // if(curruentlidararraydata.objs[i].x_pos>0  &&  curruentlidararraydata.objs[i].x_pos < 12 
        //  && curruentlidararraydata.objs[i].y_pos>-3  &&  curruentlidararraydata.objs[i].y_pos < 5)
        // {   
            target te1;
            te1.deep = curruentlidararraydata.objs[i].lwh.x;
            te1.wide = curruentlidararraydata.objs[i].lwh.y;
            te1.height = curruentlidararraydata.objs[i].lwh.z;
            te1.x_pos = curruentlidararraydata.objs[i].x_pos;
            te1.y_pos = curruentlidararraydata.objs[i].y_pos;
            for (uint m = 0;m < 8; m++)
            {
                te1.points[m][0] = curruentlidararraydata.objs[i].bbox_point[m].x;
                te1.points[m][1] = curruentlidararraydata.objs[i].bbox_point[m].y;
                te1.points[m][2] = curruentlidararraydata.objs[i].bbox_point[m].z;
            // std::cout<<"te1.points["<<m<<"][0]:"<<te1.points[m][0]<<" te1.points["<<m<<"][1]:"<<te1.points[m][1]<<" te1.points["<<m<<"][2]:"<<te1.points[m][2]<<std::endl;
            }
            fout1<<"  frame "<<frame_count<<"     "<<te1.x_pos<<"     "<<te1.y_pos<<"     "<<te1.wide<<"     "<<te1.deep<<"    "<<te1.height<<std::endl;
            // std::cout<<"  原始点:"<<te1.x_pos<<"  "<<te1.y_pos<<"  "<<te1.wide<<"  "<<te1.deep<<"  "<<te1.height<<std::endl;
            Z_k.push_back(te1);
        // }
    }
    // std::cout<<std::endl;
    // std::cout<<"Z_k size:"<<Z_k.size()<<std::endl;
    // //过滤掉目标中deep/wide>8的物体
    // for(auto it = Z_k.begin(); it != Z_k.end();)
    // {
    //      if(it->deep/it->wide>8.0)
    //      {
    //         it = Z_k.erase(it);
    //             continue;
    //      }
    //     //  if(it->wide <  = 1 && it->deep >= 2.5)
    //     //  {
    //     //     it = Z_k.erase(it);
    //     //         continue;
    //     //  }
    //     //  if(it->deep>2 && it->height < 1)
    //     //  {
    //     //     it = Z_k.erase(it);
    //     //         continue;
    //     //  }
    //     //  std::cout<<"原始点--wide:"<<it->wide<<"  deep:"<<it->deep<<"  height"<<it->height<<std::endl;
    //     it++;
    // }
    
}

// void Data_association::get_all_data()
// {
//     Z_k.clear();
//     for(uint i = 0;i < Z_k_img.size();i++)
//         Z_k.push_back(Z_k_img[i]);
//     for(uint i = 0;i < Z_k_lidar.size();i++)
//         Z_k.push_back(Z_k_lidar[i]);    
// }

// //使用目标的位置关系先过滤掉太近的物体
// void Data_association::initial_filter()
// {
//     vector <float>temp_x;
//     vector <float>temp_y;
//     for(uint i = 0;i < Z_k.size();i++)
//         temp_x.push_back(Z_k[i].x_pos);
//     sort(temp_x.begin(),temp_x.end());
//     for(uint i = 0;i < Z_k.size();i++)
//         temp_y.push_back(Z_k[i].y_pos);
//     sort(temp_y.begin(),temp_y.end());

//     auto it_x = temp_x.begin();
//     auto it_y = temp_y.begin();
//     for(;it_x  != temp_x.end();it_x++,it_y++)
//     {
//         if(abs(*it_x-*(it_x+1))  <=  0.3  &&  abs(*it_y-*(it_y+1))  <=  0.3 )
//         {
//             for(auto it1 = Z_k.begin();it1  != Z_k.end();it1++)
//             {
//                 if(*it_x == it1->x_pos)
//                 {
//                     it1 = Z_k.erase(it1);
//                     break;
//                 }
//             }
//         }
//     }

//     // std::cout<<" 原始点after_initial:";
//     // for(auto it1 = Z_k.begin();it1! = Z_k.end();it1++)
//     // {
//     //     std::cout<<" x_pos:"<<it1->x_pos<<" y_pos:"<<it1->y_pos;
//     // }
//     // std::cout<<std::endl;
// }

/*******************************************************初始化过程*********************************************************/
void Data_association::init()
{
    if(result.empty() && first_flag == 0)          //第一帧有没有目标均进入if，有了目标之后不再进入
    {
        for(uint i = 0;i < Z_k.size();i++)
        {
            target temp1;
            target_pre_his new_initial;
            temp1.tar_associated = 1;
            temp1.wide = Z_k[i].wide;
            temp1.deep = Z_k[i].deep;
            temp1.height = Z_k[i].height;
            temp1.x_pos = Z_k[i].x_pos;
            temp1.y_pos = Z_k[i].y_pos;
            for (uint m = 0;m < 8; m++)
            {
                temp1.points[m][0] = Z_k[i].points[m][0];
                temp1.points[m][1] = Z_k[i].points[m][1];
                temp1.points[m][2] = Z_k[i].points[m][2];
            }
            new_initial.history.push_back(temp1);
            new_initial.predict.push_back(temp1);
            result.push_back(new_initial);
            first_flag = 1;
        }
    }
}

/*******************************************************出现和消失判断*********************************************************/
void Data_association::disppear_tar_judge(std::vector < target_pre_his> &result)
{
    int sum_tar_associated = 0;     //判定目标出现或消失的帧数
    if(!result.empty())
    {
            auto it = result.begin();
            for(int i = 0;i < result.size() && it  != result.end();i++)
            {
                //利用10帧位置来计算速度
                if(result[i].history.size() >= 10)
                {
                    result[i].history.back().vx = ((*(result[i].history.rbegin())).x_pos-(*(result[i].history.rbegin()+9)).x_pos)/save_time(fabs(delta_time),9);
                    result[i].history.back().vy = ((*(result[i].history.rbegin())).y_pos-(*(result[i].history.rbegin()+9)).y_pos)/save_time(fabs(delta_time),9);
                    result[i].history.back().v = sqrt(result[i].history.back().vx*result[i].history.back().vx+result[i].history.back().vy*result[i].history.back().vy)*3.6; 
                }


                // if(result[i].history.size() <  = 5)                     //防止后面预测时flag_new_target变乱
                //     result[i].flag_new_target = 0;
                // float v_5 = 0.0;
                // float v_7 = 0.0;
                // //利用7帧位置来计算速度
                // if(result[i].history.size() >= 7)
                // {
                //     result[i].history.back().vx = ((*(result[i].history.rbegin())).x_pos-(*(result[i].history.rbegin()+7)).x_pos)/save_time(fabs(delta_time),7);
                //     result[i].history.back().vy = ((*(result[i].history.rbegin())).y_pos-(*(result[i].history.rbegin()+7)).y_pos)/save_time(fabs(delta_time),7);
                //     result[i].history.back().v = sqrt(result[i].history.back().vx*result[i].history.back().vx+result[i].history.back().vy*result[i].history.back().vy)*3.6; 
                //     v_7 = result[i].history.back().v;
                // }
                // //利用5帧位置来计算速度
                // if(result[i].history.size() >= 5)
                // {
                //     result[i].history.back().vx = ((*(result[i].history.rbegin())).x_pos-(*(result[i].history.rbegin()+4)).x_pos)/save_time(fabs(delta_time),4);
                //     result[i].history.back().vy = ((*(result[i].history.rbegin())).y_pos-(*(result[i].history.rbegin()+4)).y_pos)/save_time(fabs(delta_time),4);
                //     result[i].history.back().v = sqrt(result[i].history.back().vx*result[i].history.back().vx+result[i].history.back().vy*result[i].history.back().vy)*3.6; 
                //     v_5 = result[i].history.back().v;
                // }

                // if(result[i].history.size() >= 7)
                //     result[i].history.back().v = (v_5 + v_7)/2.0;
                // else result[i].history.back().v = v_5;

                //4帧中出现2帧即判定为新目标
                if(result[i].history.size() >= 4)
                {
                    sum_tar_associated = 0;
                    for(auto it1 = result[i].history.rbegin();it1 != result[i].history.rbegin()+4;it1++)
                        sum_tar_associated += it1->tar_associated;     
                    if(sum_tar_associated >= 2 && (result[i].flag_new_target == 0))
                    {
                        result[i].flag_new_target = 1;
                        result[i].id = ++tarnum;               //确认为新目标之后再分配id，否则一律为0
                        // std::cout<<"新目标:  "<<result[i].id<<std::endl;
                        sum_tar_associated = 0;
                        continue;
                    }
                }

                //17帧中消失15帧判定为消失
                if(result[i].history.size() >= 17)
                {
                    sum_tar_associated = 0;
                    for(auto it1 = result[i].history.rbegin();it1 != result[i].history.rbegin()+17;it1++)
                        sum_tar_associated += it1->tar_associated;
                    if(sum_tar_associated  <= 3)
                    {
                        // std::cout<<"shanchuanqian size = "<<result.size()<<std::endl;
                        // std::cout<<"shanchumubiao:  "<<result[i].history.back().id<<std::endl;
                        
                        it = result.erase(it);
                        i--;     //这里i可能为负
                        // std::cout<<"shanchuhou size = "<<result.size()<<std::endl;
                        sum_tar_associated = 0;
                        continue;   
                    }
                }
                sum_tar_associated = 0;
                it++;
            }
    }
}

/*******************************************************预测*********************************************************/
void Data_association::predict()
{
    for(uint j = 0;j < result.size() && first_flag == 0;j++)
    {
        target predict_temp;
        predict_temp.x_pos = result[j].history.back().x_pos+result[j].history.back().vx*delta_time;            //对目标的状态预测
        predict_temp.y_pos = result[j].history.back().y_pos+result[j].history.back().vy*delta_time;
        predict_temp.vx = result[j].history.back().vx;
        predict_temp.vy = result[j].history.back().vy;
        predict_temp.v = result[j].history.back().v;
        predict_temp.wide = result[j].history.back().wide;
        predict_temp.deep = result[j].history.back().deep;
        predict_temp.height = result[j].history.back().height;
        predict_temp.tar_associated = 0;
        for (uint m = 0;m < 8; m++)
        {
            predict_temp.points[m][0] = result[j].history.back().points[m][0];
            predict_temp.points[m][1] = result[j].history.back().points[m][1];
            predict_temp.points[m][2] = result[j].history.back().points[m][2];
        }
        // std::cout<<"id:"<<result[j].id<<"   predict_temp.x_pos:"<<predict_temp.x_pos<<"  predict_temp.y_pos:"<<predict_temp.y_pos
        // <<"  predict_temp.wide:"<<predict_temp.wide<<"  predict_temp.deep:"<<predict_temp.deep<<std::endl;
        result[j].predict.push_back(predict_temp);
        
    }
}

/*****************************************************关联*******************************************************************/
void Data_association::associate_process()
{
    disppear_tar_judge(result);
    predict();
    std::vector<int> confirm;             //保存已经匹配的观测值的下标
    
    float distance = 0.0;
    for(uint i = 0;i < Z_k.size();i++)
        // std::cout<<"Z_k["<<i<<"]"<<Z_k[i].x_pos<<std::endl;

    for(uint i = 0;i < result.size() && first_flag == 0;i++)
    {
        std::vector<int> confirm_all;     //保存所有满足条件的下标索引
        float distance_min = 10.0;
        uint j = 0;
        // std::cout<<"wai for"<<std::endl;
        for(;j < Z_k.size();j++)  
        {
            // std::cout<<"result[i].predict.back().x_pos:"<<result[i].predict.back().x_pos<<"  result[i].predict.back().wide:"<<result[i].predict.back().wide<<"  result[i].predict.back().deep:"<<result[i].predict.back().deep<<std::endl;
            // std::cout<<"Z_k[j].x_pos:"<<Z_k[j].x_pos<<" Z_k[j].wide:"<<Z_k[j].wide<<"  Z_k[j].deep:"<<Z_k[j].deep<<std::endl;
            // std::cout<<"now j = "<<j<<std::endl;
            if(!confirm.empty())
            {
                //检查下标j是否已经被匹配
                if(find(confirm.begin(),confirm.end(),j) != confirm.end())
                    {
                        // std::cout<<"进入confirm"<<std::endl;
                        continue;                                       //确认已经匹配上的目标不再参与匹配
                    }
            }
            //速度小于18km/h
            if(result[i].history.back().v < 18.0)
            {
                if(fabs(result[i].predict.back().x_pos-Z_k[j].x_pos) < 3.6  &&  fabs(result[i].predict.back().y_pos-Z_k[j].y_pos) < 3.6  
                && 
                (fabs(result[i].predict.back().wide-Z_k[j].wide) < 0.9 * result[i].predict.back().wide  || fabs(result[i].predict.back().deep-Z_k[j].deep) < 0.9 * result[i].predict.back().deep) 
                )
                {
                    float x_dif = result[i].predict.back().x_pos + 0.5 * result[i].history.back().vx * delta_time - Z_k[j].x_pos;
                    float y_dif = result[i].predict.back().y_pos + 0.5 * result[i].history.back().vy * delta_time - Z_k[j].y_pos;
                    distance = sqrt(x_dif * x_dif + y_dif * y_dif);
                    // std::cout<<"distance:"<<distance<<"  distance_min:"<<distance_min<<std::endl;
                    // float distance_mean_5 = 0.0;
                    // if(result[i].history.size() >= 5)
                    // {
                    //     float x_5 = ((*(result[i].history.rbegin())).x_pos - (*(result[i].history.rbegin()+4)).x_pos);
                    //     float y_5 = ((*(result[i].history.rbegin())).y_pos - (*(result[i].history.rbegin()+4)).y_pos);
                    //     distance_mean_5 = sqrt(x_5 * x_5 + y_5 * y_5);
                    // }
                    // else distance_mean_5 = 1.0;
                    // std::cout << "distance:" << distance << "  " << distance_mean_5 << std::endl;
                    // if(distance < distance_min && distance <= 6.0*distance_mean_5)             //只要有距离小于前面的的匹配,将其下标索引push_back
                    if(distance < distance_min) 
                    {
                        distance_min = distance;
                        confirm_all.push_back(j);       
                        // std::cout<<"j = "<<j<<std::endl;
                    }
                    // std::cout << "match " << i << std::endl;
                }
                
            }
            //速度在18km/h~31.8km/h
            if(result[i].history.back().v >= 18.0 && result[i].history.back().v < 31.8)
            {
                if(fabs(result[i].predict.back().x_pos-Z_k[j].x_pos) < 4.5  &&  fabs(result[i].predict.back().y_pos-Z_k[j].y_pos) < 4.5 
                //  && 
                // (fabs(result[i].predict.back().wide-Z_k[j].wide) < 0.8 * result[i].predict.back().wide  || fabs(result[i].predict.back().deep-Z_k[j].deep) < 0.8 * result[i].predict.back().deep) 
                )
                {
                    distance = sqrt((result[i].predict.back().x_pos-Z_k[j].x_pos)*(result[i].predict.back().x_pos-Z_k[j].x_pos)+
                                (result[i].predict.back().y_pos-Z_k[j].y_pos)*(result[i].predict.back().y_pos-Z_k[j].y_pos));
                    float distance_mean_5 = 0.0;
                    if(result[i].history.size() >= 5)
                    {
                        float x_5 = ((*(result[i].history.rbegin())).x_pos - (*(result[i].history.rbegin()+4)).x_pos);
                        float y_5 = ((*(result[i].history.rbegin())).y_pos - (*(result[i].history.rbegin()+4)).y_pos);
                        distance_mean_5 = sqrt(x_5 * x_5 + y_5 * y_5);
                    }
                    else distance_mean_5 = 1.0;
                    // std::cout << "distance:" << distance << "  " << distance_mean_5 << std::endl;
                    if(distance < distance_min && distance <= 6.0*distance_mean_5)             //只要有距离小于前面的的匹配,将其下标索引push_back
                    // if(distance < distance_min) 
                    {
                        distance_min = distance;
                        confirm_all.push_back(j);       
                        // std::cout<<"j = "<<j<<std::endl;
                    }
                }
            }
            //速度大于31.8km/h
            if(result[i].history.back().v >= 31.8)
            {
                if(fabs(result[i].predict.back().x_pos-Z_k[j].x_pos) < 6.5  &&  fabs(result[i].predict.back().y_pos-Z_k[j].y_pos) < 6.5  
                // && 
                // (fabs(result[i].predict.back().wide-Z_k[j].wide) < 0.5 * result[i].predict.back().wide  || fabs(result[i].predict.back().deep-Z_k[j].deep) < 0.5 * result[i].predict.back().deep) 
                )
                {
                    distance = sqrt((result[i].predict.back().x_pos-Z_k[j].x_pos)*(result[i].predict.back().x_pos-Z_k[j].x_pos)+
                                (result[i].predict.back().y_pos-Z_k[j].y_pos)*(result[i].predict.back().y_pos-Z_k[j].y_pos));
                    float distance_mean_5 = 0.0;
                    if(result[i].history.size() >= 5)
                    {
                        float x_5 = ((*(result[i].history.rbegin())).x_pos - (*(result[i].history.rbegin()+4)).x_pos);
                        float y_5 = ((*(result[i].history.rbegin())).y_pos - (*(result[i].history.rbegin()+4)).y_pos);
                        distance_mean_5 = sqrt(x_5 * x_5 + y_5 * y_5);
                    }
                    else distance_mean_5 = 1.0;
                    // std::cout << "distance:" << distance << "  " << distance_mean_5 << std::endl;
                    if(distance < distance_min && distance <= 6.0*distance_mean_5)             //只要有距离小于前面的的匹配,将其下标索引push_back
                    // if(distance < distance_min) 
                    {
                        distance_min = distance;
                        confirm_all.push_back(j);       
                        // std::cout<<"j = "<<j<<std::endl;
                    }
                }
            }
            // std::cout<<"nei for"<<std::endl;
        }

        if(confirm_all.size() != 0)
        {
            target temp2;
            int min_pos = confirm_all.back();                //最近的那个匹配
            confirm.push_back(confirm_all.back());      
            // std::cout<<"min_pos:"<<confirm_all.back()<<std::endl;
            temp2.tar_associated = 1;               //预测目标被观测到
            Z_k[min_pos].tar_associated = 1;
            result[i].predict.back().tar_associated = 1;
            temp2.x_pos = Z_k[min_pos].x_pos;
            temp2.y_pos = Z_k[min_pos].y_pos;
            temp2.wide = Z_k[min_pos].wide;
            temp2.deep = Z_k[min_pos].deep;
            temp2.height = Z_k[min_pos].height;
            temp2.vx = result[i].predict.back().vx;
            temp2.vy = result[i].predict.back().vy;
            temp2.v = result[i].predict.back().v;
            for (uint m = 0;m < 8; m++)
            {
                 temp2.points[m][0] = Z_k[min_pos].points[m][0];
                 temp2.points[m][1] = Z_k[min_pos].points[m][1];
                 temp2.points[m][2] = Z_k[min_pos].points[m][2];
            }
            result[i].history.push_back(temp2);       //关联上的加入history中,即更新result中的值
            // std::cout<<" 匹配id:"<<result[i].id<<" x_pos:"<<result[i].history.back().x_pos<<" y_pos:"<<result[i].history.back().y_pos<<std::endl;
            continue;            
         }

    }
    
    //没有关联上的预测值
    for(uint i = 0;i < result.size() && first_flag == 0;i++)
    {   
        target temp3;
        if(result[i].predict.back().tar_associated == 0)
        {
            temp3.vx = result[i].predict.back().vx;
            temp3.vy = result[i].predict.back().vy;
            temp3.v = result[i].predict.back().v;
            temp3.wide = result[i].predict.back().wide;
            temp3.deep = result[i].predict.back().deep;
            temp3.height = result[i].predict.back().height;
            temp3.x_pos = result[i].predict.back().x_pos;
            temp3.y_pos = result[i].predict.back().y_pos;
            temp3.tar_associated = 0;
            for (uint m = 0;m < 8; m++)
            {
                temp3.points[m][0] = result[i].predict.back().points[m][0];
                temp3.points[m][1] = result[i].predict.back().points[m][1];
                temp3.points[m][2] = result[i].predict.back().points[m][2];
            }
            result[i].history.push_back(temp3);
            // std::cout<<"id:"<<result[i].id<<std::endl;
            // std::cout<<"未关联上id:"<<result[i].id<<"  result[i].predict.back().x_pos: "<<result[i].predict.back().x_pos<<"  result[i].predict.back().y_pos: "<<result[i].predict.back().y_pos<<std::endl;
        }
    }

    //保存新出现的观测值
        for(uint j = 0;j < Z_k.size() && first_flag == 0;j++)       
        {
            target temp4;
            target_pre_his new_initial;
            if(Z_k[j].tar_associated == 0)
            {
                temp4.vx = 0;//观测结果
                temp4.vy = 0;
                temp4.v = 0;
                temp4.tar_associated = 1;            //这里一定要给tar_associated设定初值,不然后面会跳过连续帧的检测过程
                temp4.wide = Z_k[j].wide;
                temp4.deep = Z_k[j].deep;
                temp4.height = Z_k[j].height;
                temp4.x_pos = Z_k[j].x_pos;
                temp4.y_pos = Z_k[j].y_pos;
                for (uint m = 0;m < 8; m++)
                {
                    temp4.points[m][0] = Z_k[j].points[m][0];
                    temp4.points[m][1] = Z_k[j].points[m][1];
                    temp4.points[m][2] = Z_k[j].points[m][2];
                }
                new_initial.predict.push_back(temp4);
                new_initial.history.push_back(temp4);
                result.push_back(new_initial);
                // std::cout<<"新预选目标  x_pos:"<<temp4.x_pos<<"  y_pos:"<<temp4.y_pos<<std::endl;
            }
        }

    
    // std::cout<<"关联后目标数:"<<result.size()<<std::endl;
    for(uint m = 0;m < result.size();m++)
    {
        // std::cout<<std::endl;
        // std::cout<<"目标id:"<<result[m].id<<" result["<<m<<"].history.size:"<<result[m].history.size()<<"   x_pos:"<<result[m].history.back().x_pos<<"   y_pos:"<<result[m].history.back().y_pos << " v:" << result[m].history.back().v <<std::endl;
        // std::cout<<"result[m].history.size():"<<result[m].history.size()<<std::endl;
        // std::cout<<"目标flag_appear:"<<result[m].history.back().flag_appear<<std::endl;
        // // std::cout<<"目标flag_new_target:"<<result[m].history.back().flag_new_target<<std::endl;
        // std::cout<<"目标vx:"<<result[m].history.back().vx<<std::endl;
        // std::cout<<"目标vy:"<<result[m].history.back().vy<<std::endl;
        // std::cout<<"目标wide:"<<result[m].history.back().wide<<std::endl;
        // std::cout<<"目标length:"<<result[m].history.back().length<<std::endl;
        // std::cout <<"目标x:"<<result[m].history.back().x_pos<<std::endl;
        // std::cout <<"目标y:"<<result[m].history.back().y_pos<<std::endl;
        // std::cout<<result[m].history.back().id<<"号目标 result["<<m<<"].apr_wide:"<<result[m].apr_wide<<" result["<<m<<"].apr_length:"<<result[m].apr_length<<std::endl;

        fout2<<frame_count<<"     "
        <<result[m].id<<"号目标     "
        <<result[m].history.size()<<"    "
        <<result[m].history.back().tar_associated<<"    "
        <<result[m].flag_new_target<<"    "
        <<result[m].history.back().wide<<"    "
        <<result[m].history.back().deep<<"    "
        <<result[m].history.back().height<<"    "
        <<result[m].history.back().x_pos<<"    "
        <<result[m].history.back().y_pos<<"    "
        <<result[m].history.back().vy<<"    "
        <<result[m].history.back().v<<"    "
        <<delta_time<<"   "
        <<result[m].predict.back().x_pos << "    "
        <<result[m].predict.back().y_pos << "    "
        <<std::endl;
    }
    // std::cout<<std::endl;
}


/*******************************************************滤波*********************************************************/
/*****最小二乘法*****
二次方程:Z=H*A,得:A=H.inverse()*Z
反推得到Z,利用二次曲线上面的点更新Z值
******************/
void Data_association::filter()
{
    if(time_save.size() >= 10)
    {
        float s_t[10] = {0};
        int m = 0;
        std::list < double>::iterator c = time_save.end();
        //定位到time_save倒数第10个数据
        for(uint i = 0;i < 10;i++)
            c--;
        //依次获取最近10个时间并累加
        for(;c != time_save.end();c++)
        {
           if(m == 0)
                s_t[m] = *c;
           if(m >= 1 && m <= 9)
                s_t[m] = s_t[m-1]+*c;
           m++;
        }

        Eigen::MatrixXd	H_det(3,10);            //观测矩阵
        H_det << s_t[0]*s_t[0],s_t[1]*s_t[1],s_t[2]*s_t[2],s_t[3]*s_t[3],s_t[4]*s_t[4],s_t[5]*s_t[5],s_t[6]*s_t[6],s_t[7]*s_t[7],s_t[8]*s_t[8],s_t[9]*s_t[9],
                s_t[0],s_t[1],s_t[2],s_t[3],s_t[4],s_t[5],s_t[6],s_t[7],s_t[8],s_t[9],
                1,1,1,1,1,1,1,1,1,1;
        Eigen::MatrixXd parameter_x(3,1);     //这里保存a、b、c的值
        Eigen::MatrixXd parameter_y(3,1);     //这里保存a、b、c的值
        Eigen::MatrixXd Z_x(10,1);           
        Eigen::MatrixXd Z_y(10,1);           
        Z_x<<0,0,0,0,0,0,0,0,0,0;
        Z_y<<0,0,0,0,0,0,0,0,0,0;
        parameter_x<<0,0,0;
        parameter_y<<0,0,0;

        for(uint i = 0;i < result.size();i++)
        {
            // std::cout<<"filter id:"<<result[i].id<<" result["<<i<<"].history.size:"<<result[i].history.size()<<std::endl;
            if (result[i].history.size() < 10  &&  result[i].history.size()>0) 
            {
                uint j = 0;
                for (; j < 10 - result[i].history.size(); j++) 
                {
                    Z_x(j) = (*result[i].history.begin()).x_pos;
                    Z_y(j) = (*result[i].history.begin()).y_pos;
                }
                for (uint k = 0; k < 10-j; k++)
                {
                    Z_x(k+j) = (*(result[i].history.begin() + k )).x_pos;
                    Z_y(k+j) = (*(result[i].history.begin() + k )).y_pos;
                }
            }
            else
            {
                for (uint k = 10; k > 0; k--)
                {
                    Z_x(10 - k) = (*(result[i].history.rbegin() + k-1)).x_pos;
                    Z_y(10 - k) = (*(result[i].history.rbegin() + k-1)).y_pos;
                }
            }

            //参数a,b,c
            parameter_x = (H_det*H_det.transpose()).inverse()*H_det*Z_x;
            parameter_y = (H_det*H_det.transpose()).inverse()*H_det*Z_y;
            // std::cout<<"parameter_x:"<<parameter_x;

            //结果更新
            result[i].history.back().x_pos = parameter_x(0)*s_t[9]*s_t[9]+parameter_x(1)*s_t[9]+parameter_x(2);
            result[i].history.back().y_pos = parameter_y(0)*s_t[9]*s_t[9]+parameter_y(1)*s_t[9]+parameter_y(2);
            // std::cout<<"after_filter_x_pos:"<<result[i].history.back().x_pos<<std::endl;
            // std::cout<<std::endl;
        }
    }
}

/*****************KALMAN滤波*********************/
void Data_association::kalman_init(target_pre_his &target_kalman)
{
    //设定初值
    target_kalman.X_evlt << target_kalman.history.back().x_pos, target_kalman.history.back().y_pos, 0, 0;
    target_kalman.x_evlt.push_back(target_kalman.X_evlt);
    target_kalman.x_pdct.push_back(target_kalman.X_pdct);
    target_kalman.z_meas.push_back(target_kalman.Z_meas);
    target_kalman.pk.push_back(target_kalman.Pk);
    target_kalman.pk_p.push_back(target_kalman.Pk_p);
    target_kalman.k.push_back(target_kalman.K);
}
void Data_association::filter_kalman()
{
    for(uint i = 0; i < result.size(); i++)
    {
        if(result[i].history.size() == 1)
        {
            kalman_init(result[i]);     //第一帧初始化
        }
    }
    double acc = 10;       //加速度，尚未用到
    for(uint i = 0; i < result.size(); i++)
    {
        //1.状态估计
        result[i].X_pdct = result[i].A * result[i].x_evlt.back() + result[i].B * acc;
        result[i].x_pdct.push_back(result[i].X_pdct);
        //2.预测状态与真实状态的协方差矩阵，Pk'
        result[i].Pk_p = result[i].A * result[i].pk.back() * result[i].A.transpose() + result[i].Q;
        result[i].pk_p.push_back(result[i].Pk_p);
        //3.滤波增益
        MatrixXd tmp(2, 2);
        tmp = result[i].H * result[i].pk_p.back() * result[i].H.transpose() + result[i].R;
        result[i].K = result[i].pk_p.back() * result[i].H.transpose() * tmp.inverse();
        result[i].k.push_back(result[i].K);
        result[i].Z_meas << result[i].history.back().x_pos, result[i].history.back().y_pos;
        result[i].z_meas.push_back(result[i].Z_meas);
        //4.最优估计，状态更新
        result[i].X_evlt = result[i].x_pdct.back() + result[i].k.back() * (result[i].z_meas.back() - result[i].H * result[i].x_pdct.back());
        result[i].x_evlt.push_back(result[i].X_evlt);
        //5.估计状态和真实状态的协方差矩阵更新
        result[i].Pk = (MatrixXd::Identity(4, 4) - result[i].k.back() * result[i].H) * result[i].pk_p.back();
        result[i].pk.push_back(result[i].Pk);

        result[i].history.back().x_pos = result[i].X_evlt(0,0);
        result[i].history.back().y_pos = result[i].X_evlt(1,0);
        // std::cout<<"kalman_hou:"<<result[i].history.back().x_pos<<std::endl;
    }
    // std::cout<<"kalman"<<std::endl;
}

//时间计算,时间的存储从前往后1-30个时间,后面为最新的时间
double Data_association::save_time(double delta_time,uint frame_nums)
{  
    double time_sum = 0.0;
    uint i = 0;
    if(frame_nums == 0)                   
    {
        time_save.push_back(delta_time);
    }
    
    if(time_save.size() == 31)        //使总的时间帧数保持在30帧
    {
        time_save.pop_front();
    }
    if(frame_nums < time_save.size() && frame_nums >= 3)      //在3-30帧之内的时间均可
    {
        for(auto it = time_save.rbegin();i < frame_nums;it++,i++)      
            {
                time_sum += *it;
                // std::cout<<*it<<"  ";
            }
    }
    return time_sum;
}

void Data_association::get_time(custom_msgs::ObjectArray &msgout)
{
    if(current_time  ==  0)
    {
        current_time = msgout.header.stamp.toSec();
        last_time = msgout.header.stamp.toSec() - 0.1;
    }
    else
    {   
        last_time = current_time;
        current_time = msgout.header.stamp.toSec();
        // std::cout<<setiosflags(std::ios::fixed)<<std::setprecision(9)<<current_time<< std::endl;
    }
    delta_time = current_time - last_time;
    save_time(fabs(delta_time),0);          //时间，传入参数为0时表示不计算时间，仅用作存储时间

}

//传出结果
void Data_association::get_result_association(custom_msgs::ObjectArray &msgout,custom_msgs::LaneLineArray lane_line)
{ 
    vector <target>temp;
    //保持在41帧,更长时间的记录删去
    for(uint i = 0;i < result.size();i++)
    {
        if(result[i].history.size() >= 40)
        {
            for(auto it = result[i].history.rbegin();it != (result[i].history.rbegin()+40);it++)
                temp.push_back(*it);
            result[i].history.clear();
            for(auto it = temp.rbegin();it != (temp.rbegin()+40);it++)
                result[i].history.push_back(*it);
            temp.clear();
            
            for(auto it = result[i].predict.rbegin();it != (result[i].predict.rbegin()+40);it++)
                temp.push_back(*it);
            result[i].predict.clear();
            for(auto it = temp.rbegin();it != (temp.rbegin()+40);it++)
                result[i].predict.push_back(*it);
        }
        // std::cout<<"size of the history:"<<result[i].history.size()<<std::endl;
    }

    //只传出flag_new_target=1的目标
    custom_msgs::ObjectArray outdataarray;
    size_t count_size = 0;
    for(uint i = 0;i < result.size();i++)           
    {
        if(result[i].flag_new_target == 1)
            {
                count_size++;
                // std::cout << "v_count_size = " << result[i].history.back().v << std::endl;
            }
    }
    outdataarray.objs.resize(count_size);
    uint out = 0;
    for(uint j = 0; j < result.size(); j++)
    {
        if(result[j].flag_new_target == 1)
        {
                //id
                outdataarray.objs[out].id = result[j].id;
                //根据大小简单分类
                if(result[j].history.back().wide>1.3 && result[j].history.back().wide < 2.0) outdataarray.objs[out].type = 1;//che
                else if(result[j].history.back().wide>0.3 && result[j].history.back().wide < 0.9) outdataarray.objs[out].type = 2;//ren
                else
                    outdataarray.objs[out].type = -1;//unknow
                //速度
                outdataarray.objs[out].vx = result[j].history.back().vx;
                outdataarray.objs[out].vy = result[j].history.back().vy;
                outdataarray.objs[out].v = result[j].history.back().v;
                // std::cout << "v = " << result[j].history.back().v << std::endl;
                //8个顶点问题
                for (uint m = 0;m < 8; m++)
                {
                    outdataarray.objs[out].bbox_point[m].x = result[j].history.back().points[m][0];
                    outdataarray.objs[out].bbox_point[m].y = result[j].history.back().points[m][1];
                    outdataarray.objs[out].bbox_point[m].z = result[j].history.back().points[m][2];
                }
                //位置
                outdataarray.objs[out].x_pos = result[j].history.back().x_pos;
                outdataarray.objs[out].y_pos = result[j].history.back().y_pos;
                outdataarray.objs[out].lwh.x = result[j].history.back().deep;
                outdataarray.objs[out].lwh.y = result[j].history.back().wide;
                outdataarray.objs[out].lwh.z = result[j].history.back().height;
                // //计算s、d
                // std::vector < double> object_s_d = getFrenet(result[j].history.back().x_pos,result[j].history.back().y_pos,lane_line.lines[0].x,lane_line.lines[0].y); 
                // outdataarray.objs[out].s_pos = object_s_d[0];
                // outdataarray.objs[out].d_pos = object_s_d[1];
                out++;
        }
    }

    // std::cout <<std::endl;
    msgout = outdataarray;
    first_flag = 0;
    count_size = 0;
}
