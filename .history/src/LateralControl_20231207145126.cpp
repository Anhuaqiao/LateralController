#include "LateralControl.h"
#include <cmath>
namespace aiforce {
namespace control{


Lateral_Control::Lateral_Control()
    :
      factor_str_kp(1.0),
      factor_str_ki(1.0),
      factor_str_kd(1.0),
      factor_ridge_kp(1.0),
      factor_ridge_ki(1.0),
      factor_ridge_kd(1.0),
      PreTurn_(true),
      refpath_limitnum(30),
      Last_TargetAngle_(0),
      max_angle_(30),
      min_angle_(-30),
      gap_(0.2),
      fst_in_autodriving(true),
      fst_in_lateral_control_(true){}

/**
 * @brief 函数简要说明-测试函数
 * @param index    参数1
 * @param t        参数2 @see CTest
 *
 * @return 返回说明
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
float Lateral_Control::GetTargetSteer(vector<decision::SinglePoint> msg,
                                      int Pts_Num,
                                      float CurrentSpeed,
                                      float Yaw_rate,
                                      float WheelBase,
                                      int driving_mode_,
                                      int rnd_gear_,
                                      int wk_type,
                                      int machine_type)
{
    PreTurn_=true; // 路径是否连续控制处理
    Driving_mode_=driving_mode_;
    RND_gear_=rnd_gear_;
    if(fst_in_autodriving)
    {
        fst_in_autodriving=false;
    }
    int ave_num=5; //均值滤波的阈值
    if(CurrentSpeed<1)
    {
        ave_num=5;
    }
    else if(CurrentSpeed>=1&&CurrentSpeed<=2)
    {
        ave_num=10;
    }
    else if(CurrentSpeed>=2&&CurrentSpeed<=3)
    {
        ave_num=15;
    }
    else
    {
        ave_num=20;
    }
    bool path_flag=UpdatePath(msg);

    if(!path_flag)
    {
        return Last_TargetAngle_;
    }

    State cur_state(0,0,PI/2);   //(x,y,posture) x沿车身方向 y垂直车身方向 xy满足右手定则

    if(Driving_mode_==2&&RND_gear_==0)
    {
        cur_state.x=-wheel_base_dist_;
    }
    else
    {
        cur_state.x=wheel_base_dist_;
    }


    double calculate_angle=0;
    /*********** Stanley ***********/
    double k_stanley=2;
    if(fst_in_autodriving)
    {
    }    //                    k_psi,        k_dist,    k_soft,       k_yaw_rate,     kd
    stanley.set_parameter(factor_str_kp,factor_str_ki,factor_str_kd,factor_ridge_kp,factor_ridge_ki);
    stanley.cur_speed=CurrentSpeed;
    stanley.yaw_rate=Yaw_rate;
    calculate_angle=stanley.stanleyControl(cur_state,CurrentSpeed,wheel_base_dist_,RefState_);
    /*######### Stanley #########*/

    /*********** PurePursuit ***********/
//    PurePursuit pp;
//    calculate_angle=pp.purePursuitControl(cur_state,CurrentSpeed,RefState_,WheelBase);
//    Stanley stanley(k_stanley);
    /*######### PurePursuit #########*/


    /*********** RearWheelFeedback ***********/
//    double k_stanley=5;
//    Stanley stanley(k_stanley);
    /*######### RearWheelFeedback #########*/

    /**************** 均值滤波 *************/
    float sum_angle=0;
    if (ave_output_list_.size() >= ave_num) {
        while(ave_output_list_.size() >= ave_num)
            ave_output_list_.pop();
        ave_output_list_.push(calculate_angle);
    } else {
        ave_output_list_.push(calculate_angle);
    }
    if(ave_output_list_.size()>0)
    {
        std::queue<float> temp_list(ave_output_list_);
        while (!temp_list.empty()) {
            sum_angle += temp_list.front();
            temp_list.pop();
        }
        calculate_angle=sum_angle/ave_output_list_.size();
    }
    /*######### 均值滤波 #########*/

    calculate_angle=normalizeAngle(calculate_angle);
    if(calculate_angle*180/PI>max_angle_)
        Target_angle_=max_angle_;
    else if(calculate_angle*180/PI<min_angle_)
        Target_angle_=min_angle_;
    else
        Target_angle_=calculate_angle*180/PI;
    return Target_angle_;
}

/**
 * @brief 函数简要说明-更新参考路径
 * @param index    参数1
 * @param t        参数2 @see CTest
 *
 * @return false 有效路径点小于两个; Refpath_为新的参考路径
 *     -<em>false</em> fail
 *     -<em>true</em> succeed
 */
bool Lateral_Control::UpdatePath(vector<decision::SinglePoint> msg)
{
    vector<decision::SinglePoint> path_temp;
    RefState_.clear();
    double dx,dy,dist;
    bool Backgear_flag=false;
    if(msg.size()<2)
        return false;
    decision::SinglePoint last_pt=msg[0];
    decision::SinglePoint pt_temp;
    State pt_state;
    double path_dx=0;
    double path_dy=0;
    int cur_mode1=msg[0].mode1;
    int cur_mode2=msg[0].mode2;
    int count_num=0;
    int target_num=100;
    if(msg.size()>1)
    {
        path_dx=(msg[1].x-msg[0].x)/Pt_dist(msg[1],msg[0]);
        path_dy=(msg[1].y-msg[0].y)/Pt_dist(msg[1],msg[0]);
    }



    if(Driving_mode_==0)
        return false;
    else if(Driving_mode_==2)  //导航模式
    {
        for(unsigned int i=0;i<msg.size();i++)
        {
            if(i>0) //相邻点横纵坐标一样,排除
                if(IsEqual(last_pt.x,msg[i].x)&&IsEqual(last_pt.y,msg[i].y))
                    continue;
            last_pt=msg[i];
            if(RND_gear_==0)
            {
//                msg[i].x*=-1;
//                msg[i].y*=-1;
//                path_temp.insert(path_temp.begin(),msg[i]);

                pt_temp.x=msg[0].x-path_dx*i*0.2;
                pt_temp.y=msg[0].y-path_dy*i*0.2;
                path_temp.insert(path_temp.begin(),pt_temp);
            }
            else
                path_temp.push_back(msg[i]);
        }
    }
    else   //
    {
        int n=0;
        for(n=0;n<msg.size();n++)
        {
            if(n>0) //相邻点横纵坐标一样,排除
                if(IsEqual(last_pt.x,msg[n].x)&&IsEqual(last_pt.y,msg[n].y))
                    continue;
            last_pt=msg[n];

            if(PreTurn_) //耙地作业
            {
                path_temp.push_back(msg[n]);
                count_num++;
            }
            else
            {
                if(cur_mode1==1||cur_mode1==3) //工作区
                {
                    if(msg[n].mode1==1||msg[n].mode1==3)
                    {
                        path_temp.push_back(msg[n]);
                        count_num++;
                    }
                }
                else if(cur_mode2==0||cur_mode2==3) //停车点
                    return false;
                else //非工作区
                {
                    if(msg[n].mode2==0)
                        break;
                    else
                    {
                        path_temp.push_back(msg[n]);
                        count_num++;
                    }
                }
            }
        }
        if(count_num<target_num)
        {
            if(count_num>1)
            {
                path_dx=(msg[n].x-msg[n-1].x)/Pt_dist(msg[n],msg[n-1]);
                path_dy=(msg[n].y-msg[n-1].y)/Pt_dist(msg[n],msg[n-1]);
            }
            else
            {
                path_dx=(msg[n+1].x-msg[n].x)/Pt_dist(msg[n+1],msg[n]);
                path_dy=(msg[n+1].y-msg[n].y)/Pt_dist(msg[n+1],msg[n]);
            }
            for(int j=count_num+1;j<=target_num;j++)
            {
                pt_temp.x=msg[n].x+path_dx*(j-count_num)*0.2;
                pt_temp.y=msg[n].y+path_dy*(j-count_num)*0.2;
                path_temp.push_back(pt_temp);
            }
        }
    }

    if(path_temp.size()<4) //小于4个点 无法求曲率 用上次的转角
        return false;
    else if(path_temp.size()<refpath_limitnum)
    {
        for(unsigned int i=0;i<path_temp.size();i++)
        {
            pt_state.x=path_temp[i].x;
            pt_state.y=path_temp[i].y;
            if(i<path_temp.size()-1)
                pt_state.psi=atan2((path_temp[i+1].x-path_temp[i].x),(path_temp[i+1].y-path_temp[i].y));
            else
                pt_state.psi=atan2((path_temp[i].x-path_temp[i-1].x),(path_temp[i].y-path_temp[i].y));
            RefState_.push_back(pt_state);
        }
        dist=Pt_dist(path_temp[path_temp.size()-1],path_temp[path_temp.size()-2]);
        dx=(path_temp[path_temp.size()-1].x-path_temp[path_temp.size()-2].x)/dist*gap_;
        dy=(path_temp[path_temp.size()-1].y-path_temp[path_temp.size()-2].y)/dist*gap_;
        for(unsigned int i=1;i<(refpath_limitnum-path_temp.size());i++)
        {
            pt_state.x=path_temp[path_temp.size()-1].x+dx*i;
            pt_state.y=path_temp[path_temp.size()-1].y+dy*i;
            pt_state.psi=atan2(dx,dy);
            RefState_.push_back(pt_state);
        }
    }
    else
    {
        for(unsigned int i=0;i<path_temp.size();i++)
        {
            pt_state.x=path_temp[i].x;
            pt_state.y=path_temp[i].y;
            if(i<path_temp.size()-1)
                pt_state.psi=atan2((path_temp[i+1].x-path_temp[i].x),(path_temp[i+1].y-path_temp[i].y));
            else
                pt_state.psi=atan2((path_temp[i].x-path_temp[i-1].x),(path_temp[i].y-path_temp[i].y));
            RefState_.push_back(pt_state);
        }
    }

    if(Backgear_flag)
        for(unsigned int i=0;i<RefState_.size();i++)
            RefState_[i].psi=(RefState_[i].psi-PI)>0?(RefState_[i].psi-PI):(RefState_[i].psi+PI);

    // 求目标路径曲率
    double ddx,ddy;
    for(int i=0;i<RefState_.size();i++){
        if (i==0){
             dx = RefState_[i+1].x - RefState_[i].x;
             dy = RefState_[i+1].y - RefState_[i].y;
             ddx = RefState_[2].x + RefState_[0].x - 2*RefState_[1].x;
             ddy = RefState_[2].y + RefState_[0].y - 2*RefState_[1].y;
        }else if(i==RefState_.size()-1){
             dx = RefState_[i].x - RefState_[i-1].x;
             dy = RefState_[i].y - RefState_[i-1].y;
             ddx = RefState_[i].x + RefState_[i-2].x - 2*RefState_[i-1].x;
             ddy = RefState_[i].y + RefState_[i-2].y - 2*RefState_[i-1].y;
        }else{
             dx = RefState_[i+1].x - RefState_[i].x;
             dy = RefState_[i+1].y - RefState_[i].y;
             ddx = RefState_[i+1].x + RefState_[i-1].x - 2*RefState_[i].x;
             ddy = RefState_[i+1].y + RefState_[i-1].y - 2*RefState_[i].y;
        }
//        cout << "dx " << dx<<"  dy "<<dy<< std::endl;
//        cout << "atan " << atan2(dy,dx)<< std::endl;

        //refer_path[i][2] = atan2(dy,dx);//yaw
        //计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        //参考：https://blog.csdn.net/weixin_46627433/article/details/123403726
        //refer_path[i][3]= (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3 / 2) ;// 曲率k计算
            RefState_[i].K= (ddx * dy - ddy * dx) / pow((dy * dy + dx * dx), 3 / 2) ;// 曲率k计算
    }
    return true;
}


}
}



