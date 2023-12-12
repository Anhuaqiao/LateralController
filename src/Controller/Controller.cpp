#include "Controller.h"
#include <cmath>
namespace aiforce {
namespace control{


Controller::Controller():
      PreTurn_(true),
      refpath_limitnum(5),
      max_angle_(30),
      min_angle_(-30),
      gap_(0.2){}

double Controller::steer_limit(double steer_angle){
    steer_angle = steer_angle > PI/6 ? PI/6:steer_angle;
    steer_angle = steer_angle < (-PI/6) ? (-PI/6):steer_angle;
    return steer_angle;
}

void Controller::Controller_preprocessor(vector<decision::SinglePoint> msg,
                         int Pts_Num,
                         double currentspeed,
                         float yaw_rate,
                         float WheelBase,
                         int driving_mode_, //0：手动驾驶， 1:自动驾驶，2:导航模式
                         int rnd_gear_){
    PreTurn_=true; // 路径是否连续控制处理
    Driving_mode_=driving_mode_;
    RND_gear_=rnd_gear_;
    avgfilt.setting_num(currentspeed);
    bool path_flag=UpdatePath(msg);

    State cur_state(0,0,0,0);   //(x,y,posture) x沿车身方向 y垂直车身方向 xy满足右手定则
    cur_state.x=WheelBase;
                         }

double Controller::calRefPsiRate(std::vector<State>& refer_path, int& target_index, double cur_speed){
    return target_index==(refer_path.size()-1)? (refer_path[target_index].psi-refer_path[target_index-1].psi)/Pt_dist(refer_path[target_index],refer_path[target_index-1])/cur_speed:(refer_path[target_index+1].psi-refer_path[target_index].psi)/Pt_dist(refer_path[target_index+1],refer_path[target_index])/cur_speed;
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
bool Controller::UpdatePath(vector<decision::SinglePoint> msg)
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
    int target_num=5;
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
                pt_state.psi=atan2((path_temp[i+1].y-path_temp[i].y),(path_temp[i+1].x-path_temp[i].x));
            else
                pt_state.psi=atan2((path_temp[i].y-path_temp[i].y),(path_temp[i].x-path_temp[i-1].x));
            RefState_.push_back(pt_state);
        }
        dist=Pt_dist(path_temp[path_temp.size()-1],path_temp[path_temp.size()-2]);
        dx=(path_temp[path_temp.size()-1].x-path_temp[path_temp.size()-2].x)/dist*gap_;
        dy=(path_temp[path_temp.size()-1].y-path_temp[path_temp.size()-2].y)/dist*gap_;
        for(unsigned int i=1;i<(refpath_limitnum-path_temp.size());i++)
        {
            pt_state.x=path_temp[path_temp.size()-1].x+dx*i;
            pt_state.y=path_temp[path_temp.size()-1].y+dy*i;
            pt_state.psi=atan2(dy,dx);
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
                pt_state.psi=atan2((path_temp[i+1].y-path_temp[i].y),(path_temp[i+1].x-path_temp[i].x));
            else
                pt_state.psi=atan2((path_temp[i].y-path_temp[i].y),(path_temp[i].x-path_temp[i-1].x));
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

        //curvature: r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
            RefState_[i].K= (ddx * dy - ddy * dx) / pow((dy * dy + dx * dx), 3 / 2) ;// 曲率k计算
    }
    return true;
}


}
}



