#include "LQR.h"



LQR::LQR(double x, double y, double psi, double v, double L, double dt, MatrixXd Q, MatrixXd R):aiforce::control::Controller(){
    model.Initialization(x, y, psi, v, L, dt);
    this->Q = Q;
    this->R = R;
}

MatrixXd LQR::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R){
    MatrixXd Qf= Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for(int i=0;i<N;i++){
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        //小于预设精度时返回
        if((P_-P).maxCoeff()<eps&&(P-P_).maxCoeff()<eps)break;
        //if((P_-P).cwiseAbs().maxCoeff()<EPS)break;

        P = P_;

    }
    return P_;
}


double LQR::steer(State cur_state,double cur_speed,double L, vector<State> refer_path){
    int target_index = calTargetIndex(cur_state, refer_path);
    double symbol = normalizeAngle(refer_path[target_index].psi-atan2(refer_path[target_index].y-cur_state.y, refer_path[target_index].x-cur_state.x))>0? 1:-1;
    double cross_track_error = calTargetIndexDistance(cur_state, refer_path);
    cross_track_error *=symbol;
    crs_track_err.push_back(cross_track_error);
    int target_index_curvature = refer_path[target_index].K;
    double ref_delta = atan2(L*target_index_curvature, 1);
    double ref_psi = refer_path[target_index].psi;
    vector<MatrixXd>state_space = model.stateSpace(ref_delta,ref_psi);
    MatrixXd A = state_space[0];
    MatrixXd B = state_space[1];
    MatrixXd X(3,1);
    X<<cur_state.x-refer_path[target_index].x,cur_state.y-refer_path[target_index].y,cur_state.psi-refer_path[target_index].psi;
    MatrixXd P = calRicatti(A,B,Q,R);
    MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
    MatrixXd u = K*X; //[v-ref_v,delta-ref_delta]
    double delta = u(1,0)-ref_delta;
    return delta;
}