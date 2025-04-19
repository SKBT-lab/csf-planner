#include "Minimumsnap.hpp"

//define factorial function, input i, output i!
int TRAJECTORY_GENERATOR::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

std::vector<Eigen::VectorXd> TRAJECTORY_GENERATOR::PolyQPGeneration(const std::vector<Eigen::Vector3d> &Path_, // waypoints coordinates (3d)
                                                                    const std::vector<Eigen::Vector3d> &Vel_,  // boundary velocity
                                                                    const std::vector<Eigen::Vector3d> &Acc_,  // boundary acceleration
                                                                    const std::vector<double>& Time_, // time allocation in each segment
                                                                    const Soft_waypoints& Softwps_)
{
    int n = Path_.size();
    Eigen::MatrixXd Path(n, 3);
    for (int i = 0; i < n; ++i) {
        Path.row(i) = Path_[i];  
    }
    
    n = Vel_.size();
    Eigen::MatrixXd Vel(n, 3);
    for (int i = 0; i < n; ++i) {
        Vel.row(i) = Vel_[i];  
    }

    n = Acc_.size();
    Eigen::MatrixXd Acc(n, 3);
    for (int i = 0; i < n; ++i) {
        Acc.row(i) = Acc_[i];  
    }
    
    Eigen::VectorXd Time = Eigen::VectorXd::Map(Time_.data(), Time_.size());

    int seg_num = Time.size();          // the number of segments

    MatrixXd PolyCoeff = MatrixXd::Zero(seg_num, 3 * _poly_num1D);     // position(x,y,z), so we need (3 * _poly_num1D) coefficients

    VectorXd Px(_poly_num1D * seg_num);     // coefficients in each axis
    VectorXd Py(_poly_num1D * seg_num);
    VectorXd Pz(_poly_num1D * seg_num);

    // enforce initial and final position,velocity and accleration, for higher order derivatives, just assume them be 0
    MatrixXd StartState(_dev_order, 3);
    MatrixXd EndState(_dev_order, 3);
    StartState.row(0) = Path.row(0);
    StartState.row(1) = Vel.row(0);
    StartState.row(2) = Acc.row(0);
    EndState.row(0) = Path.row((Path.rows()-1));
    EndState.row(1) = Vel.row(1);
    EndState.row(2) = Acc.row(1);
    if(_dev_order == 4)
    {
        StartState.row(3) = VectorXd::Zero(3);  // jerk
        EndState.row(3) = VectorXd::Zero(3); 
    }
    // cout << " StartState = " << endl;
    // cout << StartState << endl;
    // cout << " EndState = " << endl;
    // cout << EndState << endl;


    // MatrixXd DiffMat = MatrixXd::Zero(_poly_num1D, _poly_num1D);
    // for(int i = 0; i < DiffMat.rows()-1; i++)
    //     DiffMat(i,i+1) = i+1;
    // cout << " DiffMat = " << endl;
    // cout << DiffMat << endl;

    // MatrixXd A = MatrixXd::Identity(_poly_num1D, _poly_num1D);
    // for(int i = 0; i < _dev_order; i++)   
    // {
    //     A *= DiffMat;                // coefficients of n-order derivative
    //     cout << " A = " << endl;
    //     cout << A << endl;
    // } 
 

    _Q = MatrixXd::Zero(_poly_num1D * seg_num, _poly_num1D * seg_num);
    _M = MatrixXd::Zero(_poly_num1D * seg_num, _poly_num1D * seg_num);
    _Ct = MatrixXd::Zero(2 * _dev_order * seg_num, _dev_order * (seg_num + 1));

    for(int seg_index = 0; seg_index < seg_num; seg_index++)
    {
        // calculate Matrix Q
        _Q.block(seg_index*_poly_num1D, seg_index*_poly_num1D, _poly_num1D, _poly_num1D) = getQ(Time, seg_index);
        // calculate Matrix M
        _M.block(seg_index*_poly_num1D, seg_index*_poly_num1D, _poly_num1D, _poly_num1D) = getM(Time, seg_index);
    }
    // calculate Matrix Ct
    
    _Ct = getCt(seg_num);
    
    // cout << " Q = " << endl;
    // cout << _Q << endl;
    // cout << " M = " << endl;
    // cout << _M << endl;
    // cout << " Ct = " << endl;
    // cout << _Ct << endl;
    
    if(!Softwps_.enable){
        Px = closedFormCalCoeff1D(Path.col(0), StartState.col(0), EndState.col(0), seg_num);
        Py = closedFormCalCoeff1D(Path.col(1), StartState.col(1), EndState.col(1), seg_num);
        Pz = closedFormCalCoeff1D(Path.col(2), StartState.col(2), EndState.col(2), seg_num);
    }
    else{
        get_G_S(Softwps_, Time_);
        Px = closedFormCalCoeff1D(Path.col(0), StartState.col(0), EndState.col(0), _S.col(0), seg_num);
        Py = closedFormCalCoeff1D(Path.col(1), StartState.col(1), EndState.col(1), _S.col(1), seg_num);
        Pz = closedFormCalCoeff1D(Path.col(2), StartState.col(2), EndState.col(2), _S.col(2), seg_num);
    }
    
    // cout << " Px = " << endl;
    // cout << Px << endl;
    // cout << " Py = " << endl;
    // cout << Py << endl;
    // cout << " Pz = " << endl;
    // cout << Pz << endl;

    for(int i = 0; i < seg_num; i++)
    {
        PolyCoeff.row(i).segment(0, _poly_num1D) = Px.segment(_poly_num1D*i, _poly_num1D);
        PolyCoeff.row(i).segment(_poly_num1D, _poly_num1D) = Py.segment(_poly_num1D*i, _poly_num1D);
        PolyCoeff.row(i).segment(2*_poly_num1D, _poly_num1D) = Pz.segment(_poly_num1D*i, _poly_num1D);
    }
    // std::cout << " PolyCoeff = " << std::endl;
    // std::cout << PolyCoeff << std::endl;

    // std::cout << " Time = " << std::endl;
    // std::cout << Time << std::endl;

    int rows = PolyCoeff.rows();

    std::vector<Eigen::VectorXd> PolyCoeff_;
    for (int i = 0; i < rows; ++i) {
        // 使用 head() 函数动态截取该行的前 cols 个元素
        Eigen::VectorXd rowVec = PolyCoeff.row(i);  // 动态获取每行的 VectorXd
        PolyCoeff_.push_back(rowVec);
    }

    return PolyCoeff_;
}

double TRAJECTORY_GENERATOR::GetLoss_withT(const Eigen::MatrixXd &StartState, // waypoints coordinates (3d)
                                        const Eigen::MatrixXd &EndState,  // boundary velocity
                                        const Eigen::MatrixXd &Path,
                                        const Eigen::VectorXd& Time // time allocation in each segment
                                        )
{

    int seg_num = Time.size();          // the number of segments

    MatrixXd PolyCoeff = MatrixXd::Zero(seg_num, 3 * _poly_num1D);     // position(x,y,z), so we need (3 * _poly_num1D) coefficients

    double Lx, Ly, Lz;     // coefficients in each axis

    _Q = MatrixXd::Zero(_poly_num1D * seg_num, _poly_num1D * seg_num);
    _M = MatrixXd::Zero(_poly_num1D * seg_num, _poly_num1D * seg_num);
    _Ct = MatrixXd::Zero(2 * _dev_order * seg_num, _dev_order * (seg_num + 1));

    for(int seg_index = 0; seg_index < seg_num; seg_index++)
    {
        // calculate Matrix Q
        
        _Q.block(seg_index*_poly_num1D, seg_index*_poly_num1D, _poly_num1D, _poly_num1D) = getQ(Time, seg_index);
        // calculate Matrix M
        _M.block(seg_index*_poly_num1D, seg_index*_poly_num1D, _poly_num1D, _poly_num1D) = getM(Time, seg_index);
    }
    // calculate Matrix Ct
    
    _Ct = getCt(seg_num);
    
    Lx = closedFormCalLoss(Path.col(0), StartState.col(0), EndState.col(0), seg_num);
    Ly = closedFormCalLoss(Path.col(1), StartState.col(1), EndState.col(1), seg_num);
    Lz = closedFormCalLoss(Path.col(2), StartState.col(2), EndState.col(2), seg_num);

    double total_Loss = Lx + Ly + Lz + Ktime * Time.sum();
}

std::vector<double> TRAJECTORY_GENERATOR::timeRefine(const std::vector<Eigen::Vector3d> &Path_,
                                                    const std::vector<Eigen::Vector3d> &Vel_,
                                                    const std::vector<Eigen::Vector3d> &Acc_,
                                                    const std::vector<double>& Init_time)
{
    int n = Path_.size();
    Eigen::MatrixXd Path(n, 3);
    for (int i = 0; i < n; ++i) {
        Path.row(i) = Path_[i];  
    }
    
    n = Vel_.size();
    Eigen::MatrixXd Vel(n, 3);
    for (int i = 0; i < n; ++i) {
        Vel.row(i) = Vel_[i];  
    }

    n = Acc_.size();
    Eigen::MatrixXd Acc(n, 3);
    for (int i = 0; i < n; ++i) {
        Acc.row(i) = Acc_[i];  
    }
    
    Eigen::VectorXd Time = Eigen::VectorXd::Map(Init_time.data(), Init_time.size());

    int seg_num = Time.size();

    MatrixXd StartState(_dev_order, 3);
    MatrixXd EndState(_dev_order, 3);
    StartState.row(0) = Path.row(0);
    StartState.row(1) = Vel.row(0);
    StartState.row(2) = Acc.row(0);
    EndState.row(0) = Path.row((Path.rows()-1));
    EndState.row(1) = Vel.row(1);
    EndState.row(2) = Acc.row(1);
    if(_dev_order == 4)
    {
        StartState.row(3) = VectorXd::Zero(3);  // jerk
        EndState.row(3) = VectorXd::Zero(3); 
    }
    double initLoss = GetLoss_withT(StartState, EndState, Path, Time);
    Eigen::VectorXd Loss_list = Eigen::VectorXd::Constant(seg_num, initLoss);
    Eigen::VectorXd dt_list = Eigen::VectorXd::Zero(seg_num);

    int k = 0;
    while(k < max_iters){
        for(int n = 0; n < Time.size(); n++){
            Eigen::VectorXd time_temp = Time;
            time_temp(n) += h;
            double loss = GetLoss_withT(StartState, EndState, Path, time_temp);
            dt_list(n) = - (loss - Loss_list(n)) / h * dt;
            Loss_list(n) = loss;
        }
        std::cout << k << "dt: " << dt_list << std::endl;
        if(dt_list.sum() < 0){
            std::cout << k << "th Optimal Time:" << Time << std::endl;
            break;
        }
        else{
            Time = Time + dt_list;
        }
        k ++;
    }

    return std::vector<double>(Time.data(), Time.data() + Time.size());
}
                                    
Eigen::MatrixXd TRAJECTORY_GENERATOR::getQ(const Eigen::VectorXd &Time, const int seg_index)
{
    // calculate Matrix Q_k of the seg_index-th segment
    MatrixXd Q_k = MatrixXd::Zero(_poly_num1D, _poly_num1D);
    for (int i = 0; i < _poly_num1D; i++)
    {
        for (int j = 0; j < _poly_num1D; j++)
        {
            if (i >= _poly_num1D - _dev_order && j >= _poly_num1D - _dev_order)
            {
                Q_k(i, j) = (Factorial(i) / Factorial(i - _dev_order)) * ((Factorial(j) / Factorial(j - _dev_order))) /
                            (i + j - 2 * _dev_order + 1) * pow(Time(seg_index), (i + j - 2 * _dev_order + 1)); // Q of one segment
            }
        }
    }
    // cout << " Q_k = " << endl;
    // cout << Q_k << endl;

    return Q_k;
}


Eigen::MatrixXd TRAJECTORY_GENERATOR::getM(const Eigen::VectorXd &Time, const int seg_index)
{
    MatrixXd M_k = MatrixXd::Zero(_poly_num1D, _poly_num1D);
    VectorXd t_pow = VectorXd::Zero(_poly_num1D);
    for(int i = 0; i < _poly_num1D; i++)
    {
        t_pow(i) = pow(Time(seg_index),i);
    }
    // cout << "t_pow = " << endl;
    // cout << t_pow << endl;

    if(_poly_num1D == 6)        // minimum jerk
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3);
    }
    else if(_poly_num1D == 8)   // minimum snap
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),    t_pow(6),    t_pow(7),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),  6*t_pow(5),  7*t_pow(6),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3), 30*t_pow(4), 42*t_pow(5),
               0,     0   ,     0     ,     6     , 24*t_pow(1), 60*t_pow(2),120*t_pow(3),210*t_pow(4);
    }
    // cout << "M_k = " << endl;
    // cout << M_k << endl;

    return M_k;
}

Eigen::MatrixXd TRAJECTORY_GENERATOR::getCt(const int seg_num)
{
    int d_num = 2 * _dev_order * seg_num;
    int df_and_dp_num = _dev_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * _dev_order + mid_waypts_num;
    // int dp_num = (_dev_order - 1) * mid_waypts_num;

    Eigen::MatrixXd Ct = MatrixXd::Zero(d_num, df_and_dp_num);
    
    // Ct for the first segment: pos,vel,acc,(jerk)
    
    Ct.block(0, 0, _dev_order, _dev_order) = MatrixXd::Identity(_dev_order, _dev_order);
    // Ct for the last segment: pos,vel,acc,(jerk)
    
    Ct.block(d_num - _dev_order, df_num - _dev_order, _dev_order, _dev_order) = MatrixXd::Identity(_dev_order, _dev_order);
    for(int mid_waypts_index = 0; mid_waypts_index < mid_waypts_num; mid_waypts_index++)
    {
        // Ct for middle waypoints: pos
        Ct(_dev_order + 2 * _dev_order * mid_waypts_index, _dev_order + mid_waypts_index) = 1;
        Ct(_dev_order+(_dev_order+2*_dev_order*mid_waypts_index), _dev_order+mid_waypts_index) = 1;

        // Ct for middle waypoints: vel
        Ct(_dev_order+1+2*_dev_order*mid_waypts_index, df_num+(_dev_order-1)*mid_waypts_index) = 1;
        Ct(_dev_order+(_dev_order+1+2*_dev_order*mid_waypts_index), df_num+(_dev_order-1)*mid_waypts_index) = 1;

        // Ct for middle waypoints: acc
        Ct(_dev_order+2+2*_dev_order*mid_waypts_index, (df_num+1)+(_dev_order-1)*mid_waypts_index) = 1;
        Ct(_dev_order+(_dev_order+2+2*_dev_order*mid_waypts_index), (df_num+1)+(_dev_order-1)*mid_waypts_index) = 1;

        if(_dev_order == 4)  // minimum snap
        {
            // Ct for middle waypoints: jerk
            Ct(_dev_order+3+2*_dev_order*mid_waypts_index, (df_num+2)+(_dev_order-1)*mid_waypts_index) = 1;
            Ct(_dev_order+(_dev_order+3+2*_dev_order*mid_waypts_index), (df_num+2)+(_dev_order-1)*mid_waypts_index) = 1;   
        }
    }
    // cout << "Ct = " << endl;
    // cout << Ct << endl;

    return Ct;
}
double TRAJECTORY_GENERATOR::closedFormCalLoss(const Eigen::VectorXd &WayPoints1D,
                                                            const Eigen::VectorXd &StartState1D,
                                                            const Eigen::VectorXd &EndState1D,
                                                            const int seg_num)
{
    int df_and_dp_num = _dev_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * _dev_order + mid_waypts_num;
    int dp_num = (_dev_order - 1) * mid_waypts_num;

    Eigen::MatrixXd C = _Ct.transpose();
    Eigen::MatrixXd M_inv = _M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::MatrixXd R = C * M_inv_tran * _Q * M_inv * _Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

    // compute dF
    Eigen::VectorXd dF(df_num);
    dF.head(_dev_order) = StartState1D;    // start state: pos,vel,acc,(jerk)
    dF.segment(_dev_order, mid_waypts_num) = WayPoints1D.segment(1, WayPoints1D.rows() - 2);  // middle waypoints: pos
    dF.tail(_dev_order) = EndState1D;      // end state: pos,vel,acc,(jerk)
    // cout << "dF = " << endl;
    // cout << dF << endl;
    
    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;   // closed-form solution of Unconstrained quadratic programming

    Eigen::VectorXd dF_and_dP(df_and_dp_num);
    dF_and_dP << dF, dP;
    
    double J = dF_and_dP.transpose() * R * dF_and_dP;
    
    return J;
}
Eigen::VectorXd TRAJECTORY_GENERATOR::closedFormCalCoeff1D(const Eigen::VectorXd &WayPoints1D,
                                                            const Eigen::VectorXd &StartState1D,
                                                            const Eigen::VectorXd &EndState1D,
                                                            const int seg_num)
{
    int df_and_dp_num = _dev_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * _dev_order + mid_waypts_num;
    int dp_num = (_dev_order - 1) * mid_waypts_num;

    auto time_start = std::chrono::high_resolution_clock::now();

    Eigen::MatrixXd C = _Ct.transpose();
    Eigen::MatrixXd M_inv = _M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::VectorXd PolyCoeff1D;
    
    Eigen::MatrixXd R = C * M_inv_tran * _Q * M_inv * _Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

    // compute dF
    Eigen::VectorXd dF(df_num);
    dF.head(_dev_order) = StartState1D;    // start state: pos,vel,acc,(jerk)
    dF.segment(_dev_order, mid_waypts_num) = WayPoints1D.segment(1, WayPoints1D.rows() - 2);  // middle waypoints: pos
    dF.tail(_dev_order) = EndState1D;      // end state: pos,vel,acc,(jerk)
    // cout << "dF = " << endl;
    // cout << dF << endl;
    
    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;   // closed-form solution of Unconstrained quadratic programming

    Eigen::VectorXd dF_and_dP(df_and_dp_num);
    dF_and_dP << dF, dP;
    PolyCoeff1D = M_inv * _Ct * dF_and_dP;   // all coefficients of one Dof
    
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_duration = time_end - time_start; // 秒为单位
    std::cout << "Trajectory Generation Time is " << time_duration.count() << " ms" << std::endl;

    return PolyCoeff1D;
}

Eigen::VectorXd TRAJECTORY_GENERATOR::closedFormCalCoeff1D(const Eigen::VectorXd &WayPoints1D,
                                                            const Eigen::VectorXd &StartState1D,
                                                            const Eigen::VectorXd &EndState1D,
                                                            const Eigen::VectorXd &S1D,
                                                            const int seg_num)
{   
    int df_and_dp_num = _dev_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * _dev_order + mid_waypts_num;
    int dp_num = (_dev_order - 1) * mid_waypts_num;

    auto time_start = std::chrono::high_resolution_clock::now();

    Eigen::MatrixXd C = _Ct.transpose();
    Eigen::MatrixXd M_inv = _M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::VectorXd PolyCoeff1D;
        
    Eigen::MatrixXd R = C * M_inv_tran * (_Q + Ksoft * _G * _G.transpose()) * M_inv * _Ct;
    // Eigen::MatrixXd R = C * M_inv_tran * (Ksoft * _G * _G.transpose()) * M_inv * _Ct;
    
    Eigen::MatrixXd H = 2 * Ksoft * S1D.transpose() * _G.transpose() * M_inv * _Ct;

    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

    Eigen::MatrixXd H_p = H.block(0, df_num, H.rows(), dp_num);

    // compute dF
    Eigen::VectorXd dF(df_num);
    dF.head(_dev_order) = StartState1D;    // start state: pos,vel,acc,(jerk)
    dF.segment(_dev_order, mid_waypts_num) = WayPoints1D.segment(1, WayPoints1D.rows() - 2);  // middle waypoints: pos
    dF.tail(_dev_order) = EndState1D;      // end state: pos,vel,acc,(jerk)
    // cout << "dF = " << endl;
    // cout << dF << endl;
    
    Eigen::VectorXd dP = R_pp.inverse() * (0.5 * H_p.transpose() - R_fp.transpose() * dF);   // closed-form solution of Unconstrained quadratic programming

    Eigen::VectorXd dF_and_dP(df_and_dp_num);
    dF_and_dP << dF, dP;
    PolyCoeff1D = M_inv * _Ct * dF_and_dP;   // all coefficients of one Dof

    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_duration = time_end - time_start; // 秒为单位
    std::cout << "Trajectory Generation Time is " << time_duration.count() << " ms" << std::endl;
    
    return PolyCoeff1D;
}

std::vector<double> TRAJECTORY_GENERATOR::timeAllocation(std::vector<Eigen::Vector3d>& Path, double vel, Eigen::Vector3d start_vel_){
    std::vector<double> time;
    // The time allocation is many relative timelines but not one common timeline
    if(vel == -1){  //默认值为yaml文件中设置的值
        vel = _Vel;
    }
    double v0 = start_vel_.norm(); //目前暂未考虑初始速度，后期有待补充

    for(int i = 0; i < (Path.size() - 1); i++)
    {
        double distance = (Path[i+1] - Path[i]).norm();    // or .lpNorm<2>()
        if(Path.size() == 2){   //如果只有一段的话
            double x1 = abs(vel * vel - v0 * v0) / (2 * _Acc); 
            double x2 = distance - 2 * x1;
            double t1 = abs(vel - v0) / _Acc;
            double t2 = x2 / vel;
            if(t2 < 0){
                t2 = 0;
            }
            time.push_back(2 * t1 + t2);
        }
        else{
            if(i == 0){  //首段和末端考虑匀速和加/减速
                double x1 = abs(vel * vel - v0 * v0) / (2 * _Acc); 
                double x2 = distance - x1;
                double t1 = abs(vel - v0) / _Acc;
                double t2 = x2 / vel;
                if(t2 < 0){
                    t2 = 0;
                }
                time.push_back(t1 + t2);  
            }
            else if(i == Path.size() - 2){  //首段和末端考虑匀速和加/减速
                double x1 = vel * vel / (2 * _Acc); 
                double x2 = distance - x1;
                double t1 = vel / _Acc;
                double t2 = x2 / vel;
                if(t2 < 0){
                    t2 = 0;
                }
                time.push_back(t1 + t2);  
            }
            else{
                time.push_back(distance / vel);  //其余匀速
            }
            // else if(i == Path.size() - 2){  //首段和末端考虑匀速和加/减速
            //     double x1 = v0 * v0 / (2 * _Acc); 
            //     double x2 = distance - x1;
            //     double t1 = vel / _Acc;
            //     double t2 = x2 / v0;
            //     if(t2 < 0){
            //         t2 = 0;
            //     }
            //     time.push_back(t1 + t2);  
            // }
            // else{
            //     time.push_back(distance / v0);  //其余匀速
            // }
        }
    }
    return time;
}

std::pair<int, bool> TRAJECTORY_GENERATOR::getTrajID(const MiniSnapTraj& traj, double time){
    std::pair<int, bool> ID_info;
    ID_info.first = -1;
    ID_info.second = true;
    //判断当前时间处于哪段轨迹当中
    for (int i = 0; i < traj.traj_nums; i++){
        if (time < traj.poly_time[i]){
            ID_info.first = i;

            // if(abs(time - traj.poly_time[i]) < insert_time_thresh){
            //     ID_info.second = false;
            // }
            // if(i > 0){
            //     if(abs(time - traj.poly_time[i - 1]) < insert_time_thresh){
            //         ID_info.second = false;
            //     }
            // }
            
            break;
        }
        if (time == traj.poly_time[i]){
            ID_info.first = i;
            ID_info.second = false;
            break;
        }
    }
    return ID_info;
}

Traj_info  TRAJECTORY_GENERATOR::getTrajInfo(const MiniSnapTraj& traj, double time){
    int     traj_id = -1;
    double  t;
    //判断当前时间处于哪段轨迹当中
    for (int i = 0; i < traj.traj_nums; i++){
        if (time <= traj.poly_time[i]){
            traj_id = i;
            t = (i == 0 ? time : time - traj.poly_time[i - 1]);
            break;
        } 
    }
    if(traj_id == -1){
        traj_id = traj.traj_nums - 1;
        t = traj.poly_time[traj.traj_nums - 1] - traj.poly_time[traj.traj_nums - 2];
    }

    Vector3d position, velocity, acceleration;
    for (int dim = 0; dim < 3; dim++){
        VectorXd coeff      = (traj.poly_coeff[traj_id]).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd posi_time  = VectorXd::Zero( _poly_num1D );
        VectorXd vel_time   = VectorXd::Zero( _poly_num1D );
        VectorXd acc_time   = VectorXd::Zero( _poly_num1D );

        posi_time(0) = 1.0;
        vel_time(0)  = std::numeric_limits<double>::min();
        acc_time(0)  = std::numeric_limits<double>::min();
        acc_time(1)  = std::numeric_limits<double>::min();
        
        for (int j = 1; j < _poly_num1D; j++){
            if (j == 1){
                posi_time(j) = pow(t, j);
                vel_time(j)  = 1;
            }else if(j == 2){
                posi_time(j) = pow(t, j);
                vel_time(j)  = j * pow(t, j - 1); 
                acc_time(j) = 1;
            }
            else{
                posi_time(j) = pow(t, j);
                vel_time(j)  = j * pow(t, j - 1);
                acc_time(j)  = j * (j - 1) * pow(t, j - 2);                 
            }
        }
        position(dim) = coeff.dot(posi_time);
        velocity(dim) = coeff.dot(vel_time);
        acceleration(dim) = coeff.dot(acc_time);
    }
    if(height_fix){
        position.z() = fixed_height;
        velocity.z() = 0.0;
        acceleration.z() = 0.0;
    }

    Traj_info t_info(position, velocity, acceleration);

    return t_info;
}

MiniSnapTraj TRAJECTORY_GENERATOR::trajGeneration(std::vector<Eigen::Vector3d>& path, ros::Time Time_start, double vel_desire, Eigen::Vector3d startVel, Eigen::Vector3d endVel, Eigen::Vector3d startAcc, Eigen::Vector3d endAcc, const Soft_waypoints& Softwps){
    std::vector<Eigen::Vector3d> vel;
    std::vector<Eigen::Vector3d> acc;
    vel.push_back(startVel);
    vel.push_back(endVel);
    acc.push_back(startAcc);
    acc.push_back(endAcc);
    // use "trapezoidal velocity" time allocation
    // auto time_start = std::chrono::high_resolution_clock::now();
    auto _polyTime = timeAllocation(path, vel_desire, startVel);
    // std::cout << "Time: " ;
    // for(int i = 0; i < _polyTime.size(); i++){
    //     std::cout << _polyTime[i] << ", ";
    // }
    // std::cout << std::endl;
    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    // _polyTime = timeRefine(path, vel, acc, _polyTime);
    // auto time_start = std::chrono::high_resolution_clock::now();
    auto _polyCoeff = PolyQPGeneration(path, vel, acc, _polyTime, Softwps);
    // auto time_end = std::chrono::high_resolution_clock::now();
    MiniSnapTraj Traj = MiniSnapTraj(Time_start, _polyTime, _polyCoeff, path);
    // std::chrono::duration<double, std::milli> time_duration = time_end - time_start; // 秒为单位
    // std::cout << "Time consumed in trajectory generation is " << time_duration.count() << " ms" << std::endl;
    std::cout << "The duration of trajectory is " << Traj.time_duration << " s" << std::endl; 
    //轨迹可视化
    //getVisual(Traj);
    Traj.start_time = ros::Time::now();
    return Traj;
} 


std::pair<MiniSnapTraj, bool> TRAJECTORY_GENERATOR::waypointInsert(MiniSnapTraj Traj, Eigen::Vector3d& point_insert, double time_insert, double time_start, int& hwps_num, Soft_waypoints Softwps){
    bool new_hwp_flag = false;
    std::pair<int, bool> start_trajID_info = getTrajID(Traj, time_start);
    std::pair<int, bool> insert_trajID_info = getTrajID(Traj, time_insert); 


    int start_trajID = start_trajID_info.first;
    int insert_trajID = insert_trajID_info.first;

    int start_pointID = start_trajID + 1 - hwps_num;
    int insert_pointID = insert_trajID + 1 - hwps_num;

    // std::cout << "seg_num: " << Traj.traj_nums << std::endl;
    // std::cout << "startID: " << start_trajID << std::endl;
    // std::cout << "insertID: " << insert_trajID << std::endl;
    // std::cout << "hardwpsID: " << Traj.hard_waypoints_ID << std::endl;

    Traj_info start_info = getTrajInfo(Traj, time_start);
    Eigen::Vector3d start_pos = start_info.position;
    Eigen::Vector3d start_vel = start_info.velocity;

    double start_vel_norm = start_vel.norm();    
    if(start_vel_norm > start_vel_lim){
        start_vel = start_vel_lim * start_vel.normalized();
    }

    Eigen::Vector3d start_acc = start_info.acceleration;
    // std::cout << "start_vel: " << start_vel.x() << " " << start_vel.y() << " " << start_vel.z() << std::endl;
    // std::cout << "start_acc: " << start_acc.x() << " " << start_acc.y() << " " << start_acc.z() << std::endl;

    //PS1：仅更新startpoint之后的那部分轨迹
    //insertpoint为动态插入，并不永久添加到traj.waypoints中，而是暂存到traj.hardwaypont(下文简称为hwps)，仅在更新轨迹前临时插入到路标点序列中
    //这意味着上次参与更新的hwps将不参与本次更新，而仅考虑当前帧的insertpoint,以避免相邻两帧hwps过近引发错误
    //这会带来各方面ID的微调，如下面代码

    if(start_trajID_info.second){  //start_point 无异常
        std::cout << "normal insert waypoint" << std::endl;
        if(start_trajID < Traj.hard_waypoints_ID){   //每次更新仅更新startpoint以后的那部分轨迹，此时上次更新的hwps仍处在更新范围内，
                                                     // 需要将其剔除并换为新的insertpoint，轨迹长度只增加1
            Traj.traj_nums += 1; 
            if(insert_trajID >= Traj.hard_waypoints_ID){
                insert_trajID -= 1; //如果insertpoint位于上次更新后的hwps的后面,由于上次的hwps不参与本次更新，即它不会存在于本次的路标点序列里，这会使得insert_trajID减一
                insert_pointID -= 1;
            }
        }
        else{
            Traj.traj_nums += 2;//如果startpoint位于上次更新的hwps之后，由于每次只更新startpoint以后的部分，不会影响其之前的轨迹信息，此时路标点序列直接新增insertpoint和startpont,长度加二
            if(Traj.hard_waypoints_ID != -1){
                start_pointID -= 1;
                insert_pointID -= 1;    //发现新的历史hwp,历史hardwps生成的两段轨迹已经永久存在，tajID不会变，但是历史hardwps不会加入到wps序列中,pointID减一
                new_hwp_flag = true;
            }
        }

        Traj.waypoints.insert(Traj.waypoints.begin() + start_pointID, start_pos); // start_point直接插到原轨迹的路标点中
        insert_trajID += 1;  //point_start插入后,会使insert_trajID增加1
        insert_pointID += 1;
    }
    else{ //startpoint异常，不插入，则整体Traj.traj_nums变化量减一
        std::cout << "abnormal insert waypoint" << std::endl;
        if(start_trajID >= Traj.hard_waypoints_ID){
            Traj.traj_nums += 1;
        }
        else if(insert_trajID >= Traj.hard_waypoints_ID){
            insert_trajID -= 1;
            insert_pointID -= 1;
        }
    }

    // std::cout << "waypoints: " << std::endl;
    // for(int i = 0; i < Traj.waypoints.size(); i++){
    //     std::cout << i << ": " << Traj.waypoints[i] << std::endl;
    // }
    
    std::vector<Eigen::Vector3d> tem_wps = Traj.waypoints;
    tem_wps.insert(tem_wps.begin() + insert_pointID, point_insert); //point_insert仅插入临时的路标点序列中用于计算，不加入到轨迹自身的路标点中
    
    std::vector<Eigen::Vector3d> subpath(tem_wps.begin() + start_pointID, tem_wps.end());

    for(int i = 0; i < Softwps.times.size(); i++){
        Softwps.times[i] -= time_start;
    }
    // std::cout << "subpath:" << std::endl;
    // for(int i = 0; i < subpath.size(); i++){
    //     std::cout << i << ": " << subpath[i] << std::endl;
    // }

    // MiniSnapTraj subtraj = trajGeneration(subpath, ros::Time::now(), -1, start_vel, Eigen::Vector3d(0.0, 0.0, 0.0), start_acc, Eigen::Vector3d(0.0, 0.0, 0.0), Softwps);
    MiniSnapTraj subtraj = trajGeneration(subpath, ros::Time::now(), -1, start_vel, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Softwps);
    std::vector<Eigen::VectorXd> new_coeff;
    std::vector<double> new_time;
    double new_duration = 0.0;
    for(int i = 0; i < Traj.traj_nums; i++){
        if(i < start_trajID){
            new_coeff.push_back(Traj.poly_coeff[i]);
            new_time.push_back(Traj.poly_time[i]);
        }
        else if(i == start_trajID){
            new_coeff.push_back(Traj.poly_coeff[i]);
            new_time.push_back(time_start);
        }
        else{
            new_coeff.push_back(subtraj.poly_coeff[i - (start_trajID + 1)]);
            new_time.push_back(subtraj.poly_time[i - (start_trajID + 1)] + time_start);
        }
    }
    Traj.poly_coeff = new_coeff;
    Traj.poly_time = new_time;
    Traj.time_duration = new_time[Traj.traj_nums - 1];
    // std::cout << "WPS_nums: " << Traj.waypoints.size() << std::endl;
    Traj.hard_waypoints = point_insert;
    Traj.hard_waypoints_ID = insert_trajID + 1;
    std::pair<MiniSnapTraj, bool> result;
    result.first = Traj;
    result.second = new_hwp_flag;
    return result;
}

void TRAJECTORY_GENERATOR::get_G_S(const Soft_waypoints& Softwps, const std::vector<double>& Time)
{
    int seg_num = Time.size();
    // std::cout << "seg_num: " << seg_num << std::endl;
    _G = Eigen::MatrixXd::Zero(seg_num * _poly_num1D, Softwps.points.size());
    std::vector<int> ID_list;    //存储每个Soft_waypoint的ID(在轨迹中所属的段数)
    std::vector<double> t_list;
    for (int i = 0; i < Softwps.times.size(); i++){
        double t = Softwps.times[i];
        // std::cout << i << "th Softytime: " << t << std::endl;
        for(int j = 0; j < seg_num; j++){
            double t_ = t;  //缓存相减之前的t
            t = t - Time[j];
            if(t < 0){
                // std::cout << "j1: " << j << std::endl;
                // std::cout << "t_: " << t_ << std::endl;
                ID_list.push_back(j);
                t_list.push_back(t_);
                break;
            }
        }
    }
    
    for(int i = 0; i < Softwps.points.size(); i++){
        for(int j = 0; j < _poly_num1D; j++){
            // std::cout << "j2: " << j <<" ID: " << ID_list[i] << " _poly_num1D: " << _poly_num1D << " i: " << i << std::endl;
            // std::cout << "cols: " << _G.cols() << " rows: " << _G.rows() << std::endl;
            _G(j + ID_list[i] * _poly_num1D, i) = pow(t_list[i], j);
        }
    }
    _S = Eigen::MatrixXd::Zero(Softwps.points.size(), 3);
    for (int i = 0; i < Softwps.points.size(); ++i) {
        _S.row(i) = Softwps.points[i];
    }
}


std::pair<double, double> TRAJECTORY_GENERATOR::calculate_yaw(Traj_info& t_info){
    Vector3d pos = t_info.position;
    Vector3d vel = t_info.velocity;
    Vector3d acc = t_info.acceleration;

    double dx = vel.x();
    double dy = vel.y();

    double ddx = acc.x();
    double ddy = acc.y();

    double yaw = atan(dy / dx);
    double dyaw =  ((ddy - dy * ddx) / dx) / (1 + (dy / dx) * (dy / dx));
        
    if(dx < 0){               //由于反正切的值域是-PI/2到PI/2,而dx<0的部分在该范围之外，故额外叠加个PI
        yaw = yaw - M_PI;
    }
    
    std::pair<double, double> yaw_dyaw;
    yaw_dyaw.first = yaw;
    yaw_dyaw.second = dyaw;
    return yaw_dyaw; 
}

