#include "EKF_core.h"

struct Feature
{
  bool fresh;
  Eigen::Vector3d z;
  Eigen::MatrixXd kti;
  Eigen::MatrixXd hti;
};

struct Dis_item
{ 
  double dis;
  Eigen::Vector3d det;
  Eigen::MatrixXd h;
  Eigen::MatrixXd psi;
};

EKF_core::EKF_core()
{
	sigma_ = Eigen::MatrixXd::Zero(3,3);
	u_ = Eigen::VectorXd::Zero(3);
	R_ = Eigen::Matrix3d::Identity();
	qt_ = Eigen::Matrix3d::Identity();
	N_ = 0;
}

EKF_core::EKF_core(double r1, double r2, double r3, double q1, double q2, double q3)
{
	sigma_ = Eigen::MatrixXd::Zero(3,3);
	u_ = Eigen::VectorXd::Zero(3);
	R_ = Eigen::Matrix3d::Identity();
	R_(0,0) = r1;
	R_(1,1) = r2;
	R_(2,2) = r3;
	qt_ = Eigen::Matrix3d::Identity();
	qt_(0,0) = q1;
	qt_(1,1) = q2;
	qt_(2,2) = q3;
	N_ = 0;
}

bool EKF_core::process(Eigen::Vector3d control, std::vector<Eigen::Vector2d> points)
{
		std::vector<Feature> feature_list;
  	std::vector<Dis_item> dis_list;
    
    Eigen::MatrixXd fx = Eigen::MatrixXd::Zero(3,3+3*N_);
    fx.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    
    control(2) = control(2) - u_(2);

    u_ += fx.transpose()*control;
    sigma_ = sigma_ + fx.transpose()*R_*fx;
        
    for(int i = 0; i < points.size(); i++)
    { 
      double x, y;

      x = points[i][0];
      y = points[i][1];

      Eigen::Vector3d z(x,y,0);
      Eigen::Vector3d new_m;

      new_m(0) = x*cos(u_(2)) - y*sin(u_(2)) + u_(0);
      new_m(1) = x*sin(u_(2)) + y*cos(u_(2)) + u_(1);
      new_m(2) = 0;
      
      for(int j = 0; j < N_; j++)
      {
        Eigen::Vector3d det_v;
        det_v(0) = u_(3+j*3) - u_(0);
        det_v(1) = u_(4+j*3) - u_(1);
        det_v(2) = 0;

        Eigen::Vector3d zkt;
        zkt(0) = det_v(0) * cos(u_(2)) + det_v(1) * sin(u_(2));
        zkt(1) = (-1) * det_v(0) * sin(u_(2)) + det_v(1) * cos(u_(2));
        zkt(2) = 0;
        
        Eigen::MatrixXd fxk = Eigen::MatrixXd::Zero(6,3+3*N_);
        fxk.block(0,0,3,3) = Eigen::Matrix3d::Identity();
        fxk.block(3,3+3*j,3,3) = Eigen::Matrix3d::Identity();
        
        Eigen::MatrixXd h_base(3,6);
        h_base << -cos(u_(2)),  -sin(u_(2)), zkt(1),  cos(u_(2)),  sin(u_(2)), 0,
                  sin(u_(2)),   -cos(u_(2)), -zkt(0), -sin(u_(2)), cos(u_(2)), 0,
                  0,           0,          0,       0,          0,         1; 
    
        Eigen::MatrixXd htk;
        htk = h_base*fxk;
        
        Eigen::MatrixXd psi_k;
        psi_k = htk*sigma_*htk.transpose()+qt_;
        double dis = (z-zkt).transpose() * psi_k.inverse()*(z-zkt);
        
        if (dis < 0)
        {
          dis = -dis;
        }
        
        Dis_item item;
        item.dis = dis;
        item.det = z-zkt;
        item.h = htk;
        item.psi = psi_k;
        dis_list.push_back(item);
        ROS_INFO_STREAM("Push dis " << dis);
      }

      int min_ind = N_+1;
      double min = 10; 

      for(int k = 0; k < dis_list.size(); k++)
      {
        if(dis_list[k].dis < min)
        {
          min = dis_list[k].dis;
          min_ind = k;
        }
      }
      
      Feature feature;
      
      if (min_ind == N_+1)
      {
        ROS_INFO_STREAM("New find");
        Eigen::Matrix3d kti = Eigen::Matrix3d::Zero();
        feature.fresh = true;
        feature.z = new_m;
        feature.kti = qt_;
        feature_list.push_back(feature);
      }
      else 
      {
        ROS_INFO_STREAM("Update old");

        feature.fresh = false;
        feature.z = dis_list[min_ind].det;
        feature.hti = dis_list[min_ind].h;        
        feature.kti = sigma_ * feature.hti.transpose() * dis_list[min_ind].psi.inverse();
        feature_list.push_back(feature);
      }
      dis_list.clear();
    } 
    
    Eigen::VectorXd u_temp = Eigen::VectorXd::Zero(3+3*N_);
    Eigen::MatrixXd psi_temp = Eigen::MatrixXd::Zero(3+3*N_,3+3*N_);
    
    std::vector<Feature>::iterator it;
    for (it = feature_list.begin(); it != feature_list.end(); ++it)
    {      
      if (!(*it).fresh)
      {
        
        u_temp += (*it).kti*(*it).z;
        std::cout << "detz: " << std::endl << (*it).z << std::endl;
        std::cout << "kti: " << std::endl << (*it).kti << std::endl;
        psi_temp += (*it).kti*(*it).hti;
      }
    }
    std::cout << "u_temp: " << std::endl << u_temp << std::endl;
    std::cout << "psi_temp: " << std::endl << psi_temp << std::endl;
    u_ += u_temp;
    sigma_ = (Eigen::MatrixXd::Identity(3+3*N_,3+3*N_) - psi_temp) * sigma_;
    
    for (it = feature_list.begin(); it != feature_list.end(); ++it)
    {
      if ((*it).fresh)
      {
        N_=N_+1;
        Eigen::VectorXd u_temp(3+3*N_);
        u_temp.head(3*N_) = u_;
        u_temp(3*N_) = (*it).z(0);
        u_temp(3*N_+1) = (*it).z(1);
        u_temp(3*N_+2) = 0;
        std::cout << "Resize!" << std::endl;
        u_ = u_temp;
        std::cout << "Resized" << std::endl;
        Eigen::MatrixXd sigma_temp = Eigen::MatrixXd::Zero(3+3*N_,3+3*N_);
        sigma_temp.block(0,0,3*N_,3*N_) = sigma_;
        sigma_temp.block(3*N_,3*N_,3,3) = (*it).kti;
        std::cout << "Resize sigma" << std::endl;
        sigma_ = sigma_temp;       
        std::cout << "Resized sigma" << std::endl;
      }
    }
    std::cout << "u: " << std::endl << u_ << std::endl;
    std::cout << "next" << std::endl;

    return true;
}