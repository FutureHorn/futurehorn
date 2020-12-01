#include "CubicSpline.h"

// class ubicSpline2D
CubicSpline2D::CubicSpline2D()
{
    m_ds = 0.1;
}

CubicSpline2D::~CubicSpline2D()
{

}

void CubicSpline2D::addEndline(std::vector<double> x, std::vector<double> y, std::vector<double> s, std::vector<double> yaw, std::vector<double> kappa)
{
    double m_s_last = m_s[m_s.size()-1];
    for (int i = 0 ; i < x.size(); i++)
    {
        m_x.push_back(x[i]);
        m_y.push_back(y[i]);
        m_s.push_back(m_s_last+s[i]);
        m_yaw.push_back(yaw[i]);
        m_kappa.push_back(kappa[i]);
    }
}

void CubicSpline2D::setTrajectory(std::vector<cv::Point2d> waypoints)
{
    for (int i = 0 ; i < waypoints.size(); i++)
    {
        m_waypoints_x.push_back(waypoints[i].x);
        m_waypoints_y.push_back(waypoints[i].y);
    }   
    calc_s(m_waypoints_x, m_waypoints_y);
    
    spline_x.init(m_waypoints_s, m_waypoints_x);
    spline_y.init(m_waypoints_s, m_waypoints_y);


    double s = 0;
    while( s<m_waypoints_s[m_waypoints_s.size()-1] )
    {
        cv::Point2d p = get_position(s);
        m_x.push_back(p.x);
        m_y.push_back(p.y);
        m_s.push_back(s);
        m_yaw.push_back(get_yaw(s));
        m_kappa.push_back(get_kappa(s));       
        s=s+m_ds;
    }
}

void CubicSpline2D::calc_s(std::vector<double> x, std::vector<double> y)
{
    m_waypoints_s.push_back(0);
    double s_tmp = 0;
    for (int i = 0 ; i < x.size() ; i++)
    {
        if (i != x.size()-1)
        {
            double dx = x[i+1] - x[i];
            double dy = y[i+1] - y[i];
            double ds = sqrt(dx*dx + dy*dy);
            m_waypoints_ds.push_back(ds);
            s_tmp = s_tmp + ds;
            m_waypoints_s.push_back(s_tmp);
        }
    }
}

cv::Point2d CubicSpline2D::get_position(double s)
{
    double x = spline_x.calc(s);
    double y = spline_y.calc(s);

    return cv::Point2d(x,y);
}

double CubicSpline2D::get_kappa(double s)
{
    double dx = spline_x.calcd(s);
    double ddx = spline_x.calcdd(s);
    double dy = spline_y.calcd(s);
    double ddy = spline_y.calcdd(s);
    
    double kappa = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy),(3 / 2));
       
    return kappa;
}

double CubicSpline2D::get_yaw(double s)
{
    double dx = spline_x.calcd(s);
    double dy = spline_y.calcd(s);
    double yaw = atan2(dy, dx);
    return yaw;
}

void CubicSpline2D::clear()
{
    m_waypoints_x.clear();
    m_waypoints_y.clear();
    m_waypoints_s.clear();
    m_waypoints_ds.clear();
    m_x.clear();
    m_y.clear();
    m_s.clear();
    m_yaw.clear();
    m_kappa.clear();

    std::vector<double>(m_waypoints_x).swap(m_waypoints_x);
    std::vector<double>(m_waypoints_y).swap(m_waypoints_y);
    std::vector<double>(m_waypoints_s).swap(m_waypoints_s);
    std::vector<double>(m_waypoints_ds).swap(m_waypoints_ds);
    std::vector<double>(m_x).swap(m_x);
    std::vector<double>(m_y).swap(m_y);
    std::vector<double>(m_s).swap(m_s);
    std::vector<double>(m_yaw).swap(m_yaw);
    std::vector<double>(m_kappa).swap(m_kappa);

    spline_x.m_x.clear();
    spline_x.m_y.clear();
    spline_x.m_a.clear();
    spline_x.m_b.clear();
    spline_x.m_c.clear();
    spline_x.m_d.clear();
    std::vector<double>(spline_x.m_x).swap(spline_x.m_x);
    std::vector<double>(spline_x.m_y).swap(spline_x.m_y);
    std::vector<double>(spline_x.m_a).swap(spline_x.m_a);
    std::vector<double>(spline_x.m_b).swap(spline_x.m_b);
    std::vector<double>(spline_x.m_c).swap(spline_x.m_c);
    std::vector<double>(spline_x.m_d).swap(spline_x.m_d);

    spline_y.m_x.clear();
    spline_y.m_y.clear();
    spline_y.m_a.clear();
    spline_y.m_b.clear();
    spline_y.m_c.clear();
    spline_y.m_d.clear();
    std::vector<double>(spline_y.m_x).swap(spline_y.m_x);
    std::vector<double>(spline_y.m_y).swap(spline_y.m_y);
    std::vector<double>(spline_y.m_a).swap(spline_y.m_a);
    std::vector<double>(spline_y.m_b).swap(spline_y.m_b);
    std::vector<double>(spline_y.m_c).swap(spline_y.m_c);
    std::vector<double>(spline_y.m_d).swap(spline_y.m_d);


}

// class CubicSpline
CubicSpline::CubicSpline()
{

}
CubicSpline::~CubicSpline()
{

}

void CubicSpline::init(std::vector<double> x, std::vector<double> y)
{
    // self.b, self.c, self.d, self.w = [], [], [], [];
    
    m_x = x;
    m_y = y;

    m_x_size = x.size(); 

    std::vector<double> h;
    for (int i = 0 ; i < x.size(); i++)
    {
        if (i != x.size()-1)
        {
            h.push_back(x[i+1]-x[i]);
        }
    }
    
    m_a = y;

    m_A = calc_A(h);
    m_B = calc_B(h);

    m_C = m_A.colPivHouseholderQr().solve(m_B);

    for (int i = 0 ; i < m_C.rows(); i ++)
    {
        m_c.push_back(m_C(i,0));
    } 
       
    for (int i = 0 ; i < m_x_size-1 ; i++)
    {
        m_d.push_back((m_c[i + 1] - m_c[i]) / (3.0 * h[i]));
        double tb = (m_a[i + 1] - m_a[i]) / h[i] - h[i] * (m_c[i + 1] + 2.0 * m_c[i]) / 3.0;
        m_b.push_back(tb);
    }

}

Eigen::MatrixXd CubicSpline::calc_A(std::vector<double> h)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m_x_size, m_x_size);
    A(0, 0) = 1.0;
    for (int i = 0 ; i < m_x_size-1; i++)
    {
        if (i!= m_x_size-2)
        {
            A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
        }
        A(i + 1, i) = h[i];
        A(i, i + 1) = h[i];
    }
    A(0,1) = 0.0;
  
    A(m_x_size-1, m_x_size-2) = 0.0;
    A(m_x_size-1, m_x_size-1) = 1.0;

    return A;
}

Eigen::MatrixXd CubicSpline::calc_B(std::vector<double> h)
{
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(m_x_size, 1);
       
    for (int i = 0 ; i < m_x_size-2; i++)
    {
        B(i + 1,0) = 3.0 * (m_a[i + 2] - m_a[i + 1]) / h[i + 1]
         - 3.0 * (m_a[i + 1] - m_a[i]) / h[i];
    }  

    return B;
}

double CubicSpline::calc(double t)
{
    int i = search_index(t);
    double dx = t - m_x[i];
    double result = m_a[i] + m_b[i] * dx + m_c[i] * dx * dx + m_d[i] * pow(dx, 3);
      
    return result;
}

double CubicSpline::calcd(double t)
{
    int i = search_index(t);
    double dx = t - m_x[i];
    double result = m_b[i] + 2.0 * m_c[i] * dx + 3.0 * m_d[i] * dx * dx;
    
    return result;
}

double CubicSpline::calcdd(double t)
{
    int i = search_index(t);
    double dx = t - m_x[i];
    double result = 2.0 * m_c[i] + 6.0 * m_d[i] * dx;
    return result;
}

int CubicSpline::search_index(double x)
{
    int i = 0;
    for (i = 0 ; i < m_x.size(); i++)
    {
        if (x > m_x[i] && x < m_x[i+1])
        {
            break;
        }
    }
    return i;
}
