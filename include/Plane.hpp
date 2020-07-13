#ifndef OBJECT_DISCOVERY_PLANE_HPP
#define OBJECT_DISCOVERY_PLANE_HPP



struct CAMERA_INFO{
    double r0, c0;
    double fr, fc;
    double scale;
};


struct DEPTH_PIXEL
{
    int r, c;
    double inv_d;
};

class Plane
{
public:
    double a, b, c, d;
    double alpha, beta, gamma, theta;
    cv::Mat mask;

    // Plane Plane(){
    //     a=0.0;
    //     b=0.0;
    //     c=0.0;
    //     d=0.0;
    //     alpha=0.0;
    //     beta=0.0;
    //     gamma=0.0;
    //     theta=0.0;
    // }

    bool in_plane(DEPTH_PIXEL &p, double distance)
    {
        return calculate_distance(p)<distance;
    }

    double calculate_distance(DEPTH_PIXEL &p)
    {
        return abs((alpha*p.c+beta*p.r+gamma+theta*p.inv_d)/p.inv_d)/sqrt(1+d*d);
    }

    double is_normalized()
    {
        return abs(a*a+b*b+c*c-1)<0.0001;
    }

private:
    void fake2real(CAMERA_INFO camera)
    {
        a = alpha*camera.fc*camera.scale;
        b = beta*camera.fr*camera.scale;
        c = camera.scale*(gamma+alpha*camera.c0+beta*camera.r0);
        d = theta;
    }

};

#endif