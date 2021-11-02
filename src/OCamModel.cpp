//
// Created by xuqw on 12/4/19.
//

#include "OCamModel.h"

double horner(
        const double* coeffs, const int& s, const double& x)
{
    double res = 0.0;
    for (int i = s - 1; i >= 0; i--)
        res = res * x + coeffs[i];
    return res;
}

OCamModel::OCamModel() :
        c(1),
        d(0),
        e(0),
        u0(0),
        v0(0),
        p((cv::Mat_<double>(1, 1) << 1)),
        invP((cv::Mat_<double>(1, 1) << 1)),
        p_deg(1),
        invP_deg(1),
        Iwidth(0),
        Iheight(0),
        p1(1) {
}

OCamModel::OCamModel(double *cdeu0v0,
                     cv::Mat_<double> p_,
                     cv::Mat_<double> invP_)  :
        c(cdeu0v0[0]),
        d(cdeu0v0[1]),
        e(cdeu0v0[2]),
        u0(cdeu0v0[3]),
        v0(cdeu0v0[4]),
        p(p_),
        invP(invP_) {
    // initialize degree of polynomials
    p_deg = (p_.rows > 1) ? p_.rows : p_deg = p_.cols;
    invP_deg = (p_.rows > 1) ? invP_deg = invP_.rows : invP_deg = invP_.cols;

    cde1 = (cv::Mat_<double>(2, 2) << c, d, e, 1.0);
    p1 = p.at<double>(0);
    invAffine = c - d*e;
}

OCamModel::OCamModel(double *cdeu0v0,
                     cv::Mat_<double> p_,
                     cv::Mat_<double> invP_,
                     double Iw_,
                     double Ih_)  :
        c(cdeu0v0[0]),
        d(cdeu0v0[1]),
        e(cdeu0v0[2]),
        u0(cdeu0v0[3]),
        v0(cdeu0v0[4]),
        p(p_),
        invP(invP_),
        Iwidth(Iw_),
        Iheight(Ih_) {
    // initialize degree of polynomials
    p_deg = (p_.rows > 1) ? p_.rows : p_deg = p_.cols;
    invP_deg = (p_.rows > 1) ? invP_deg = invP_.rows : invP_deg = invP_.cols;

    cde1 = (cv::Mat_<double>(2, 2) << c, d, e, 1.0);
    p1 = p.at<double>(0);
    invAffine = c - d*e;
}

cv::Point2d OCamModel::WorldToImg(cv::Point3d &X) {
    double norm = sqrt(X.x*X.x + X.y*X.y);

    if (norm == 0.0)
        norm = 1e-14;

    const double theta = atan(-X.z / norm);
    const double rho = horner((double*)invP.data, invP_deg, theta);

    const double uu = X.x / norm * rho;
    const double vv = X.y / norm * rho;

    cv::Point2d m;
    m.x = uu*c + vv*d + u0;
    m.y = uu*e + vv + v0;
    return m;
}

cv::Point3d OCamModel::ImgToWorld(cv::Point2d &pixel) {
    cv::Point3d X;
    //double invAff = c - d*e;
    const double u_t = pixel.x - u0;
    const double v_t = pixel.y - v0;
    // inverse affine matrix image to sensor plane conversion
    X.x = (1 * u_t - d * v_t) / this->invAffine;
    X.y = (-e * u_t + c * v_t) / this->invAffine;
    const double X2 = X.x * X.x;
    const double Y2 = X.y * X.y;
    X.z = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

    // normalize vectors spherically
    const double norm = sqrt(X2 + Y2 + X.z*X.z);
    X.x /= norm;
    X.y /= norm;
    X.z /= norm;
    return X;
}