#include <opencv2/opencv.hpp>

namespace cv {
    enum RobotWorldHandEyeCalibrationMethod
    {
        CALIB_ROBOT_WORLD_HAND_EYE_SHAH = 0, //!< Solving the robot-world/hand-eye calibration problem using the kronecker product @cite Shah2013SolvingTR
        CALIB_ROBOT_WORLD_HAND_EYE_LI   = 1  //!< Simultaneous robot-world and hand-eye calibration using dual-quaternions and kronecker product @cite Li2010SimultaneousRA
    };

    // Kronecker product or tensor product
    // https://stackoverflow.com/a/36552682
    static Mat kron(const Mat& A, const Mat& B)
    {
        CV_Assert(A.channels() == 1 && B.channels() == 1);

        Mat1d Ad, Bd;
        A.convertTo(Ad, CV_64F);
        B.convertTo(Bd, CV_64F);

        Mat1d Kd(Ad.rows * Bd.rows, Ad.cols * Bd.cols, 0.0);
        for (int ra = 0; ra < Ad.rows; ra++)
        {
            for (int ca = 0; ca < Ad.cols; ca++)
            {
                Kd(Range(ra*Bd.rows, (ra + 1)*Bd.rows), Range(ca*Bd.cols, (ca + 1)*Bd.cols)) = Bd.mul(Ad(ra, ca));
            }
        }

        Mat K;
        Kd.convertTo(K, A.type());
        return K;
    }

    static Mat_<double> normalizeRotation(const Mat_<double>& R_)
    {
        // Make R unit determinant
        Mat_<double> R = R_.clone();
        double det = determinant(R);
        if (std::fabs(det) < FLT_EPSILON)
        {
            CV_Error(Error::StsNoConv, "Rotation normalization issue: determinant(R) is null");
        }
        R = std::cbrt(std::copysign(1, det) / std::fabs(det)) * R;

        // Make R orthogonal
        Mat w, u, vt;
        SVDecomp(R, w, u, vt);
        R = u*vt;

        // Handle reflection case
        if (determinant(R) < 0)
        {
            Matx33d diag(1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, -1.0);
            R = u*diag*vt;
        }

        return R;
    }



    //Reference:
    //M. Shah, "Solving the robot-world/hand-eye calibration problem using the kronecker product"
    //Journal of Mechanisms and Robotics, vol. 5, p. 031007, 2013.
    //Matlab code: http://math.loyola.edu/~mili/Calibration/
    static void calibrateRobotWorldHandEyeShah(const std::vector<Mat_<double>>& cRw, const std::vector<Mat_<double>>& ctw,
                                            const std::vector<Mat_<double>>& gRb, const std::vector<Mat_<double>>& gtb,
                                            Matx33d& wRb, Matx31d& wtb, Matx33d& cRg, Matx31d& ctg)
    {
        Mat_<double> T = Mat_<double>::zeros(9, 9);
        for (size_t i = 0; i < cRw.size(); i++)
        {
            T += kron(gRb[i], cRw[i]);
        }

        Mat_<double> w, u, vt;
        SVDecomp(T, w, u, vt);

        Mat_<double> RX(3,3), RZ(3,3);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                RX(j,i) = vt(0, i*3+j);
                RZ(j,i) = u(i*3+j, 0);
            }
        }

        wRb = normalizeRotation(RX);
        cRg = normalizeRotation(RZ);
        Mat_<double> Z = Mat(cRg.t()).reshape(1, 9);

        const int n = static_cast<int>(cRw.size());
        Mat_<double> A = Mat_<double>::zeros(3*n, 6);
        Mat_<double> b = Mat_<double>::zeros(3*n, 1);
        Mat_<double> I3 = Mat_<double>::eye(3,3);

        for (int i = 0; i < n; i++)
        {
            Mat cRw_ = -cRw[i];
            cRw_.copyTo(A(Range(i*3, (i+1)*3), Range(0,3)));
            I3.copyTo(A(Range(i*3, (i+1)*3), Range(3,6)));

            Mat ctw_ = ctw[i] - kron(gtb[i].t(), I3) * Z;
            ctw_.copyTo(b(Range(i*3, (i+1)*3), Range::all()));
        }

        Mat_<double> t;
        solve(A, b, t, DECOMP_SVD);

        for (int i = 0; i < 3; i++)
        {
            wtb(i) = t(i);
            ctg(i) = t(i+3);
        }
    }

    //Reference:
    //A. Li, L. Wang, and D. Wu, "Simultaneous robot-world and hand-eye calibration using dual-quaternions and kronecker product"
    //International Journal of Physical Sciences, vol. 5, pp. 1530-1536, 2010.
    //Matlab code: http://math.loyola.edu/~mili/Calibration/
    static void calibrateRobotWorldHandEyeLi(const std::vector<Mat_<double>>& cRw, const std::vector<Mat_<double>>& ctw,
                                            const std::vector<Mat_<double>>& gRb, const std::vector<Mat_<double>>& gtb,
                                            Matx33d& wRb, Matx31d& wtb, Matx33d& cRg, Matx31d& ctg)
    {
        const int n = static_cast<int>(cRw.size());
        Mat_<double> A = Mat_<double>::zeros(12*n, 24);
        Mat_<double> b = Mat_<double>::zeros(12*n, 1);
        Mat_<double> I3 = Mat_<double>::eye(3,3);

        for (int i = 0; i < n; i++)
        {
            //Eq 19
            kron(cRw[i], I3).copyTo(A(Range(i*12, i*12 + 9), Range(0, 9)));
            kron(-I3, gRb[i].t()).copyTo(A(Range(i*12, i*12 + 9), Range(9, 18)));

            kron(I3, gtb[i].t()).copyTo(A(Range(i*12 + 9, (i+1)*12), Range(9, 18)));
            Mat cRw_ = -cRw[i];
            cRw_.copyTo(A(Range(i*12 + 9, (i+1)*12), Range(18, 21)));
            I3.copyTo(A(Range(i*12 + 9, (i+1)*12), Range(21, 24)));

            ctw[i].copyTo(b(Range(i*12 + 9, i*12+12), Range::all()));
        }

        Mat_<double> x;
        solve(A, b, x, DECOMP_SVD);

        Mat_<double> RX = x(Range(0,9), Range::all()).reshape(3, 3);
        wRb = normalizeRotation(RX);
        x(Range(18,21), Range::all()).copyTo(wtb);

        Mat_<double> RZ = x(Range(9,18), Range::all()).reshape(3, 3);
        cRg = normalizeRotation(RZ);
        x(Range(21,24), Range::all()).copyTo(ctg);
    }

    void calibrateRobotWorldHandEye(InputArrayOfArrays R_world2cam, InputArrayOfArrays t_world2cam,
                                    InputArrayOfArrays R_base2gripper, InputArrayOfArrays t_base2gripper,
                                    OutputArray R_base2world, OutputArray t_base2world,
                                    OutputArray R_gripper2cam, OutputArray t_gripper2cam,
                                    RobotWorldHandEyeCalibrationMethod method=CALIB_ROBOT_WORLD_HAND_EYE_SHAH) // 这里做了默认初始化
    {
        CV_Assert(R_base2gripper.isMatVector() && t_base2gripper.isMatVector() &&
                R_world2cam.isMatVector() && t_world2cam.isMatVector());

        std::vector<Mat> R_base2gripper_tmp, t_base2gripper_tmp;
        R_base2gripper.getMatVector(R_base2gripper_tmp);
        t_base2gripper.getMatVector(t_base2gripper_tmp);

        std::vector<Mat> R_world2cam_tmp, t_world2cam_tmp;
        R_world2cam.getMatVector(R_world2cam_tmp);
        t_world2cam.getMatVector(t_world2cam_tmp);

        CV_Assert(R_base2gripper_tmp.size() == t_base2gripper_tmp.size() &&
                R_world2cam_tmp.size() == t_world2cam_tmp.size() &&
                R_base2gripper_tmp.size() == R_world2cam_tmp.size());
        CV_Check(R_base2gripper_tmp.size(), R_base2gripper_tmp.size() >= 3, "At least 3 measurements are needed");

        // Convert to double
        std::vector<Mat_<double>> R_base2gripper_, t_base2gripper_;
        std::vector<Mat_<double>> R_world2cam_, t_world2cam_;

        R_base2gripper_.reserve(R_base2gripper_tmp.size());
        t_base2gripper_.reserve(R_base2gripper_tmp.size());
        R_world2cam_.reserve(R_world2cam_tmp.size());
        t_world2cam_.reserve(R_base2gripper_tmp.size());

        // Convert to rotation matrix if needed
        for (size_t i = 0; i < R_base2gripper_tmp.size(); i++)
        {
            {
                Mat rot = R_base2gripper_tmp[i];
                Mat R(3, 3, CV_64FC1);
                if (rot.size() == Size(3,3))
                {
                    rot.convertTo(R, CV_64F);
                    R_base2gripper_.push_back(R);
                }
                else
                {
                    cv::Mat R_temp;
                    Rodrigues(rot, R_temp);
                    R_temp.convertTo(R, CV_64F);
                    R_base2gripper_.push_back(R);
                }
                Mat tvec = t_base2gripper_tmp[i];
                Mat t;
                tvec.convertTo(t, CV_64F);
                t_base2gripper_.push_back(t);
            }
            {
                Mat rot  = R_world2cam_tmp[i];
                Mat R(3, 3, CV_64FC1);
                if (rot.size() == Size(3,3))
                {
                    rot.convertTo(R, CV_64F);
                    R_world2cam_.push_back(R);
                }
                else
                {
                    cv::Mat R_temp;
                    Rodrigues(rot, R_temp);
                    R_temp.convertTo(R, CV_64F);
                    R_world2cam_.push_back(R);
                }
                Mat tvec = t_world2cam_tmp[i];
                Mat t;
                tvec.convertTo(t, CV_64F);
                t_world2cam_.push_back(t);
            }
        }

        CV_Assert(R_world2cam_.size() == t_world2cam_.size() &&
                R_base2gripper_.size() == t_base2gripper_.size() &&
                R_world2cam_.size() == R_base2gripper_.size());

        Matx33d wRb, cRg;
        Matx31d wtb, ctg;
        switch (method)
        {
        case CALIB_ROBOT_WORLD_HAND_EYE_SHAH:
            calibrateRobotWorldHandEyeShah(R_world2cam_, t_world2cam_, R_base2gripper_, t_base2gripper_, wRb, wtb, cRg, ctg);
            break;

        case CALIB_ROBOT_WORLD_HAND_EYE_LI:
            calibrateRobotWorldHandEyeLi(R_world2cam_, t_world2cam_, R_base2gripper_, t_base2gripper_, wRb, wtb, cRg, ctg);
            break;
        }

        Mat(wRb).copyTo(R_base2world);
        Mat(wtb).copyTo(t_base2world);

        Mat(cRg).copyTo(R_gripper2cam);
        Mat(ctg).copyTo(t_gripper2cam);
    }
}