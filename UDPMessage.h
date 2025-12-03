// UDPMessage.h
#ifndef UDPMESSAGE_H
#define UDPMESSAGE_H

#include <Eigen/Dense>
#include <DeviceFactory.h>

struct UDPMessage {
    char flag; // 'e' voor extrinsic, 'i' voor intrinsic, 'p' voor pose
    double data[16] = {0.0}; // Groot genoeg om een 4x4 matrix te bevatten, of een combinatie van parameters

    void set_flag(char flag) {
        this->flag = flag;
    };

    void set_intrinsic_data(CameraCalibration calib, bool is_proj_calib = 0) {
        // Expected: [fx, fy, cx, cy, w, h, ... ]
        this->data[0] = calib.getFocalLengthX();
        this->data[1] = calib.getFocalLengthY();
        this->data[2] = calib.getPrincipalPointX();
        this->data[3] = calib.getPrincipalPointY();
        this->data[4] = calib.getWidth();
        this->data[5] = calib.getHeight();
        this->data[6] = (int)is_proj_calib;
    };

    void set_pose_data(Eigen::Matrix4d mat) { // put in matrix as row major
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                this->data[c * 4 + r] = mat(r, c);
            }
        }
    };
};

#endif // UDPMESSAGE_H
