#include <iostream>
#include <thread>
#include <filesystem>

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/publish/map_publisher.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <DeviceFactory.h>

#include <pangolin_viewer/viewer.h>
#include <stella_vslam/util/yaml.h>

#include "UDPSender.h"
#include "UDPMessage.h"
#include "PoseIO.h"
#include "imudatareceiver.h"
#include "ESKF.h"


#define GRAVITY 9.812f  // London g value.
#define SQ(x) (x*x)
#define I_3 (Eigen::Matrix3f::Identity())
#define I_dx (Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity())

using namespace std;

// ------------------------------ Helper functions ------------------------------

ESKF initESKF(float sigma_accel, float sigma_gyro) {
    // float sigma_accel = [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    // float sigma_gyro = [rad/s] (value derived from Noise Spectral Density in datasheet)
    float sigma_accel_drift = 0.001f*sigma_accel; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 0.001f*sigma_gyro; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 10000*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 10000*sigma_gyro_drift; // [rad/s]

    ESKF eskf(
        Eigen::Vector3f(0, 0, -9.81), // Acceleration due to gravity in global frame
        ESKF::makeState(
            Eigen::Vector3f(0, 0, 0), // init pos
            Eigen::Vector3f(0, 0, 0), // init vel
            Eigen::Quaternionf(Eigen::AngleAxisf(0.5f, Eigen::Vector3f(1, 0, 0))), // init quaternion
            Eigen::Vector3f(0, 0, 0), // init accel bias
            Eigen::Vector3f(0, 0, 0) // init gyro bias
            ),
        ESKF::makeP(
            SQ(sigma_init_pos) * I_3,
            SQ(sigma_init_vel) * I_3,
            SQ(sigma_init_dtheta) * I_3,
            SQ(sigma_init_accel_bias) * I_3,
            SQ(sigma_init_gyro_bias) * I_3
            ),
        SQ(sigma_accel),
        SQ(sigma_gyro),
        SQ(sigma_accel_drift),
        SQ(sigma_gyro_drift));
    return eskf;
}

void convert_mat_cv2unity(Eigen::Matrix4d &pose) {
    // Conversion matrices
    Eigen::Matrix4d cv2unity = Eigen::Matrix4d::Identity();
    cv2unity(1,1) = -1.0;

    pose = cv2unity * pose * cv2unity;
}

void save_traj_TUM(const std::string& path, std::vector<Eigen::Matrix4d> cam_poses) {
    std::cout << "[save_traj] Saving trajectory to " + path << std::endl;

    std::ofstream ofs(path, std::ios::out);
    if (!ofs.is_open()) {
        spdlog::critical("cannot create a file at {}", path);
        throw std::runtime_error("cannot create a file at " + path);
    }

    int index = 0;
    for (auto cam_pose_wc : cam_poses) {
        const Eigen::Matrix3d& rot_wc = cam_pose_wc.block<3, 3>(0, 0);
        const Eigen::Vector3d& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
        const Eigen::Quaterniond quat_wc = Eigen::Quaterniond(rot_wc);
        ofs << std::setprecision(15)
            << index << " " // << timestamps.at(frm_id) << " "
            << std::setprecision(9)
            << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
            << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;

        index++;
    }

    std::cout << "[save_traj] Trajectory saved!" << std::endl;
}

// ------------------------------ Helper functions ------------------------------



// ------------------------------ Thread functions ------------------------------

void slam_handler(stella_vslam::system &slam, std::atomic<bool>& is_slam_init, ESKF &kf, std::filesystem::path &cam_calib_path, std::filesystem::path &imu_extrinsic_calib_path, std::atomic<bool>& running, cv::Mat &img, std::atomic<bool>& disable_camera, std::atomic<bool>& render_new_image) {
    // Init realsense
    DeviceFactory factory;
    auto cap = factory.createFirstDevice("RealSense2", cam_calib_path);
    if (!cap) {
        std::cerr << "Kon geen RealSense2 device aanmaken!" << std::endl;
        return exit(-1);
    }

    // Init vars
    cv::Mat depth;
    double timestamp;

    cv::Mat imu2cam_cv; Eigen::Matrix4d imu2cam;
    poseio::loadPose(imu_extrinsic_calib_path, imu2cam_cv);
    cv::cv2eigen(imu2cam_cv, imu2cam);

    float sigma_slam_pos = 0.01f;
    float sigma_slam_rot = 0.05f;
    Eigen::Matrix3f pos_covariance = SQ(sigma_slam_pos)*I_3;
    Eigen::Matrix3f quat_covariance = SQ(sigma_slam_rot)*I_3;

    bool first_loop = true;

    while (running)
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(300));

        cap->captureImages(img, depth, timestamp);

        if (disable_camera) {
            img.setTo(cv::Scalar::all(0));
        }
        render_new_image = true;

        auto cam2slam_ptr = slam.feed_monocular_frame(img, timestamp);
        // auto cam2slam_ptr = slam.feed_RGBD_frame(img, depth, timestamp);

        if (cam2slam_ptr == nullptr) continue;

        Eigen::Matrix4d imu2slam = (*cam2slam_ptr) * imu2cam;

        Eigen::Vector3d pos = imu2slam.block<3,1>(0,3);

        Eigen::Matrix3d R = imu2slam.block<3,3>(0,0);
        Eigen::Quaterniond quat(R);

        kf.measurePos(pos.cast<float>(), pos_covariance);
        kf.measureQuat(quat.cast<float>(), quat_covariance);

        if (first_loop) {
            is_slam_init = true;

            first_loop = false;
        }

        // std::cout << "[slam_handler] pos: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
        // std::cout << "[slam_handler] quat: " << quat << std::endl;
    }
}

void imu_handler_arduino(stella_vslam::system& slam, std::atomic<bool>& is_slam_init, ESKF& kf, std::atomic<bool>& disable_kf, std::atomic<bool>& running, std::filesystem::path &imu_extrinsic_calib_path, std::vector<Eigen::Matrix4d> &tum_poses) {
    cv::Mat imu2cam_cv; Eigen::Matrix4d imu2cam;
    poseio::loadPose(imu_extrinsic_calib_path, imu2cam_cv);
    cv::cv2eigen(imu2cam_cv, imu2cam);

    auto map_publisher = slam.get_map_publisher();

    // -------------------------------- IMU SETUP --------------------------------

    std::cout << "[imuHandler] Setting up serial connection..." << std::endl;
    IMUDataReceiver receiver("/dev/ttyACM0", 115200);
    // IMUDataReceiver receiver("/dev/ttyUSB0", 115200);
    std::cout << "[imuHandler] Connection established!" << std::endl;

    IMUData data;
    const uint16_t header = 0xABCD;

    // -------------------------------- IMU SETUP --------------------------------

    double last_timestamp = 0;
    bool first_loop = true;

    while (!is_slam_init) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (running) {
        // -------------------------------- IMU DATA --------------------------------

        receiver.read(data);

        if (header != data.header) {
            receiver.syncToData(header, sizeof(IMUData));
            continue;
        }

        if (first_loop) {
            last_timestamp = data.timestamp;
            first_loop = false;
            continue;
        }

        // data.print();

        Eigen::Vector3f accel(data.ax, data.ay, data.az);
        Eigen::Vector3f gyro(data.gx, data.gy, data.gz);

        double dt = (data.timestamp - last_timestamp) * 1e-6;

        kf.predictIMU(accel, gyro, dt);

        Eigen::Vector3d pos = kf.getPos().cast<double>();
        Eigen::Quaterniond quat = kf.getQuat().cast<double>();

        Eigen::Matrix4d imu2slam = Eigen::Matrix4d::Identity();
        imu2slam.block<3,3>(0,0) = quat.toRotationMatrix();
        imu2slam.block<3,1>(0,3) = pos;

        Eigen::Matrix4d slam2cam = imu2cam * imu2slam.inverse();
        map_publisher->set_current_cam_pose(slam2cam);

        last_timestamp = data.timestamp;

        // -------------------------------- IMU DATA --------------------------------

        // std::cout << "-----------------------------------" << std::endl;
        // std::cout << "accel: " << accel.x() << " " << accel.y() << " " << accel.z() << std::endl;
        // std::cout << "gyro: " << gyro.x() << " " << gyro.y() << " " << gyro.z() << std::endl;
        // std::cout << "pos: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
        // std::cout << "quat: " << quat << std::endl;
        // std::cout << "slam2cam: " << std::endl << slam2cam << std::endl;

    }
}

void data_sender_handler(stella_vslam::system& slam, std::atomic<bool>& is_slam_init, std::atomic<bool>& running, std::filesystem::path &proj_calib_path, std::filesystem::path &proj_extrinsic_calib_path) {
    cv::Mat cam2proj_cv; Eigen::Matrix4d cam2proj;
    poseio::loadPose(proj_extrinsic_calib_path, cam2proj_cv);
    cv::cv2eigen(cam2proj_cv, cam2proj);

    // Read proj intrinsics
    CameraCalibration proj_calib;
    proj_calib.loadCalibration(proj_calib_path);

    // Init UDP things and send proj intrinsics
    UDPSender sender("127.0.0.1", 5005);
    UDPMessage msg;

    while (!is_slam_init) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Send proj intrinsics
    msg.set_flag('i');
    msg.set_intrinsic_data(proj_calib, true);
    sender.send_data(msg);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Get map_publisher to access camera pose
    auto map_publisher = slam.get_map_publisher();

    // Start sending poses
    msg.set_flag('p');
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto slam2cam = map_publisher->get_current_cam_pose();

        auto slam2proj = cam2proj * slam2cam;
        Eigen::Matrix4d pose = slam2proj.inverse();

        convert_mat_cv2unity(pose);

        msg.set_pose_data(pose);
        sender.send_data(msg);
    }
}

void imshow_handler(cv::Mat &img, std::atomic<bool>& render_new_img, std::atomic<bool>& disable_camera, std::atomic<bool>& disable_kf, std::atomic<bool>& running) {
    while (running) {
        if (!render_new_img) continue;
        render_new_img = false;

        double scale = 0.3;
        cv::Mat output;
        cv::resize(img, output, cv::Size(), scale, scale);

        cv::imshow("img - press 'q' to quit", output);
        auto key = cv::waitKey(30);

        if (key == 'q' || key == 'Q') running = false;
        else if (key == 'd' || key == 'D') {
            disable_camera = !disable_camera;
            std::cout << "disable_camera: " << disable_camera << std::endl;
        }
        else if (key == 'k' || key == 'K') {
            disable_kf = !disable_kf;
            std::cout << "disable_kf: " << disable_kf << std::endl;
        }
    }

    cv::destroyAllWindows();
}

// ------------------------------ Thread functions ------------------------------



// ------------------------------------ Main ------------------------------------

int main(int argc, char**argv)
{
    if (argc < 8) {
        std::cerr << "Missing required parameter:" << std::endl;
        std::cerr << "Usage: ./Stella_VSLAM_Tracking vocab_path slam_config slam_map cam_calib proj_calib proj_extrinsic_calib imu_extrinsic_calib" << std::endl;
        exit(-1);
    }

    std::filesystem::path vocab_path = argv[1];
    std::filesystem::path config_path = argv[2];
    std::filesystem::path path_to_slam_map = std::filesystem::path(argv[3]);
    std::filesystem::path cam_calib_path = std::filesystem::path(argv[4]);
    std::filesystem::path proj_calib_path = std::filesystem::path(argv[5]);
    std::filesystem::path proj_extrinsic_calib_path = std::filesystem::path(argv[6]);
    std::filesystem::path imu_extrinsic_calib_path = std::filesystem::path(argv[7]);

    // Init + start SLAM
    auto config = std::make_shared<stella_vslam::config>(config_path);
    auto slam = std::make_shared<stella_vslam::system>(config, vocab_path);

    spdlog::set_level(spdlog::level::off); // Disable a lot of logging
    slam->startup(false);
    slam->load_map_database(path_to_slam_map);
    slam->disable_mapping_module();

    // Collect poses for later analysis
    std::vector<Eigen::Matrix4d> tum_poses;

    // Thread vars
    std::atomic<bool> running(true);
    std::atomic<bool> render_new_image(false);
    std::atomic<bool> disable_camera(false);
    std::atomic<bool> disable_kf(false);
    std::atomic<bool> is_slam_init(false);

    cv::Mat img;

    ESKF kf = initESKF(0.3f, 0.055f);

    // Start threads. imshow_thread has a big impact on performance !! DO NOT USE IT IF NOT NEEDED !!
    std::thread slam_thread(slam_handler, std::ref(*slam), std::ref(is_slam_init), std::ref(kf), std::ref(cam_calib_path), std::ref(imu_extrinsic_calib_path), std::ref(running), std::ref(img), std::ref(disable_camera), std::ref(render_new_image));
    std::thread data_sender_thread(data_sender_handler, std::ref(*slam), std::ref(is_slam_init), std::ref(running), std::ref(proj_calib_path), std::ref(proj_extrinsic_calib_path));
    std::thread imu_thread_arduino(imu_handler_arduino, std::ref(*slam), std::ref(is_slam_init), std::ref(kf), std::ref(disable_kf), std::ref(running), std::ref(imu_extrinsic_calib_path), std::ref(tum_poses));
    // std::thread imshow_thread(imshow_handler, std::ref(img), std::ref(render_new_image), std::ref(disable_camera), std::ref(disable_kf), std::ref(running));

    // Viewer
    auto viewer = std::make_shared<pangolin_viewer::viewer>(
        stella_vslam::util::yaml_optional_ref(config->yaml_node_, "PangolinViewer"),
        slam,
        slam->get_frame_publisher(),
        slam->get_map_publisher());

    viewer->run();

    // After closing viewer, this code will run
    save_traj_TUM("assets/traj_TUM.txt", tum_poses);

    // Shut everything down correctly
    running = false;
    slam_thread.join();
    imu_thread_arduino.join();
    data_sender_thread.join();
    // imshow_thread.join();

    // viewer->request_terminate();

    return 0;
}

// ------------------------------------ Main ------------------------------------
