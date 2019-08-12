#include <condition_variable>
#include <mutex>
#include <stdio.h>
#include <iostream>

#include <ST/CaptureSession.h>
#include "register.hpp"

#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

class SessionDelegate : public ST::CaptureSessionDelegate {
    private:
        std::mutex lock;
        std::condition_variable cond;
        bool ready = false;
        bool done = false;

        ros::Publisher depth_image_pub_;
        ros::Publisher depth_info_pub_;

        ros::Publisher depth_color_aligned_pub_;
        ros::Publisher depth_color_aligned_info_pub_;

        ros::Publisher depth_ir_aligned_pub_;
        ros::Publisher depth_ir_aligned_info_pub_;

        ros::Publisher visible_image_pub_;
        ros::Publisher visible_info_pub_;

        ros::Publisher left_image_pub_;
        ros::Publisher left_info_pub_;
        ros::Publisher right_image_pub_;
        ros::Publisher right_info_pub_;

        ros::Publisher ir_color_aligned_image_pub_;
        ros::Publisher ir_color_aligned_info_pub_;

        ros::Publisher imu_combined_pub_;
        ros::Publisher imu_accel_pub_;
        ros::Publisher imu_gyro_pub_;

        bool new_gyro_data;
        ST::GyroscopeEvent gyro_last_event;

        bool combined_imu = true;

        bool projector_flag;
        bool reboot_flag;

        ros::Subscriber switch_projector_sub;

        std::string camera_name;

        sensor_msgs::ImagePtr imageFromDepthFrame(const std::string& frame_id, const ST::DepthFrame& f)
        {
            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);

            msg->header.frame_id = frame_id;
            msg->header.stamp.fromSec(f.timestamp()); 

            msg->encoding = "16UC1";
            msg->height = f.height();
            msg->width = f.width();
            msg->step = 2*f.width();
            msg->is_bigendian = 0;

            msg->data.resize(msg->height * msg->step);
            uint16_t* data_as_shorts = reinterpret_cast<uint16_t*>(msg->data.data());
            std::transform (&f.depthInMillimeters()[0], &f.depthInMillimeters()[0]+(f.height()*f.width()), &data_as_shorts[0],
                [](float f)->uint16_t{
                    if(std::isnormal(f))
                    {
                        return f;
                    }
                    return 0;
                }
            );

            return msg;
        }


        template <class frameType>
        sensor_msgs::CameraInfoPtr infoFromFrame(const std::string& frame_id, const frameType& f, int width_scale=1)
        {
          sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo);
          info->header.frame_id = frame_id;
          info->header.stamp.fromSec(f.timestamp()); 
          info->height = f.intrinsics().height;
          info->width = f.intrinsics().width/width_scale;
          info->distortion_model = "plumb_bob";
          info->D = { f.intrinsics().k1, f.intrinsics().k2, f.intrinsics().p1, f.intrinsics().p2, f.intrinsics().k3 };
          info->K = { f.intrinsics().fx, 0, f.intrinsics().cx,
                      0, f.intrinsics().fy, f.intrinsics().cy,
                      0, 0, 1. };
          info->P = { f.intrinsics().fx, 0, f.intrinsics().cx, 0,
                      0, f.intrinsics().fy, f.intrinsics().cy, 0,
                      0, 0, 1., 0 };
          info->R = { 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1 };

          return info;
        }

        sensor_msgs::ImagePtr imageFromVisibleFrame(const std::string& frame_id, const ST::ColorFrame& f)
        {
            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);

            msg->header.frame_id = frame_id;
            msg->header.stamp.fromSec(f.timestamp()); //= ros::Time::now();

            int num_channels = f.rgbSize()/(f.width()*f.height());

            if( num_channels == 3){
                msg->encoding = "rgb8";
            } else {
                msg->encoding = "mono8";
            }
            
            msg->height = f.height();
            msg->width = f.width();
            msg->step = f.rgbSize()/f.height();
            msg->is_bigendian = 0;

            msg->data.resize(msg->height * msg->step);

            std::copy(&f.rgbData()[0], &f.rgbData()[0]+f.rgbSize(), &msg->data[0]);

            return msg;
        }

        std::vector<sensor_msgs::ImagePtr> imagesFromInfraredFrame(const std::string& left_frame_id, const std::string& right_frame_id, const ST::InfraredFrame& f, bool as_8bit=false)
        {
            int single_frame_width = f.width()/2;

            sensor_msgs::ImagePtr right(new sensor_msgs::Image);
            right->header.frame_id = right_frame_id;
            right->header.stamp.fromSec(f.timestamp()); 

            right->encoding = "mono16";
            right->height = f.height();
            right->width = single_frame_width;
            right->step = 2*single_frame_width;
            right->is_bigendian = 0;
            right->data.resize(right->height * right->step);
            uint16_t* right_as_shorts = reinterpret_cast<uint16_t*>(right->data.data());

            sensor_msgs::ImagePtr left(new sensor_msgs::Image);
            left->header.frame_id = left_frame_id;
            left->header.stamp.fromSec(f.timestamp()); 

            left->encoding = "mono16";
            left->height = f.height();
            left->width = single_frame_width;
            left->step = 2*single_frame_width;
            left->is_bigendian = 0;
            left->data.resize(left->height * left->step);
            uint16_t* left_as_shorts = reinterpret_cast<uint16_t*>(left->data.data());


            if(as_8bit){
                left->data.resize(left->height*left->width);
                right->data.resize(right->height*right->width);

                left->encoding = "mono8";
                right->encoding = "mono8";

                left->step = single_frame_width;
                right->step = single_frame_width;

                for(int j = 0; j < f.height(); j++){
                    for(int i = 0; i < f.width(); i++){
                        if(i < single_frame_width){
                            right->data[right->width*j + i] = f.data()[f.width()*j + i] >> 3; // Reduce from 11 bits to 8 bits
                        } else {
                            left->data[left->width*j + i - single_frame_width] = f.data()[f.width()*j + i] >> 3; // Reduce from 11 bits to 8 bits
                        }
                    }
                }
            }
            else // Copy as full 16 bit
            {
                for(int j = 0; j < f.height(); j++){
                    // Right is first and main imager
                    std::copy(&f.data()[f.width()*j], &f.data()[f.width()*j + 0] + single_frame_width, &right_as_shorts[right->width*j]);
                    std::copy(&f.data()[f.width()*j+single_frame_width], &f.data()[f.width()*j] + 2*single_frame_width, &left_as_shorts[left->width*j]);
                }
            }

            return {left, right};
        }

        void setAccelerometerData(const ST::AccelerometerEvent& e, sensor_msgs::Imu& imu_msg) {
            const auto accel_data = e.acceleration();
            imu_msg.linear_acceleration.x = accel_data.x;
            imu_msg.linear_acceleration.y = accel_data.y;
            imu_msg.linear_acceleration.z = accel_data.z;
        }

        void setGyroscopeData(const ST::GyroscopeEvent& e, sensor_msgs::Imu& imu_msg) {
            const auto gyro_data = e.rotationRate();
            imu_msg.angular_velocity.x = gyro_data.x;
            imu_msg.angular_velocity.y = gyro_data.y;
            imu_msg.angular_velocity.z = gyro_data.z;
        }

        void publishDepthFrame(const ST::DepthFrame& f)
        {
            if(f.isValid())
            { 
                std::string depth_frame_id = camera_name + "_depth_optical_frame";
                if (depth_info_pub_.getNumSubscribers() > 0) {
                    depth_info_pub_.publish(infoFromFrame(depth_frame_id, f));
                }
                if (depth_image_pub_.getNumSubscribers() > 0) {
                    depth_image_pub_.publish(imageFromDepthFrame(depth_frame_id, f));
                }
            }
        }

        void publishVisibleFrame(const ST::ColorFrame& f)
        {
            if(f.isValid())
            {
                std::string visible_frame_id = camera_name + "_rgb_optical_frame";
                if (visible_image_pub_.getNumSubscribers() > 0) {
                    visible_image_pub_.publish(imageFromVisibleFrame(visible_frame_id, f));
                }
                if (visible_info_pub_.getNumSubscribers() > 0) {
                    visible_info_pub_.publish(infoFromFrame(visible_frame_id, f));
                }
            }
            
        }

        void publishInfraredFrame(const ST::InfraredFrame& f, bool as_8bit=false)
        {
            if(!f.isValid()) //  || (left_image_pub_.getNumSubscribers() == 0 && right_image_pub_.getNumSubscribers() == 0)
            {
                return;
            }
            std::string left_frame_id = camera_name + "_left_optical_frame";
            std::string right_frame_id = camera_name + "_depth_optical_frame";

            std::vector<sensor_msgs::ImagePtr> frames = imagesFromInfraredFrame(left_frame_id, right_frame_id, f, as_8bit);

            if (left_image_pub_.getNumSubscribers() > 0) {
                left_image_pub_.publish(frames[0]);
            }
            if (left_info_pub_.getNumSubscribers() > 0) {
                left_info_pub_.publish(infoFromFrame(left_frame_id, f, 2));
            }

            if (right_image_pub_.getNumSubscribers() > 0) {
                right_image_pub_.publish(frames[1]);
            }
            if (right_info_pub_.getNumSubscribers() > 0) {
                right_info_pub_.publish(infoFromFrame(right_frame_id, f, 2));
            }
        }

        void publishDepthAligned(const ST::DepthFrame& depth, const ST::ColorFrame& visual)
        {
            if(!depth.isValid() || !visual.isValid() || 
              (depth_color_aligned_pub_.getNumSubscribers() == 0 && depth_color_aligned_info_pub_.getNumSubscribers() == 0)) {
                return;
            }

            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
            msg->header.frame_id = camera_name + "_rgb_optical_frame";
            msg->header.stamp.fromSec(visual.timestamp()); 
            msg->encoding = "16UC1";
            msg->height = visual.height();
            msg->width = visual.width();
            msg->step = 2*visual.width();
            msg->is_bigendian = 0;
            register_convert(depth, visual, msg->data);

            // Camera info is same as visual, but with depth timestamp
            auto info = infoFromFrame(msg->header.frame_id, visual);
            info->header.stamp = msg->header.stamp;


            depth_color_aligned_pub_.publish(msg);
            depth_color_aligned_info_pub_.publish(info);
        }

        void publishDepthIRAligned(const ST::DepthFrame& depth, const ST::InfraredFrame& ir)
        {
            if(!depth.isValid() || !ir.isValid() || depth_ir_aligned_pub_.getNumSubscribers() == 0){
                return;
            }

            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
            msg->header.frame_id = camera_name + "_depth_optical_frame";
            msg->header.stamp.fromSec(ir.timestamp()); 
            msg->encoding = "16UC1";
            msg->height = ir.height();
            msg->width = ir.width()/2;
            msg->step = 2*msg->width;
            msg->is_bigendian = 0;
            register_convert(depth, ir, msg->data);

            // Camera info is same as visual, but with depth timestamp
            auto info = infoFromFrame(msg->header.frame_id, ir, 2);
            info->header.stamp = msg->header.stamp;

            depth_ir_aligned_pub_.publish(msg);
            depth_ir_aligned_info_pub_.publish(info);
        }

        void publishIRRGBAligned(const ST::InfraredFrame& ir, const ST::ColorFrame& visual)
        {
            if(!ir.isValid() || !visual.isValid() || ir_color_aligned_image_pub_.getNumSubscribers() == 0){
                return;
            }
            sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
            msg->header.frame_id = camera_name + "_rgb_optical_frame";
            msg->header.stamp.fromSec(visual.timestamp()); 
            msg->encoding = "mono16";
            msg->height = visual.height();
            msg->width = visual.width();
            msg->step = 2*visual.width();
            msg->is_bigendian = 0;

            register_convert(ir, visual, msg->data);
            auto info = infoFromFrame(msg->header.frame_id, visual);
            info->header.stamp = msg->header.stamp;
            ir_color_aligned_image_pub_.publish(msg);
            ir_color_aligned_info_pub_.publish(info);
        }

        void publishIMUCombined(const ST::AccelerometerEvent& ae, const ST::GyroscopeEvent& ge) {
            if (imu_combined_pub_.getNumSubscribers() == 0) return;
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp.fromSec(ae.timestamp());
            setAccelerometerData(ae, imu_msg);
            setGyroscopeData(ge, imu_msg);

            imu_combined_pub_.publish(imu_msg);
        }

        void publishIMUAccel(const ST::AccelerometerEvent& ae) {
            if (imu_accel_pub_.getNumSubscribers() == 0) return;
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp.fromSec(ae.timestamp());
            setAccelerometerData(ae, imu_msg);

            imu_accel_pub_.publish(imu_msg);
        }

        void publishIMUGyro(const ST::GyroscopeEvent& ge) {
            if (imu_gyro_pub_.getNumSubscribers() == 0) return;
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp.fromSec(ge.timestamp());
            setGyroscopeData(ge, imu_msg);

            imu_gyro_pub_.publish(imu_msg);
        }

    public:

        SessionDelegate(ros::NodeHandle& n, ros::NodeHandle& pnh, std::string& camera)
        {
            ros::NodeHandle dn(n, "depth");
            depth_image_pub_ = dn.advertise<sensor_msgs::Image>("image", 10);
            depth_info_pub_ = dn.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle da(n, "depth_aligned");
            depth_color_aligned_pub_ = da.advertise<sensor_msgs::Image>("image", 10);
            depth_color_aligned_info_pub_ = da.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle di(n, "depth_ir_aligned");
            depth_ir_aligned_pub_ = di.advertise<sensor_msgs::Image>("image", 10);
            depth_ir_aligned_info_pub_ = di.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle vn(n, "visible");
            visible_image_pub_ = vn.advertise<sensor_msgs::Image>("image_raw", 10);
            visible_info_pub_ = vn.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle ln(n, "left");
            left_image_pub_ = ln.advertise<sensor_msgs::Image>("image_raw", 10);
            left_info_pub_ = ln.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle rn(n, "right");
            right_image_pub_ = rn.advertise<sensor_msgs::Image>("image_raw", 10);
            right_info_pub_ = rn.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle ia(n, "ir_visible_aligned");
            ir_color_aligned_image_pub_ = ia.advertise<sensor_msgs::Image>("image_raw", 10);
            ir_color_aligned_info_pub_ = ia.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

            ros::NodeHandle imu(n, "imu");
            if (combined_imu) {
                imu_combined_pub_ = imu.advertise<sensor_msgs::Imu>("combined", 10);
            } else {
                imu_accel_pub_ = imu.advertise<sensor_msgs::Imu>("accel", 10);
                imu_gyro_pub_ = imu.advertise<sensor_msgs::Imu>("gyro", 10);
            }

            switch_projector_sub = n.subscribe("switch_projector", 1, &SessionDelegate::switchProjectorCallback, this);
            reboot_flag = false;
            projector_flag = true;

            camera_name = camera;
        }

        void switchProjectorCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            projector_flag = msg->data;
            reboot_flag = true;
        }

        bool getRebootFlag()
        {
            return reboot_flag;
        }

        bool getProjectFlag()
        {
            return projector_flag;
        }

        void resetRebootFlag()
        {
            reboot_flag = false;
        }

        void captureSessionEventDidOccur(ST::CaptureSession *, ST::CaptureSessionEventId event) override {
            std::cout << "Received capture session event " << (int)event << " " << ST::CaptureSessionSample::toString(event) << std::endl;
            switch (event) {
                case ST::CaptureSessionEventId::Ready: {
                    std::unique_lock<std::mutex> u(lock);
                    ready = true;
                    cond.notify_all();
                } break;
                case ST::CaptureSessionEventId::Disconnected:
                case ST::CaptureSessionEventId::EndOfFile:
                case ST::CaptureSessionEventId::Error: {
                    std::unique_lock<std::mutex> u(lock);
                    done = true;
                    cond.notify_all();
                } break;
                default:
                    std::cout << "Event " << (int)event << " unhandled" << std::endl;
            }
        }

        void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample& sample) {
            //printf("Received capture session sample of type %d (%s)\n", (int)sample.type, ST::CaptureSessionSample::toString(sample.type));
            switch (sample.type) {
                case ST::CaptureSessionSample::Type::DepthFrame:
                    //printf("Depth frame: size %dx%d\n", sample.depthFrame.width(), sample.depthFrame.height());
                    publishDepthFrame(sample.depthFrame);
                    break;
                case ST::CaptureSessionSample::Type::VisibleFrame:
                    //printf("Visible frame: size %dx%d\n", sample.visibleFrame.width(), sample.visibleFrame.height());
                    // todo: seperate pub for undistorted
                    publishVisibleFrame(sample.visibleFrame.undistorted());
                    break;
                case ST::CaptureSessionSample::Type::InfraredFrame:
                    //printf("Infrared frame: size %dx%d\n", sample.infraredFrame.width(), sample.infraredFrame.height());
                    publishInfraredFrame(sample.infraredFrame);
                    break;
                case ST::CaptureSessionSample::Type::SynchronizedFrames:
                    {
                        publishDepthFrame(sample.depthFrame);

                        publishVisibleFrame(sample.visibleFrame.undistorted());

//                        publishInfraredFrame(sample.infraredFrame, true);

                        publishDepthAligned(sample.depthFrame, sample.visibleFrame.undistorted());

//                        publishDepthIRAligned(sample.depthFrame, sample.infraredFrame);

                        publishIRRGBAligned(sample.infraredFrame, sample.visibleFrame);
                    }
                    break;
                case ST::CaptureSessionSample::Type::AccelerometerEvent:
                    if (!combined_imu) {
                        publishIMUAccel(sample.accelerometerEvent);
                    } else {
                        if (new_gyro_data) {
                            publishIMUCombined(sample.accelerometerEvent, gyro_last_event);
                        }
                    }
                    break;
                case ST::CaptureSessionSample::Type::GyroscopeEvent:
                    if (!combined_imu) {
                        publishIMUGyro(sample.gyroscopeEvent);
                    } else {
                        gyro_last_event = sample.gyroscopeEvent;
                        new_gyro_data = true;
                    }
                    break;
                default:
                    printf("Sample type %d unhandled\n", (int)sample.type);
            }
        }

        void waitUntilReady() {
            std::unique_lock<std::mutex> u(lock);
            cond.wait(u, [this]() {
                return ready;
            });
        }

        void waitUntilDone() {
            std::unique_lock<std::mutex> u(lock);
            cond.wait(u, [this]() {
                return done;
            });
        }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "structure_driver");

    ros::NodeHandle n;

    ros::NodeHandle pnh("~");

    std::string camera_name;
    pnh.param<std::string>("camera", camera_name, "camera");
    ROS_INFO("camera name %s", camera_name.c_str());

    int serial;
    const char* serial_c;
    pnh.param<int>("serial", serial, -1);
    if (serial == -1)
    {
        ROS_INFO("No serial number set, looking for first sensor");
        serial_c = nullptr;
    }
    else
    {
        ROS_INFO("Serial number: %d", serial);
        serial_c = std::to_string(serial).c_str();
    }

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;

    /** @brief Set to true to enable frame synchronization between visible or color and depth. */
    settings.frameSyncEnabled = true;
    /** @brief Set to true to deliver IMU events on a separate, dedicated background thread. Only supported for Structure Core, currently. */
    settings.lowLatencyIMU = false;
    /** @brief Set to true to apply a correction filter to the depth before streaming. This may effect performance. */
    settings.applyExpensiveCorrection = true;
    /** @brief Set to true to enable depth streaming. */
    settings.structureCore.depthEnabled = true;
    /** @brief Set to true to enable infrared streaming. */
    settings.structureCore.infraredEnabled = false;
    /** @brief Set to true to enable visible streaming. */
    settings.structureCore.visibleEnabled = true;
    /** @brief Set to true to enable accelerometer streaming. */
    settings.structureCore.accelerometerEnabled = true;
    /** @brief Set to true to enable gyroscope streaming. */
    settings.structureCore.gyroscopeEnabled = true;
    /** @brief The target resolution for streamed depth frames. @see StructureCoreDepthResolution */
    settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::_640x480;
    /** @brief The preset depth range mode for streamed depth frames. Modifies the min/max range of the depth values. */
    settings.structureCore.depthRangeMode = ST::StructureCoreDepthRangeMode::Default;
    /** @brief The target resolution for streamed depth frames. @see StructureCoreInfraredResolution
        Non-default infrared and visible resolutions are currently unavailable.
    */
    settings.structureCore.infraredResolution = ST::StructureCoreInfraredResolution::Default;
    /** @brief The target resolution for streamed visible frames. @see StructureCoreVisibleResolution
        Non-default infrared and visible resolutions are currently unavailable.
    */
    settings.structureCore.visibleResolution = ST::StructureCoreVisibleResolution::Default;
    /** @brief Set to true to apply gamma correction to incoming visible frames. */
    settings.structureCore.visibleApplyGammaCorrection = false;
    /** @brief Enable auto-exposure for infrared frames. */
    settings.structureCore.infraredAutoExposureEnabled = true;
    /** @brief Specifies how to stream the infrared frames. @see StructureCoreInfraredMode */
    settings.structureCore.infraredMode = ST::StructureCoreInfraredMode::RightCameraOnly;
    /** @brief The target stream rate for IMU data. (gyro and accel) */
    settings.structureCore.imuUpdateRate = ST::StructureCoreIMUUpdateRate::Default;
    /** @brief Serial number of sensor to stream. If null, the first connected sensor will be used. */
    // settings.structureCore.sensorSerial = serial_c;
    /** @brief Maximum amount of time (in milliseconds) to wait for a sensor to connect before throwing a timeout error. */
    settings.structureCore.sensorInitializationTimeout = 6000;
    /** @brief The target framerate for the infrared camera. If the value is not supported, the default is 30. */
    settings.structureCore.infraredFramerate = 15.f;
    /** @brief The target framerate for the depth sensor. If the value is not supported, the default is 30. */
    settings.structureCore.depthFramerate    = 30.f;
    /** @brief The target framerate for the visible camera. If the value is not supported, the default is 30. */
    settings.structureCore.visibleFramerate  = 30.f;
    /** @brief The initial visible exposure to start streaming with (milliseconds, but set in seconds). */
    settings.structureCore.initialVisibleExposure = 0.006f;
    /** @brief The initial visible gain to start streaming with. Can be any number between 1 and 8. */
    settings.structureCore.initialVisibleGain = 1.0f;
    /** @brief The initial infrared exposure to start streaming with. */
    settings.structureCore.initialInfraredExposure = 0.0146f;
    /** @brief The initial infrared gain to start streaming with. Can be 0, 1, 2, or 3. */
    settings.structureCore.initialInfraredGain = 3;
    /** @brief Setting this to true will eliminate saturation issues, but might result in sparser depth. */
    // settings.structureCore.disableInfraredIntensityBalance = true;
    /** @brief Setting this to true will reduce latency, but might drop more frame */
    settings.structureCore.latencyReducerEnabled = false;
    /** @brief Laser projector power setting from 0.0 to 1.0 inclusive. Projector will only activate if required by streaming configuration. */
    // settings.structureCore.initialProjectorPower = 1.0f;

    SessionDelegate delegate(n, pnh, camera_name);
    ST::CaptureSession session;
    session.setDelegate(&delegate);
    if (!session.startMonitoring(settings)) {
        std::cout << "Failed to initialize capture session" << std::endl;
        return 1;
    }

    std::cout << "Waiting for session to become ready..." << std::endl;
    delegate.waitUntilReady();
    session.startStreaming();

    auto info = session.sensorInfo();
    std::cout << "Connected Sensor SN: " << info.serialNumber << "\n";

    while(ros::ok())
    {
        if (delegate.getRebootFlag())
        {
            // if (delegate.getProjectFlag())
            // {
            //     settings.structureCore.initialProjectorPower = 1.0f;
            // }
            // else
            // {
            //     settings.structureCore.initialProjectorPower = 0.0f;
            // }
            delegate.resetRebootFlag();
            session.stopStreaming();
            if (!session.startMonitoring(settings)) {
                std::cout << "Failed to initialize capture session" << std::endl;
                return 1;
            }

            std::cout << "Waiting for session to become ready..." << std::endl;
            delegate.waitUntilReady();
            session.startStreaming();
        }
        ros::spinOnce();
    }
    session.stopStreaming();
    return 0;
}
