#include <chrono>
#include <memory>

#include <libusb.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

/** @file

  @brief ROS2 driver interface for UEYE-compatible USB digital cameras.

*/
#define BUF85SIZE 1048576

namespace driver_flir
{

  class DriverFlir
  {
  public:
    // Use shared pointers for node instances
    DriverFlir( rclcpp::Node::SharedPtr nh,
                rclcpp::Node::SharedPtr priv_nh,
                rclcpp::Node::SharedPtr camera_nh );
    ~DriverFlir();
    void poll(void);
    void setup(void);
    void shutdown(void);

    bool ok();

  private:
    void publish(const sensor_msgs::msg::Image::SharedPtr &image);
    void read(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]);
    void print_bulk_result(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]);

    libusb_context *context;
    struct libusb_device_handle *devh;

    unsigned char buf[1048576];
    int actual_length;
    char EP81_error[50];
    char EP83_error[50];
    char EP85_error[50];
    int buf85pointer = 0;
    unsigned char buf85[BUF85SIZE];

    enum states_t {INIT, INIT_1, INIT_2, ASK_ZIP, ASK_VIDEO, POOL_FRAME, ERROR};
    states_t states;

    enum setup_states_t {SETUP_INIT, SETUP_LISTING, SETUP_FIND, SETUP_SET_CONF, SETUP_CLAIM_INTERFACE_0, SETUP_CLAIM_INTERFACE_1, SETUP_CLAIM_INTERFACE_2, SETUP_ALL_OK, SETUP_ERROR};
    setup_states_t setup_states;

    int error_code;
    bool isOk;

    // Consider switching to std::chrono for time keeping
    std::chrono::steady_clock::time_point t1, t2;
    long long fps_t;
    int vendor_id;
    int product_id;

    rclcpp::Node::SharedPtr nh_;         // node pointer
    rclcpp::Node::SharedPtr priv_nh_;      // private node pointer
    rclcpp::Node::SharedPtr camera_nh_;    // camera namespace node pointer
    std::string camera_name_;              // camera name
    std::string camera_frame_;             // camera frame name

    /** image transport interfaces */
    std::shared_ptr<image_transport::ImageTransport> it_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_8b_pub_;
  };
};
