#include <chrono>
#include <ctime>
#include <cstring>
#include <cassert>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>

#include "driver_flir.h"

namespace driver_flir
{

  DriverFlir::DriverFlir(const rclcpp::Node::SharedPtr node) :
    node_(node),
    camera_name_("FLIR_USB"),
    camera_frame_("flir"),
    isOk(true),
    states(INIT),
    setup_states(SETUP_INIT),
    context(nullptr),
    vendor_id(0x09cb),
    product_id(0x1996)
  {
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ir_16b/image_raw", 1);
    image_rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", 1);
    image_8b_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("ir_8b/image_raw", 1);
  }

  DriverFlir::~DriverFlir() {
  }

  void DriverFlir::shutdown() {
    libusb_reset_device(devh);
    libusb_close(devh);
    libusb_exit(nullptr);
    isOk = false;
  }

  bool DriverFlir::ok(void){
    return isOk;
  }

  void DriverFlir::publish(const sensor_msgs::msg::Image::SharedPtr &image) {
    image_pub_->publish(*image);
  }

  void DriverFlir::print_bulk_result(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]) {
    std::time_t now1 = std::time(nullptr);
    if (r < 0) {
      if (strcmp(EP_error, libusb_error_name(r)) != 0) {
        strcpy(EP_error, libusb_error_name(r));
        RCLCPP_ERROR(node_->get_logger(), "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s: %s",
                     std::ctime(&now1), ep , libusb_error_name(r));
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "\n: %s bulk read EP %s, actual length %d", std::ctime(&now1), ep, actual_length);
    }
  }

  void DriverFlir::read(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]) {
    // reset buffer if the new chunk begins with magic bytes or the buffer size limit is exceeded
    unsigned char magicbyte[4] = {0xEF, 0xBE, 0x00, 0x00};

    if ((strncmp(reinterpret_cast<const char*>(buf), reinterpret_cast<const char*>(magicbyte), 4) == 0) ||
        ((buf85pointer + actual_length) >= BUF85SIZE)) {
      buf85pointer = 0;
    }

    memmove(buf85 + buf85pointer, buf, actual_length);
    buf85pointer += actual_length;

    if (strncmp(reinterpret_cast<const char*>(buf85), reinterpret_cast<const char*>(magicbyte), 4) != 0) {
      buf85pointer = 0;
      RCLCPP_ERROR(node_->get_logger(), "Reset buffer because of bad Magic Byte!");
      return;
    }

    uint32_t FrameSize   = buf85[8]  + (buf85[9]  << 8) + (buf85[10] << 16) + (buf85[11] << 24);
    uint32_t ThermalSize = buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
    uint32_t JpgSize     = buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);
    uint32_t StatusSize  = buf85[20] + (buf85[21] << 8) + (buf85[22] << 16) + (buf85[23] << 24);

    if ((FrameSize + 28) > buf85pointer) {
      RCLCPP_ERROR(node_->get_logger(), "wait for next chunk");
      return;
    }
    rclcpp::Time stamp = node_->now();

    // get a full frame, first print status
    t1 = t2;
    gettimeofday(&t2, nullptr);
    fps_t = (19 * fps_t + 10000000 / (((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec))) / 20;

#ifdef DEBUG_
    RCLCPP_INFO(node_->get_logger(), "#%lld/10 fps:", fps_t);
    RCLCPP_INFO(node_->get_logger(), "FrameSize %d ", FrameSize);
    RCLCPP_INFO(node_->get_logger(), "ThermalSize %d ", ThermalSize);
    RCLCPP_INFO(node_->get_logger(), "JpgSize %d ", JpgSize);
    RCLCPP_INFO(node_->get_logger(), "StatusSize %d ", StatusSize);
#endif

    unsigned short pix[160 * 120];
    int v;
    for (uint8_t y = 0; y < 120; ++y) {
      for (uint8_t x = 0; x < 160; ++x) {
        if (x < 80) {
          v = buf85[2 * (y * 164 + x) + 32] + 256 * buf85[2 * (y * 164 + x) + 33];
        } else {
          v = buf85[2 * (y * 164 + x) + 32 + 4] + 256 * buf85[2 * (y * 164 + x) + 33 + 4];
        }
        pix[y * 160 + x] = static_cast<unsigned short>(v);
      }
    }

    cv_bridge::CvImage out_msg;
    cv::Mat im16 = cv::Mat(120, 160, CV_16UC1, pix);
    out_msg.header.frame_id = camera_frame_;
    out_msg.header.stamp = stamp;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    out_msg.image = im16;

    image_pub_->publish(*out_msg.toImageMsg());

    cv::Mat rawRgb = cv::Mat(1, JpgSize, CV_8UC1, &buf85[28 + ThermalSize]);
    cv::Mat decodedImage = cv::imdecode(rawRgb, cv::IMREAD_COLOR);
    cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);

    sensor_msgs::msg::Image ros_img;
    ros_img.header.frame_id = camera_frame_;
    ros_img.header.stamp = stamp;
    auto msg = cv_bridge::CvImage(ros_img.header, "rgb8", decodedImage).toImageMsg();
    image_rgb_pub_->publish(*msg);

    int maxVal = 3847;
    int minVal = 2934;
    cv::Mat im8b = 255 * (im16 - minVal) / (maxVal - minVal);
    im8b.convertTo(im8b, CV_8UC1);
    cv_bridge::CvImage out_8b;
    out_8b.header.frame_id = camera_frame_;
    out_8b.header.stamp = stamp;
    out_8b.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_8b.image = im8b;
    image_8b_pub_->publish(*out_8b.toImageMsg());
  }

  void DriverFlir::poll(void) {
    unsigned char data[2] = {0, 0};
    int r = 0;
    std::time_t now;

    switch (states) {
      case INIT:
        RCLCPP_INFO(node_->get_logger(), "stop interface 2 FRAME");
        r = libusb_control_transfer(devh, 1, 0x0b, 0, 2, data, 0, 100);
        if (r < 0) {
          RCLCPP_ERROR(node_->get_logger(), "Control Out error %d", r);
          error_code = r;
          states = ERROR;
        } else {
          states = INIT_1;
        }
        break;

      case INIT_1:
        RCLCPP_INFO(node_->get_logger(), "stop interface 1 FILEIO");
        r = libusb_control_transfer(devh, 1, 0x0b, 0, 1, data, 0, 100);
        if (r < 0) {
          RCLCPP_ERROR(node_->get_logger(), "Control Out error %d", r);
          error_code = r;
          states = ERROR;
        } else {
          states = INIT_2;
        }
        break;

      case INIT_2:
        RCLCPP_INFO(node_->get_logger(), "\nstart interface 1 FILEIO");
        r = libusb_control_transfer(devh, 1, 0x0b, 1, 1, data, 0, 100);
        if (r < 0) {
          RCLCPP_ERROR(node_->get_logger(), "Control Out error %d", r);
          error_code = r;
          states = ERROR;
        } else {
          states = ASK_ZIP;
        }
        break;

      case ASK_ZIP:
      {
        RCLCPP_INFO(node_->get_logger(), "\nask for CameraFiles.zip on EP 0x83:");
        now = std::time(nullptr);
        RCLCPP_INFO(node_->get_logger(), "\n: %s", std::ctime(&now));
        int transferred = 0;
        char my_string[128];

        int length = 16;
        unsigned char my_string2[16] = {0xcc, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0xF8, 0xB3, 0xF7, 0x00};
        RCLCPP_INFO(node_->get_logger(), "\nEP 0x02 to be sent Hexcode: %i Bytes[", length);
        for (int i = 0; i < length; i++) {
          RCLCPP_INFO(node_->get_logger(), " %02x", my_string2[i]);
        }
        RCLCPP_INFO(node_->get_logger(), " ]");

        r = libusb_bulk_transfer(devh, 2, my_string2, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          RCLCPP_INFO(node_->get_logger(), "\nWrite successful!");
        } else {
          RCLCPP_ERROR(node_->get_logger(), "\nError in write! res = %d and transferred = %d", r, transferred);
        }

        strcpy(my_string, "{\"type\":\"openFile\",\"data\":{\"mode\":\"r\",\"path\":\"CameraFiles.zip\"}}");
        length = strlen(my_string) + 1;
        RCLCPP_INFO(node_->get_logger(), "\nEP 0x02 to be sent: %s", my_string);

        unsigned char *my_string1 = reinterpret_cast<unsigned char*>(my_string);
        r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          RCLCPP_INFO(node_->get_logger(), "\nWrite successful!");
          RCLCPP_INFO(node_->get_logger(), "\nSent %d bytes with string: %s", transferred, my_string);
        } else {
          RCLCPP_ERROR(node_->get_logger(), "\nError in write! res = %d and transferred = %d", r, transferred);
        }

        length = 16;
        unsigned char my_string3[16] = {0xcc, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0xef, 0xdb, 0xc1, 0xc1};
        RCLCPP_INFO(node_->get_logger(), "\nEP 0x02 to be sent Hexcode: %i Bytes[", length);
        for (int i = 0; i < length; i++) {
          RCLCPP_INFO(node_->get_logger(), " %02x", my_string3[i]);
        }
        RCLCPP_INFO(node_->get_logger(), " ]");

        r = libusb_bulk_transfer(devh, 2, my_string3, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          RCLCPP_INFO(node_->get_logger(), "\nWrite successful!");
        } else {
          RCLCPP_ERROR(node_->get_logger(), "\nError in write! res = %d and transferred = %d", r, transferred);
        }

        strcpy(my_string, "{\"type\":\"readFile\",\"data\":{\"streamIdentifier\":10}}");
        length = strlen(my_string) + 1;
        RCLCPP_INFO(node_->get_logger(), "\nEP 0x02 to be sent %i Bytes: %s", length, my_string);
        my_string1 = reinterpret_cast<unsigned char*>(my_string);
        r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
        if (r == 0 && transferred == length) {
          RCLCPP_INFO(node_->get_logger(), "\nWrite successful!");
          RCLCPP_INFO(node_->get_logger(), "\nSent %d bytes with string: %s", transferred, my_string);
        } else {
          RCLCPP_ERROR(node_->get_logger(), "\nError in write! res = %d and transferred = %d", r, transferred);
        }
        now = std::time(nullptr);
        RCLCPP_INFO(node_->get_logger(), "\n: %s", std::ctime(&now));
        states = ASK_VIDEO;
      }
      break;

      case ASK_VIDEO:
        RCLCPP_INFO(node_->get_logger(), "\nAsk for video stream, start EP 0x85:");
        r = libusb_control_transfer(devh, 1, 0x0b, 1, 2, data, 2, 200);
        if (r < 0) {
          RCLCPP_ERROR(node_->get_logger(), "Control Out error %d", r);
          error_code = r;
          states = ERROR;
        } else {
          states = POOL_FRAME;
        }
        break;

      case POOL_FRAME:
      {
        r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length, 200);
        switch(r) {
          case LIBUSB_ERROR_TIMEOUT:
            RCLCPP_ERROR(node_->get_logger(), "LIBUSB_ERROR_TIMEOUT");
            break;
          case LIBUSB_ERROR_PIPE:
            RCLCPP_ERROR(node_->get_logger(), "LIBUSB_ERROR_PIPE");
            break;
          case LIBUSB_ERROR_OVERFLOW:
            RCLCPP_ERROR(node_->get_logger(), "LIBUSB_ERROR_OVERFLOW");
            break;
          case LIBUSB_ERROR_NO_DEVICE:
            RCLCPP_ERROR(node_->get_logger(), "LIBUSB_ERROR_NO_DEVICE");
            break;
        }
        if (actual_length > 0) {
          RCLCPP_INFO(node_->get_logger(), "Received a FRAME %d", actual_length);
          read("0x85", EP85_error, r, actual_length, buf);
        }
      }
      break;

      case ERROR:
        isOk = false;
        break;
    }

    r = libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);
    print_bulk_result("0x81", EP81_error, r, actual_length, buf);
    r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
    print_bulk_result("0x83", EP83_error, r, actual_length, buf);
  }

  void DriverFlir::setup(void) {
    do {
      switch (setup_states) {
        case SETUP_INIT:
          if (libusb_init(&context) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "failed to initialise libusb");
            setup_states = SETUP_ERROR;
          } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully initialised libusb");
            setup_states = SETUP_LISTING;
          }
          break;

        case SETUP_LISTING:
        {
          int rc = 0;
          libusb_device_handle *dev_handle = nullptr;
          libusb_device **devs;
          int count = libusb_get_device_list(context, &devs);
          for (size_t idx = 0; idx < static_cast<size_t>(count); ++idx) {
            libusb_device *device = devs[idx];
            libusb_device_descriptor desc = {0};
            rc = libusb_get_device_descriptor(device, &desc);
            assert(rc == 0);
            RCLCPP_DEBUG(node_->get_logger(), "Vendor:Device = %04x:%04x", desc.idVendor, desc.idProduct);
          }
          libusb_free_device_list(devs, 1);
          setup_states = SETUP_FIND;
          break;
        }

        case SETUP_FIND:
          devh = libusb_open_device_with_vid_pid(context, vendor_id, product_id);
          if (devh == nullptr) {
            RCLCPP_ERROR(node_->get_logger(), "Could not find/open device. devh: %p", devh);
            setup_states = SETUP_ERROR;
          } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully found the Flir One G2 device");
            setup_states = SETUP_SET_CONF;
          }
          break;

        case SETUP_SET_CONF:
          RCLCPP_INFO(node_->get_logger(), "Setting USB configuration 3");
          if (int r = libusb_set_configuration(devh, 3) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "libusb_set_configuration error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully set usb configuration 3");
            setup_states = SETUP_CLAIM_INTERFACE_0;
          }
          break;

        case SETUP_CLAIM_INTERFACE_0:
          if (int r = libusb_claim_interface(devh, 0) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "libusb_claim_interface 0 error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully claimed interface 0");
            setup_states = SETUP_CLAIM_INTERFACE_1;
          }
          break;

        case SETUP_CLAIM_INTERFACE_1:
          if (int r = libusb_claim_interface(devh, 1) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "libusb_claim_interface 1 error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully claimed interface 1");
            setup_states = SETUP_CLAIM_INTERFACE_2;
          }
          break;

        case SETUP_CLAIM_INTERFACE_2:
          if (int r = libusb_claim_interface(devh, 2) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "libusb_claim_interface 2 error %d", r);
            setup_states = SETUP_ERROR;
          } else {
            RCLCPP_INFO(node_->get_logger(), "Successfully claimed interface 2");
            setup_states = SETUP_ALL_OK;
          }
          break;
      }
    } while ((setup_states != SETUP_ERROR) && (setup_states != SETUP_ALL_OK));

    if (setup_states == SETUP_ERROR) {
      shutdown();
    }
  }

}; // namespace driver_flir
