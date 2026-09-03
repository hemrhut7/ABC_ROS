#include "my_robot_perception/image_splitter_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <filesystem>
#include <cstdlib>

static void setup_clean_gstreamer_environment()
{
  namespace fs = std::filesystem;
  const std::string src_dir = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0";
  const std::string clean_dir = "/tmp/gst_clean_plugins";

  if (fs::exists(src_dir)) {
    std::error_code ec;
    fs::create_directories(clean_dir, ec);
    for (const auto & entry : fs::directory_iterator(src_dir, ec)) {
      if (entry.is_regular_file() || entry.is_symlink()) {
        std::string filename = entry.path().filename().string();
        if (filename.find("nvargus") == std::string::npos) {
          fs::path target = fs::path(clean_dir) / filename;
          if (!fs::exists(target)) {
            fs::create_symlink(entry.path(), target, ec);
          }
        }
      }
    }
    setenv("GST_PLUGIN_SYSTEM_PATH_1_0", "", 1);
    setenv("GST_PLUGIN_PATH_1_0", clean_dir.c_str(), 1);
    setenv("GST_REGISTRY_1_0", "/tmp/gst_clean_registry.bin", 1);
  }
  setenv("GST_PLUGIN_FEATURE_RANK", "nvarguscamerasrc:NONE,nvv4l2camerasrc:NONE", 1);
}

int main(int argc, char ** argv)
{
  setup_clean_gstreamer_environment();
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<my_robot_perception::ImageSplitterNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("image_splitter_node"), "Node exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
