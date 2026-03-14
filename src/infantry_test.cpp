#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/Infantry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // tools::Plotter plotter;

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  ////DEBUG////
  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;
  //////////////////
  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    // auto command = aimer.aim(targets, t, cboard.bullet_speed);
    auto command = aimer.aim(targets, t, 14);

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    // command.shoot = true;
    cboard.send(command);

    ///////////DEBUG/////////
    tools::draw_text(
    img,
    fmt::format(
    "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
    command.pitch * 57.3, command.shoot),
    {10, 60}, {154, 50, 205});

    tools::draw_text(
        img,
        fmt::format(
        "gimbal yaw{:.2f} pitch{:.2f}", (tools::eulers(q.toRotationMatrix(), 2, 1, 0) * 57.3)[0], (tools::eulers(q.toRotationMatrix(), 2, 1, 0) * 57.3)[1]),
        {10, 90}, {255, 255, 255});

            
    if (!targets.empty()) {
      auto target = targets.front();

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      //debuggbl
      tools::draw_text(img, 
        fmt::format("armor_xyza {:.2f},  {:.2f},  {:.2f},  {:.2f}", armor_xyza_list[0](0), armor_xyza_list[0](1), armor_xyza_list[0](2), armor_xyza_list[0](3))
        ,{10, 120}, {255, 255, 255});

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});
    }

    
    nlohmann::json data;
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}