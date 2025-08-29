/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 */

/*!
  \example servoFrankaIBVS.cpp

  Example of eye-in-hand image-based control law. We control here a real robot, the
  Franka Emika Panda robot (arm with 7 degrees of freedom). The velocity is
  computed in the camera frame. The inverse jacobian that converts cartesian
  velocities in joint velocities is implemented in the robot low level
  controller. Visual features are the image coordinates of 4 points corresponding
  to the corners of an AprilTag.

  The device used to acquire images is a Realsense D435 device.

  Camera extrinsic (eMc) parameters are set by default to a value that will not match
  your configuration. Use --eMc command line option to read the values from a file.
  This file could be obtained following extrinsic camera calibration tutorial:
  https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic-eye-in-hand.html

  Camera intrinsic parameters are retrieved from the Realsense SDK.

  The target is an AprilTag that is by default 12cm large. To print your own tag, see
  https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-detection-apriltag.html
  You can specify the size of your tag using --tag-size command line option.

*/

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/core/vpMath.h>


#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_FRANKA) && defined(VISP_HAVE_PUGIXML)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                              std::vector<vpImagePoint> *traj_vip)
{
  for (size_t i = 0; i < vip.size(); ++i) {
    if (traj_vip[i].size()) {
      // Add the point only if distance with the previous > 1 pixel
      if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
        traj_vip[i].push_back(vip[i]);
      }
    }
    else {
      traj_vip[i].push_back(vip[i]);
    }
  }
  for (size_t i = 0; i < vip.size(); ++i) {
    for (size_t j = 1; j < traj_vip[i].size(); j++) {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}

int main(int argc, char **argv)
{
  double opt_tag_size = 0.05; //default tag size was originally 0.12
  double opt_desired_factor = 9.0; // default multiplier (new line added by Ulu 8/28/2025)
  bool opt_tag_z_aligned = false;
  std::string opt_robot_ip = "192.168.1.1";
  std::string opt_eMc_filename = "";
  std::string opt_intrinsic_filename = "";
  std::string opt_camera_name = "Camera";
  bool display_tag = true;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  double convergence_threshold = 0.00005;
  int no_tag_counter = 0;
  // ---- Lost-target backoff + scan params ----
  double scan_backoff_secs = 3.0;          // time to just back up before scanning
  double backoff_speed = 0.02;             // m/s during backoff (positive = forward, here we use negative)

  double scan_wx_amp = vpMath::rad(18);    // rad/s amplitude for pitch (rotate around x)
  double scan_wy_amp = vpMath::rad(18);    // rad/s amplitude for roll  (rotate around y)
  double scan_wz_amp = vpMath::rad(8);     // rad/s amplitude for a small yaw (around z)

  double scan_fx = 0.33;                   // Hz for pitch oscillation
  double scan_fy = 0.25;                   // Hz for roll oscillation
  double scan_fz = 0.20;                   // Hz for yaw oscillation

  double max_linear = 0.05;                // safety caps
  double max_angular = vpMath::rad(20);

  double lost_start_ms = -1.0;             // timestamp when we first lost the tag
  



  for (int i = 1; i < argc; ++i) {
    if ((std::string(argv[i]) == "--tag-size") && (i + 1 < argc)) {
      opt_tag_size = std::stod(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--desired-factor") && (i + 1 < argc)) {
      opt_desired_factor = std::stod(argv[++i]);
    }  
    else if ((std::string(argv[i]) == "--tag-quad-decimate") && (i + 1 < argc)) {
      opt_quad_decimate = std::stoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--tag-z-aligned") {
      opt_tag_z_aligned = true;
    }
    else if ((std::string(argv[i]) == "--ip") && (i + 1 < argc)) {
      opt_robot_ip = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      opt_intrinsic_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      opt_camera_name = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--eMc") && (i + 1 < argc)) {
      opt_eMc_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--verbose") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--adaptive-gain") {
      opt_adaptive_gain = true;
    }
    else if (std::string(argv[i]) == "--task-sequencing") {
      opt_task_sequencing = true;
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold") {
      convergence_threshold = 0.;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout << "SYNOPSYS" << std::endl
        << "  " << argv[0]
        << " [--ip <controller ip>]"
        << " [--intrinsic <xml file>]"
        << " [--camera-name <name>]"
        << " [--tag-size <size>]"
        << " [--tag-quad-decimate <decimation factor>]"
        << " [--tag-z-aligned]"
        << " [--eMc <extrinsic transformation file>]"
        << " [--adaptive-gain]"
        << " [--plot]"
        << " [--task-sequencing]"
        << " [--no-convergence-threshold]"
        << " [--verbose]"
        << " [--help] [-h]\n"
        << std::endl;
      std::cout << "DESCRIPTION" << std::endl
        << "  Use an image-based visual-servoing scheme to position the camera in front of an Apriltag." << std::endl
        << std::endl
        << "  --ip <controller ip>" << std::endl
        << "    Franka controller ip address" << std::endl
        << "    Default: " << opt_robot_ip << std::endl
        << std::endl
        << "  --intrinsic <xml file>" << std::endl
        << "    XML file that contains camera intrinsic parameters. " << std::endl
        << "    If no file is specified, use Realsense camera factory intrinsic parameters." << std::endl
        << std::endl
        << "  --camera-name <name>" << std::endl
        << "    Camera name in the XML file that contains camera intrinsic parameters." << std::endl
        << "    Default: \"Camera\"" << std::endl
        << std::endl
        << "  --tag-size <size>" << std::endl
        << "    Apriltag size in [m]." << std::endl
        << "    Default: " << opt_tag_size << " [m]" << std::endl
        << std::endl
        << "  --tag-z-aligned" << std::endl
        << "    When enabled, tag z-axis and camera z-axis are aligned." << std::endl
        << "    Default: false" << std::endl
        << std::endl
        << "  --eMc <extrinsic transformation file>" << std::endl
        << "    File containing the homogeneous transformation matrix between" << std::endl
        << "    robot end-effector and camera frame." << std::endl
        << std::endl
        << "  --tag-quad-decimate <decimation factor>" << std::endl
        << "    Decimation factor used during Apriltag detection." << std::endl
        << "    Default: " << opt_quad_decimate << std::endl
        << std::endl
        << "  --adaptive-gain" << std::endl
        << "    Flag to enable adaptive gain to speed up visual servo near convergence." << std::endl
        << std::endl
        << "  --plot" << std::endl
        << "    Flag to enable curve plotter." << std::endl
        << std::endl
        << "  --task-sequencing" << std::endl
        << "    Flag to enable task sequencing scheme." << std::endl
        << std::endl
        << "  --no-convergence-threshold" << std::endl
        << "    Flag to disable convergence threshold used to stop the visual servo." << std::endl
        << std::endl
        << "  --verbose" << std::endl
        << "    Flag to enable extra verbosity." << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Print this helper message." << std::endl
        << std::endl;

      return EXIT_SUCCESS;
    }
    else {
      std::cout << "\nERROR" << std::endl
        << std::string(argv[i]) << " command line option is not supported." << std::endl
        << "Use " << std::string(argv[0]) << " --help" << std::endl
        << std::endl;
      return EXIT_FAILURE;
    }
  }
  
  vpRealSense2 rs;
  rs2::config config;
  unsigned int width = 640, height = 480;
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
  rs.open(config);

  vpImage<unsigned char> I(height, width);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay(I, 10, 10, "Current image");
#endif

  std::cout << "Parameters:" << std::endl;
  std::cout << "  Apriltag                  " << std::endl;
  std::cout << "    Size [m]              : " << opt_tag_size << std::endl;
  std::cout << "    Z aligned             : " << (opt_tag_z_aligned ? "true" : "false") << std::endl;
  std::cout << "  Camera intrinsics         " << std::endl;
  std::cout << "    Factory parameters    : " << (opt_intrinsic_filename.empty() ? "yes" : "no") << std::endl;

  // Get camera intrinsics
  vpCameraParameters cam;
  if (opt_intrinsic_filename.empty()) {
    std::cout << "Use Realsense camera intrinsic factory parameters: " << std::endl;
    cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    std::cout << "cam:\n" << cam << std::endl;
  }
  else if (!vpIoTools::checkFilename(opt_intrinsic_filename)) {
    std::cout << "Camera parameters file " << opt_intrinsic_filename << " doesn't exist." << std::endl;
    return EXIT_FAILURE;
  }
  else {
    vpXmlParserCamera parser;
    if (!opt_camera_name.empty()) {

      std::cout << "    Param file name [.xml]: " << opt_intrinsic_filename << std::endl;
      std::cout << "    Camera name           : " << opt_camera_name << std::endl;

      if (parser.parse(cam, opt_intrinsic_filename, opt_camera_name, vpCameraParameters::perspectiveProjWithDistortion) !=
        vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Unable to parse parameters with distortion for camera \"" << opt_camera_name << "\" from "
          << opt_intrinsic_filename << " file" << std::endl;
        std::cout << "Attempt to find parameters without distortion" << std::endl;

        if (parser.parse(cam, opt_intrinsic_filename, opt_camera_name,
                         vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
          std::cout << "Unable to parse parameters without distortion for camera \"" << opt_camera_name << "\" from "
            << opt_intrinsic_filename << " file" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
  }

  std::cout << "Camera parameters used to compute the pose:\n" << cam << std::endl;

  // Setup Apriltag detector
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  // vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
  vpDetectorAprilTag detector(tagFamily);
  detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
  detector.setDisplayTag(display_tag);
  detector.setAprilTagQuadDecimate(opt_quad_decimate);
  detector.setZAlignedWithCameraAxis(opt_tag_z_aligned);

  // Setup camera extrinsics
  vpPoseVector e_P_c;
  // Set camera extrinsics default values
  e_P_c[0] = 0.0337731;
  e_P_c[1] = -0.00535012;
  e_P_c[2] = -0.0523339;
  e_P_c[3] = -0.247294;
  e_P_c[4] = -0.306729;
  e_P_c[5] = 1.53055;

  // If provided, read camera extrinsics from --eMc <file>
  if (!opt_eMc_filename.empty()) {
    e_P_c.loadYAML(opt_eMc_filename, e_P_c);
  }
  else {
    std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << std::endl;
  }
  vpHomogeneousMatrix e_M_c(e_P_c);
  std::cout << "e_M_c:\n" << e_M_c << std::endl;

  // Set desired pose used to compute the desired features
  vpHomogeneousMatrix cd_M_o(vpTranslationVector(0, 0, opt_tag_size * opt_desired_factor), // desired factor (default 9) times tag with along camera z axis (originally it was 3 times of tag size with default tag size 12)
                             vpRotationMatrix({ 1, 0, 0, 0, -1, 0, 0, 0, -1 }));

  vpRobotFranka robot;

  try {
    robot.connect(opt_robot_ip);

    // Create visual features
    std::vector<vpFeaturePoint> p(4), pd(4); // We use 4 points

    // Define 4 3D points corresponding to the CAD model of the Apriltag
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tag_size / 2., -opt_tag_size / 2., 0);
    point[1].setWorldCoordinates(+opt_tag_size / 2., -opt_tag_size / 2., 0);
    point[2].setWorldCoordinates(+opt_tag_size / 2., +opt_tag_size / 2., 0);
    point[3].setWorldCoordinates(-opt_tag_size / 2., +opt_tag_size / 2., 0);

    // Setup IBVS
    vpServo task;
    // Add the 4 visual feature points
    for (size_t i = 0; i < p.size(); ++i) {
      task.addFeature(p[i], pd[i]);
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
    }
    else {
      task.setLambda(0.5);
    }

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      plotter->initGraph(0, 8);
      plotter->initGraph(1, 6);
      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
    }

    bool final_quit = false;
    bool has_converged = false;
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

    static double t_init_servo = vpTime::measureTimeMs();

    robot.set_eMc(e_M_c); // Set location of the camera wrt end-effector frame
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    vpHomogeneousMatrix cd_M_c, c_M_o, o_M_o;

    while (!has_converged && !final_quit) {
      double t_start = vpTime::measureTimeMs();

      rs.acquire(I);

      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> c_M_o_vec;
      bool ret = detector.detect(I, opt_tag_size, cam, c_M_o_vec);

      {
        std::stringstream ss;
        ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
      }

      vpColVector v_c(6);

      // Only one tag is detected
      if (ret && (c_M_o_vec.size() == 1)) {
        no_tag_counter = 0; // reset
        c_M_o = c_M_o_vec[0];
        lost_start_ms = -1.0; // <--- add this line



        static bool first_time = true;
        if (first_time) {
          // Introduce security wrt tag positioning in order to avoid PI rotation
          std::vector<vpHomogeneousMatrix> secure_o_M_o(2), secure_cd_M_c(2);
          secure_o_M_o[1].buildFrom(0, 0, 0, 0, 0, M_PI);
          for (size_t i = 0; i < 2; ++i) {
            secure_cd_M_c[i] = cd_M_o * secure_o_M_o[i] * c_M_o.inverse();
          }
          if (std::fabs(secure_cd_M_c[0].getThetaUVector().getTheta()) < std::fabs(secure_cd_M_c[1].getThetaUVector().getTheta())) {
            o_M_o = secure_o_M_o[0];
          }
          else {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            o_M_o = secure_o_M_o[1]; // Introduce PI rotation
          }

          // Compute the desired position of the features from the desired pose
          for (size_t i = 0; i < point.size(); ++i) {
            vpColVector c_P, p;
            point[i].changeFrame(cd_M_o * o_M_o, c_P);
            point[i].projection(c_P, p);

            pd[i].set_x(p[0]);
            pd[i].set_y(p[1]);
            pd[i].set_Z(c_P[2]);
          }
        }

        // Get tag corners
        std::vector<vpImagePoint> corners = detector.getPolygon(0);

        // Update visual features
        for (size_t i = 0; i < corners.size(); ++i) {
          // Update the point feature from the tag corners location
          vpFeatureBuilder::create(p[i], cam, corners[i]);
          // Set the feature Z coordinate from the pose
          vpColVector c_P;
          point[i].changeFrame(c_M_o, c_P);

          p[i].set_Z(c_P[2]);
        }

        if (opt_task_sequencing) {
          if (!servo_started) {
            if (send_velocities) {
              servo_started = true;
            }
            t_init_servo = vpTime::measureTimeMs();
          }
          v_c = task.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
        }
        else {
          v_c = task.computeControlLaw();
        }

        // Display the current and desired feature points in the image display
        vpServoDisplay::display(task, cam, I);
        for (size_t i = 0; i < corners.size(); ++i) {
          std::stringstream ss;
          ss << i;
          // Display current point indexes
          vpDisplay::displayText(I, corners[i] + vpImagePoint(15, 15), ss.str(), vpColor::red);
          // Display desired point indexes
          vpImagePoint ip;
          vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
          vpDisplay::displayText(I, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        }
        if (first_time) {
          traj_corners = new std::vector<vpImagePoint>[corners.size()];
        }
        // Display the trajectory of the points used as features
        display_point_trajectory(I, corners, traj_corners);

        if (opt_plot) {
          plotter->plot(0, iter_plot, task.getError());
          plotter->plot(1, iter_plot, v_c);
          iter_plot++;
        }

        if (opt_verbose) {
          std::cout << "v_c: " << v_c.t() << std::endl;
        }

        double error = task.getError().sumSquare();
        std::stringstream ss;
        ss << "error: " << error;
        vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error: " << error << std::endl;

        if (error < convergence_threshold) {
          has_converged = true;
          std::cout << "Servo task has converged" << std::endl;
          vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
        }
        if (first_time) {
          first_time = false;
        }
      } // end if (c_M_o_vec.size() == 1)
      
      
      else {
        // Tag not detected
        if (lost_start_ms < 0.0) {
          lost_start_ms = vpTime::measureTimeMs(); // start the "lost" timer
        }

        const double now_ms = vpTime::measureTimeMs();
        const double dt = (now_ms - lost_start_ms) / 1000.0; // seconds since tag lost

        v_c = 0;
        v_c.resize(6);

        if (dt < scan_backoff_secs) {
          // Phase 1: back off to widen FOV
          v_c[2] = -std::min(backoff_speed, max_linear); // camera-frame +Z is forward; negative backs up
          vpDisplay::displayText(I, 60, 20, "No tag: backing up to widen FOV...", vpColor::yellow);
        } else {
          // Phase 2: head-tilt scan (pitch/roll/yaw oscillations)
          const double two_pi = 2.0 * M_PI;

          // angular velocities (rad/s) in camera frame
          double wx = scan_wx_amp * std::sin(two_pi * scan_fx * dt);            // pitch
          double wy = scan_wy_amp * std::sin(two_pi * scan_fy * dt + M_PI/2.0); // roll (phase shifted to vary)
          double wz = scan_wz_amp * std::sin(two_pi * scan_fz * dt);            // small yaw

          // safety clamp
          wx = std::max(-max_angular, std::min(max_angular, wx));
          wy = std::max(-max_angular, std::min(max_angular, wy));
          wz = std::max(-max_angular, std::min(max_angular, wz));

          // optional tiny lateral sweep to help (comment out if you prefer only rotation)
          double vx = 0.0;
          double vy = 0.0;
          // e.g., small lateral wiggle:
          // vx = 0.005 * std::sin(two_pi * 0.1 * dt);
          // vy = 0.005 * std::cos(two_pi * 0.1 * dt);

          // assign velocities (camera frame)
          v_c[0] = std::max(-max_linear, std::min(max_linear, vx));
          v_c[1] = std::max(-max_linear, std::min(max_linear, vy));
          v_c[2] = 0.0;  // stop backing up once scanning; or keep a tiny negative if you want continued retreat
          v_c[3] = wx;
          v_c[4] = wy;
          v_c[5] = wz;

          vpDisplay::displayText(I, 60, 20, "No tag: scanning (tilt/roll/yaw)...", vpColor::yellow);
        }
}


      if (!send_velocities) {
        v_c = 0;
      }

      // Send to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v_c);

      {
        std::stringstream ss;
        ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      }
      vpDisplay::flush(I);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        switch (button) {
        case vpMouseButton::button1:
          send_velocities = !send_velocities;
          break;

        case vpMouseButton::button3:
          final_quit = true;
          v_c = 0;
          break;

        default:
          break;
        }
      }
    }
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);

    if (opt_plot && plotter != nullptr) {
      delete plotter;
      plotter = nullptr;
    }

    if (!final_quit) {
      while (!final_quit) {
        rs.acquire(I);
        vpDisplay::display(I);

        vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
        vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

        if (vpDisplay::getClick(I, false)) {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }
    if (traj_corners) {
      delete[] traj_corners;
    }
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }
  catch (const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
      << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. "
      << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    return EXIT_FAILURE;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x and rebuild ViSP." << std::endl;
#endif
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka and rebuild ViSP." << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "Build ViSP with pugixml support enabled." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
