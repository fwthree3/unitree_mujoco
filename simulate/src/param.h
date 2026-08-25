#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

namespace param
{

constexpr int IDL_AUTO = -1;
constexpr int IDL_GO = 0;
constexpr int IDL_HG = 1;
constexpr int IDL_GO_MOTOR_LIMIT = 20;
constexpr int IDL_HG_MOTOR_LIMIT = 35;

inline int ResolveIdlType(const std::string &robot, int actuator_count, int configured_idl_type)
{
    if (configured_idl_type != IDL_AUTO &&
        configured_idl_type != IDL_GO &&
        configured_idl_type != IDL_HG)
    {
        throw std::invalid_argument("idl_type must be -1 (auto), 0 (unitree_go), or 1 (unitree_hg)");
    }

    int idl_type = configured_idl_type;
    if (idl_type == IDL_AUTO)
    {
        idl_type = robot == "as2" || actuator_count > IDL_GO_MOTOR_LIMIT ? IDL_HG : IDL_GO;
    }

    const int motor_limit = idl_type == IDL_GO ? IDL_GO_MOTOR_LIMIT : IDL_HG_MOTOR_LIMIT;
    if (actuator_count > motor_limit)
    {
        throw std::invalid_argument(
            std::string(idl_type == IDL_GO ? "unitree_go" : "unitree_hg") +
            " IDL supports at most " + std::to_string(motor_limit) +
            " actuators, but the model has " + std::to_string(actuator_count));
    }

    return idl_type;
}

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;
    int idl_type = IDL_AUTO;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // kaon fork addition (#192 domain-randomization sweep): sim2sim-only perturbations
    // applied in the bridge, not present upstream. All default to off/0 so an unmodified
    // config.yaml (no such keys) reproduces the exact prior behavior.
    int actuator_delay_steps = 0;      // motor_cmd is applied this many 1kHz bridge ticks late (base/floor)
    // CAN bus nodes don't all see the same effective latency (arbitration order, frame
    // position on a shared bus) -- actuator_delay_jitter_steps > 0 draws each motor's own
    // extra delay ~ Uniform[0, jitter_steps] ONCE at bridge start (a per-run structural
    // offset, not per-tick noise), so motor i's total delay is
    // actuator_delay_steps + draw_i, fixed for the run. 0 (default) keeps every motor at
    // exactly actuator_delay_steps, i.e. the old uniform-delay behavior.
    int actuator_delay_jitter_steps = 0;
    int actuator_delay_seed = -1;      // -1 = seed from random_device (default); >=0 for a reproducible draw
    double gyro_noise_std = 0.0;       // additive Gaussian noise, rad/s
    double acc_noise_std = 0.0;        // additive Gaussian noise, m/s^2
    double joint_pos_noise_std = 0.0;  // additive Gaussian noise, rad
    double joint_vel_noise_std = 0.0;  // additive Gaussian noise, rad/s

    // kaon fork addition (#192): elastic_band's own default length_=0.0 (main.cc) makes
    // it pull all the way to point_=(0,0,3) regardless of the robot's actual pose -- the
    // manual workflow instead nudges length_ toward the current torso-to-anchor distance
    // (the '8'/'down' key) until the pull is negligible, so it only re-engages as a
    // catch-net if the robot actually starts to sag. band_length lets a harness set that
    // target distance directly instead of walking it there one key-repeat at a time.
    // Empty (default) leaves elastic_band.length_ at its own 0.0 default, unchanged.
    double band_length = -1.0;  // <0 = leave elastic_band's own default; >=0 = set length_ to this
    // FixStand/Passive have no active balance control at all (fixed PD trajectory /
    // damping only -- see State_FixStand.h, State_Passive.h), so on a real robot too the
    // band (or a person) must hold it up until the *learned* policy (Velocity/Mimic)
    // actually engages; only then is it safe to let go. band_release_file, if non-empty,
    // is polled (see main.cc's PhysicsThread) and once it exists, elastic_band.enable_ is
    // set false exactly once -- letting a harness release the band the moment it has
    // confirmed (over DDS, independently of this process) that Velocity is up and stable,
    // without needing to inject a keypress into simulate's GLFW window.
    std::string band_release_file = "";

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            if (cfg["idl_type"])
            {
                idl_type = cfg["idl_type"].as<int>();
            }
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            if (cfg["actuator_delay_steps"]) actuator_delay_steps = cfg["actuator_delay_steps"].as<int>();
            if (cfg["actuator_delay_jitter_steps"]) actuator_delay_jitter_steps = cfg["actuator_delay_jitter_steps"].as<int>();
            if (cfg["actuator_delay_seed"]) actuator_delay_seed = cfg["actuator_delay_seed"].as<int>();
            if (cfg["gyro_noise_std"]) gyro_noise_std = cfg["gyro_noise_std"].as<double>();
            if (cfg["acc_noise_std"]) acc_noise_std = cfg["acc_noise_std"].as<double>();
            if (cfg["joint_pos_noise_std"]) joint_pos_noise_std = cfg["joint_pos_noise_std"].as<double>();
            if (cfg["joint_vel_noise_std"]) joint_vel_noise_std = cfg["joint_vel_noise_std"].as<double>();
            if (cfg["band_length"]) band_length = cfg["band_length"].as<double>();
            if (cfg["band_release_file"]) band_release_file = cfg["band_release_file"].as<std::string>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("idl_type,t", po::value<int>(&config.idl_type), "DDS IDL type: -1 auto, 0 unitree_go, 1 unitree_hg")
        ("actuator-delay-steps", po::value<int>(&config.actuator_delay_steps), "Motor cmd delay, in 1kHz bridge ticks")
        ("actuator-delay-jitter-steps", po::value<int>(&config.actuator_delay_jitter_steps), "Extra per-motor random delay spread (CAN-style async), in 1kHz ticks")
        ("actuator-delay-seed", po::value<int>(&config.actuator_delay_seed), "Seed for the per-motor delay jitter draw; -1 = random")
        ("gyro-noise-std", po::value<double>(&config.gyro_noise_std), "Additive gyro noise stddev, rad/s")
        ("acc-noise-std", po::value<double>(&config.acc_noise_std), "Additive accelerometer noise stddev, m/s^2")
        ("joint-pos-noise-std", po::value<double>(&config.joint_pos_noise_std), "Additive joint position noise stddev, rad")
        ("joint-vel-noise-std", po::value<double>(&config.joint_vel_noise_std), "Additive joint velocity noise stddev, rad/s")
        ("band-length", po::value<double>(&config.band_length), "elastic_band.length_ override; <0 leaves its own default")
        ("band-release-file", po::value<std::string>(&config.band_release_file), "path polled to auto-disable the elastic band once it exists")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}