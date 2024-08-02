   #include "achal/bft_kvs.h"
#include "achal/eig_tree.h"
#include "applications/physical_ivp.h"
#include "common.h"
#include "utils/logging.h"
#include "utils/macros.h"
#include "utils/sched.h"
#include "utils/misc.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <pigpio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>

#define BFT_KVS_DEFAULT_NUM_ROUNDS 1
#define BFT_KVS_DEFAULT_NETWORK_DELAY_US 100
#define BFT_KVS_DEFAULT_PRIORITY 2
#define BFT_KVS_TASK_ID(NODE_ID, CPU_ID) ((1000 * NODE_ID) + (100 * CPU_ID))

log4cpp::Category *logger;

static uint8_t faulty_data[sizeof(struct Achal::eig_node<100, 7, 15>) * 100];

// GPIO pin defs
int motor_pin = 12, dir_pin = 16, us_trig = 17, us_echo = 27, lim_sw = 22;

// IDK WHAT THIS IS FOR
int get_period_based_priority(uint64_t period_ms) {
  if (period_ms <= 1) {
    return 26;
  } else if (period_ms <= 2) {
    return 24;
  } else if (period_ms <= 5) {
    return 23;
  } else if (period_ms <= 10) {
    return 22;
  } else if (period_ms <= 20) {
    return 21;
  } else if (period_ms <= 50) {
    return 20;
  } else if (period_ms <= 100) {
    return 19;
  } else if (period_ms <= 200) {
    return 18;
  }

  return 17;
}

int limit_switch = 22;
Applications::PositionTask *periodic_task;
bool runProgram = true;

void output_to_motor(double output) {
  gpioWrite(dir_pin, output > 0);
  output = abs(output)+ 25;
  if (output > 255)
    output = 255;
  // std::cout<<"motor output: " << output<<std::endl;
  gpioPWM(motor_pin, output);
}

void movetoPoint()
{

  Ultrasonic linear_position_encoder(us_trig, us_echo);
  auto distance = linear_position_encoder.detectDistance();
  std::cout << "MOVING" << std::endl;

  while (distance < 19.0 || distance > 21.0)
  {
    distance = linear_position_encoder.detectDistance();
    std::cout << "distance " << distance << std::endl;

    if (distance < 20.0)
    {
      output_to_motor(50);
    }
    else if (distance > 20.0)
    {
      output_to_motor(-50);
    }
  }

  output_to_motor(0);
  std::cout << "Stopping" << std::endl;
}

void limitSwitchISR(int gpio, int level, uint32_t tick) {
  std::cout << "ISR ENTERED" << std::endl;
  runProgram = false;
  this_thread::sleep_for(50ms);
  if (gpioRead(limit_switch)) {
    periodic_task->stop_flag = true;
    this_thread::sleep_for(1000ms);
    movetoPoint();
  }
  std::cout << "periodic task " <<periodic_task->stop_flag<< std::endl;

}

void pwm_output_test() {
  gpioSetMode(motor_pin, PI_OUTPUT);
  gpioSetMode(dir_pin, PI_OUTPUT);
  gpioSetPWMfrequency(motor_pin, 2000);
  while (true) {
    for (int i = 1; i < 20; i++) {
      std::cout << "PWM Duty Cycle: " << i * 10 << " out of 255" << std::endl;
      gpioPWM(motor_pin, i * 10);
      this_thread::sleep_for(5000ms);
    }
  }
}

class EnergyController{
public:
  
    EnergyController(double k_energy,double n_energy, double upright_threshold=0.2, double length=0.2): k_energy(k_energy), n_energy(n_energy), upright_threshold(upright_threshold), length(length){
      bar_mass = calculate_bar_mass(0.005, length);
      inertia = calculate_polar_inertia(bar_mass, length);
      std::cout << "k_energy " << k_energy << std::endl;
      std::cout<<"n_energy" << n_energy << std::endl;
      previous_output = k_energy*gravity;
    }
    double control_input = 0;
    double upright_threshold;
    double length;
    double bar_mass;
    double inertia;
    double gravity = 9.806;
    double k_energy;
    double n_energy;

    double previous_output;

    double calculate_bar_mass(double radius, double height){

      double density = 7850;
      double mass = density * 3.14159 * radius*radius * height;
        
      return mass;
    }

    double calculate_polar_inertia(double mass,double length){


      double inertia_pivot = (1 / 12) * mass * length*length;

      return inertia_pivot;
    }
    
    double total_energy(double angle, double angular_velocity){

      double potential_energy = -bar_mass * gravity * (length) * (std::cos(angle));

      double kinetic_energy = 0.5 * inertia * angular_velocity*angular_velocity;

      double total_energy = potential_energy;

      return total_energy;

    }
    
    double control(double angle, double angular_velocity){

      Ultrasonic linear_position_encoder(us_trig, us_echo);

      double reference_energy = total_energy(0, 0);
      
      double energy_error = total_energy(angle, angular_velocity) - reference_energy;
      // std::cout << "reference energy " << reference_energy << std::endl;
      // std::cout << "total energy " << total_energy(angle, angular_velocity) << std::endl;
      // std::cout << "energy error " << energy_error << std::endl;
      
      double control_output;
      double previous_output;

    
      double abs_ang_vel =std::abs(angular_velocity);
      double cosine = std::cos(angle);


      std::cout << "angle " << angle << " angular velocity " << angular_velocity << " energy error " << energy_error << " cosine " << cosine << std::endl;
      

      const auto start = std :: chrono :: steady_clock::now();
      const auto distance = linear_position_encoder.detectDistance();
      const auto end = std :: chrono :: steady_clock::now();
    
      const auto diff = end - start;
      std::cout <<"distance "<<distance <<" " <<std::chrono::duration_cast<std::chrono::milliseconds>(diff).count()<< std::endl;

      if ((angular_velocity*cosine)>0 ) {
        control_output = -std::clamp(n_energy*energy_error,0.0,k_energy*gravity);
        previous_output = control_output;
      } else if ((angular_velocity*cosine) < 0){
        control_output = std::clamp(n_energy*energy_error, 0.0, k_energy*gravity);
        previous_output = control_output;
      }
      else{
        control_output = 0;
      }

      if(distance <= 5.00 and std::signbit(control_output) == true){
        control_output = 0;
      } else if(distance >= 30.00 and std::signbit(control_output) == false){
        control_output = 0;
      }

      
      return control_output;
    }

  };



double uart_to_radians(int uart_value, int reference_angle) {
  const auto diff = 4096 - reference_angle;
  return static_cast<double>(uart_value + diff)*2.0*3.14159/4096.0;
}


int main(int argc, char **argv) {
  gpioTerminate();
  gpioInitialise();
  // limit_sw_test();
  // ultrasonic_test();
  // AS5600_test();
  // pwm_output_test();

  int node_id = std::stoi(argv[1]);
  int total_nodes = std::stoi(argv[2]);
  uint64_t period_ms = std::stoi(argv[3]);
  uint64_t ang_period_ms = std::stoi(argv[4]);
  double pos_Kp = std::stod(argv[5]);
  double pos_Kd = std::stod(argv[6]);
  double pos_Ki = std::stod(argv[7]);
  double ang_Kp = std::stod(argv[8]);
  double ang_Kd = std::stod(argv[9]);
  double ang_Ki = std::stod(argv[10]);
  double position_set_point;
  uint16_t angle_set_point;

  std::memset((void *)faulty_data, 3, sizeof(faulty_data));

  log4cpp::Category *logger;
  std::string def_log_file = "Pi" + std::to_string(node_id) + "_log.log";
  if (std::remove(def_log_file.c_str()) == 0) {
    std::cout << "Existing log file deleted successfully." << std::endl;
  } else {
    // Check if the file doesn't exist (errno == ENOENT)
    if (errno != ENOENT) {
      std::cerr << "Error deleting existing log file." << std::endl;
      return 1;
    }
  }

  bool logger_stats = Utils::get_new_logger(
      "PIvP", def_log_file, log4cpp::Priority::PriorityLevel::DEBUG, &logger);
  // if (gpioInitialise() < 0) {
  //   logger << "pigpio initialization failed, code " << std::to_string(gpioInitialise());
  //   return -1;
  // }

  // Use this to log to terminal
  // bool logger_stats = Utils::get_new_logger(
  //     "PIvP", log4cpp::Priority::PriorityLevel::DEBUG, &logger);

  Json::Value calibration;
  std::ifstream calibrationFile("exp/calibration.json");
  Json::CharReaderBuilder reader;
  JSONCPP_STRING errs;
  Json::parseFromStream(reader, calibrationFile, &calibration, &errs);
  calibrationFile.close();

  if (errs.empty()) {
    if (argc > 11)
      position_set_point = std::stod(argv[11]);
    else
      position_set_point = calibration["Position Set Point"].asDouble();

    if (argc > 12)
      angle_set_point = stoi(argv[12]);
    else
      angle_set_point = calibration["Angle Set Point"].asInt();
  }

  calibration["Position Set Point"] = position_set_point;
  calibration["Angle Set Point"] = angle_set_point;

  Json::StreamWriterBuilder writer;
  std::ofstream calibrationFileOut("exp/calibration.json");
  calibrationFileOut << Json::writeString(writer, calibration);
  calibrationFileOut.close();
  std::cout << "Configuration updated in calibration.json" << std::endl;

  std::string kvs_type = "BFTKVS";
  std::string output_file_name = "out_file";
  int cpus_used = 1;
  int num_tasks_per_cpu = 1;
  int fault_mode;
  int number_of_faulty_node;
  int num_nmr_instances = 1;
  uint64_t pivp_task_wcet_ms = 6;
  int exp_duration_s = 10000;
  int replication_factor = total_nodes;

  gpioInitialise();
  gpioSetMode(limit_switch, PI_INPUT);
  gpioSetPullUpDown(limit_switch, PI_PUD_DOWN);
  gpioSetISRFunc(limit_switch, RISING_EDGE, 0, limitSwitchISR);
  std::vector<Achal::KVSInterface *> kvs_instances;

  if (total_nodes < 2) {
    std::cout << "single node setup" << std::endl;
    Achal::config_t simple_kvs_config;
    simple_kvs_config.id = BFT_KVS_TASK_ID(node_id, 0);
    simple_kvs_config.cpu = 2;
    simple_kvs_config.period_ns = MS_TO_NS(period_ms);
    uint64_t nmr_specific_offset_ns =
        0 * (MS_TO_NS(period_ms) / num_nmr_instances);
    simple_kvs_config.offset_ns =
        nmr_specific_offset_ns +
        MS_TO_NS((num_tasks_per_cpu * pivp_task_wcet_ms));
    simple_kvs_config.logger = logger;
    simple_kvs_config.max_network_delay_ns =
        US_TO_NS(BFT_KVS_DEFAULT_NETWORK_DELAY_US);
    simple_kvs_config.max_rounds = BFT_KVS_DEFAULT_NUM_ROUNDS;
    simple_kvs_config.max_jobs =
        SEC_TO_NS(exp_duration_s) / simple_kvs_config.period_ns;
    simple_kvs_config.priority = BFT_KVS_DEFAULT_PRIORITY;

    auto temp = new Achal::SimpleKVS(simple_kvs_config);

    kvs_instances.push_back(temp);
  } else {
    std::cout << "multinode setup" << std::endl;
    std::vector<Achal::config_t> bft_kvs_configs;
    for (unsigned nmr_id = 0; nmr_id < num_nmr_instances; nmr_id++) {
      Achal::config_t bft_kvs_config;
      bft_kvs_config.period_ns = MS_TO_NS(period_ms);
      bft_kvs_config.priority = BFT_KVS_DEFAULT_PRIORITY;
      uint64_t nmr_specific_offset_ns =
          nmr_id * (MS_TO_NS(period_ms) / num_nmr_instances);
      bft_kvs_config.offset_ns =
          nmr_specific_offset_ns +
          MS_TO_NS((num_tasks_per_cpu * pivp_task_wcet_ms));
      bft_kvs_config.logger = logger;
      bft_kvs_config.max_network_delay_ns =
          US_TO_NS(BFT_KVS_DEFAULT_NETWORK_DELAY_US);
      bft_kvs_config.max_rounds = BFT_KVS_DEFAULT_NUM_ROUNDS;
      bft_kvs_config.max_jobs =
          SEC_TO_NS(exp_duration_s) / bft_kvs_config.period_ns;
      bft_kvs_config.max_failed_comms = 80;
      bft_kvs_config.failure_time_multiple = 100;
      bft_kvs_config.wcet_kvs = 0;

      for (int i = 0; i < replication_factor; i++) {
        Achal::process_t p;
        p.id = i;
        if (node_id <= 4) {
          p.ip = node_ips[i];
        } else {
          p.ip = node_ips[i + 4];
        }
        p.port = bft_kvs_ports[nmr_id][i];
        bft_kvs_config.peers.push_back(p);
      }

      bft_kvs_configs.push_back(bft_kvs_config);
    }

    std::cout << "BFT config done" << std::endl;
    // Assuming we only do distributed tests
    for (int i = 0; i < num_nmr_instances; i++) {
      Achal::config_t &bft_kvs_config = bft_kvs_configs[i];
      bft_kvs_config.port = std::stoi(bft_kvs_ports[i][(node_id - 1) % 4]);
      bft_kvs_config.id = BFT_KVS_TASK_ID(node_id, i);
      bft_kvs_config.cpu = i;
      bft_kvs_config.my_process_id = (node_id - 1) % 4;
      try {
        std::cout << "trying to instantiate KVS" << std::endl;
        auto temp = new Achal::BFTKVS<10, 7, 15>(bft_kvs_config);
        temp->update_fuse_function(Achal::median_of_doubles_null_ignore);
        kvs_instances.push_back(temp);
      } catch (const std::exception &e) {
        std::cerr << "Error in main: " << e.what() << std::endl;
      }
    }

    std::cout << "connecting to server" << std::endl;
    for (Achal::KVSInterface *kvs_instance : kvs_instances) {
      // cast to kvs, then connect to servers
      ((Achal::BFTKVS<10, 7, 15> *)kvs_instance)->connect_to_servers();
    }
    std::cout << "connected" << std::endl;

    for (Achal::KVSInterface *kvs_instance : kvs_instances) {
      ((Achal::BFTKVS<10, 7, 15> *)kvs_instance)->accept_clients();
    }
    std::cout << "accepted all clients" << std::endl;

    ((Achal::BFTKVS<10, 7, 15> *)kvs_instances[0])->reconnect_task->spawn();
    ((Achal::BFTKVS<10, 7, 15> *)kvs_instances[0])
        ->reconnect_task->collect_ets = true;
  }
  std::cout << "ang set point: " << angle_set_point << std::endl;
  std::cout << "pos set point: " << position_set_point << std::endl;

  periodic_task = new Applications::PositionTask(
      node_id, motor_pin, dir_pin, us_trig, us_echo, lim_sw, ang_Kp, ang_Kd,
      ang_Ki, angle_set_point, pos_Kp, pos_Kd, pos_Ki, position_set_point, 4, 5,
      MS_TO_NS(period_ms), MS_TO_NS(ang_period_ms), MS_TO_NS(0), MS_TO_NS(0), 3,
      10, 0, 1, kvs_instances[0], 100000, logger);

  kvs_instances[0]->collect_ets = true;
  periodic_task->collect_ets = true;
  periodic_task->angle_task->collect_ets = true;

  AS_5600 rotary_encoder{};
  auto t_old = std::chrono::steady_clock::now();
  auto k_energy = std::stoi(argv[13]);
  auto n_energy = std::stoi(argv[14]);
  EnergyController ec{k_energy, n_energy};

  int reference_angle = angle_set_point;
  std::cout << "Reference angle " << reference_angle << std::endl;
  double prev_angle = uart_to_radians(rotary_encoder.getAngleUART(), reference_angle);
  set_my_sched_fifo_priority(1);
  int count = 0;
    while(runProgram) 
    {
      count++;
      count %=2;
      auto start = std::chrono::steady_clock::now();
      auto raw = rotary_encoder.getAngleUART();

      double angle = uart_to_radians(raw, reference_angle);

      auto d_angle = angle - prev_angle;
      

      const auto t_new = std::chrono::steady_clock::now();

      auto dt = std::chrono::duration<double>(t_new - t_old).count();
      // std::cout << "d angle " << d_angle << " dt " << dt << std::endl;
      auto angular_vel = d_angle / dt;

      t_old = t_new;
      prev_angle = angle;

      if (count == 0) {

        auto output = ec.control(angle, angular_vel);
        std::cout << "output " << output << std::endl;

        output_to_motor(output);

      }

      std::cout << "raw " << raw << std::endl;
      if ((std::abs(raw - reference_angle) < 50)) {
        break;
      }

      auto end = std::chrono::steady_clock::now();
      using namespace std::chrono_literals;
      const auto min_dt{10ms};
      const std::chrono::duration<double, std::nano> time_spent{end - start};

      
      if (time_spent > min_dt) {
        std::cerr << "Skipping sleep\n";
      } else {
        std::this_thread::sleep_for(min_dt - time_spent);
      }
    }


  kvs_instances[0]->spawn();
  std::cout << "kvs spawned" << std::endl;
  periodic_task->spawn();
  periodic_task->angle_task->spawn();
  std::cout << "angle and position task spawned" << std::endl;
  periodic_task->join();
  std::cout << "position task done" << std::endl;
  periodic_task->angle_task->join();
  std::cout << "angle task done" << std::endl;
  kvs_instances[0]->join();
  std::cout << "kvs task done";

  kvs_instances[0]->print_stats();
  kvs_instances[0]->flush_exec_times(periodic_task->log_folder_path);
  if (total_nodes > 1)
    ((Achal::BFTKVS<10, 7, 15> *)kvs_instances[0])
        ->reconnect_task->flush_exec_times(periodic_task->log_folder_path);

  // Perform the move operation
  if (rename(def_log_file.c_str(),
             (periodic_task->log_folder_path + "/" + def_log_file).c_str()) ==
      0) {
    std::cout << "Log moved successfully." << std::endl;
  } else {
    std::cerr << "Error moving log: "
              << periodic_task->log_folder_path + "/" + def_log_file
              << std::endl;
    return 1;
  }

  std::cout << "task complete" << endl;
}
