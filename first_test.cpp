#include <iostream>
#include <pigpio.h>
#include "PIvPUtils/AS_5600.h"
#include "PIvPUtils/Ultrasonic.h"
#include <thread>
#include <algorithm>
#include <string>
#include <chrono>
#include <cmath>
#include "utils/sched.h"

constexpr int motor_pin{12};
constexpr int dir_pin{16};
constexpr int limit_switch{22};
constexpr int us_trig = 17;
constexpr int us_echo = 27;

class EnergyController
{
public:
  EnergyController(double k_energy, double n_energy, double upright_threshold = 0.2, double length = 0.2) : k_energy(k_energy), n_energy(n_energy), upright_threshold(upright_threshold), length(length)
  {
    bar_mass = calculate_bar_mass(0.005, length);
    inertia = calculate_polar_inertia(bar_mass, length);
    std::cout << "k_energy " << k_energy << std::endl;
    std::cout << "n_energy" << n_energy << std::endl;
    previous_output = k_energy * gravity;
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

  double calculate_bar_mass(double radius, double height)
  {

    double density = 7850;
    double mass = density * 3.14159 * radius * radius * height;

    return mass;
  }

  double calculate_polar_inertia(double mass, double length)
  {

    double inertia_pivot = (1 / 12) * mass * length * length;

    return inertia_pivot;
  }

  double total_energy(double angle, double angular_velocity)
  {

    double potential_energy = -bar_mass * gravity * (length) * (std::cos(angle));

    double kinetic_energy = 0.5 * inertia * angular_velocity * angular_velocity;

    double total_energy = potential_energy;

    return total_energy;
  }

  double control(double angle, double angular_velocity)
  {

    Ultrasonic linear_position_encoder(us_trig, us_echo);

    double reference_energy = total_energy(0, 0);

    double energy_error = total_energy(angle, angular_velocity) - reference_energy;
    // std::cout << "reference energy " << reference_energy << std::endl;
    // std::cout << "total energy " << total_energy(angle, angular_velocity) << std::endl;
    // std::cout << "energy error " << energy_error << std::endl;

    double control_output;
    double previous_output;

    // if(angular_velocity*std::cos(angle) >= 0){
    //   control_input = k_energy * energy_error * 100;
    // }else{
    //   control_input = -k_energy*energy_error*100;
    // }
    double abs_ang_vel = std::abs(angular_velocity);
    double cosine = std::cos(angle);

    //std::cout << "angle " << angle << " angular velocity " << angular_velocity << " energy error " << energy_error << " cosine " << cosine << std::endl;
    // if (abs_ang_vel != 0) {
    // return previous_output;
    //}

    const auto start = std ::chrono ::steady_clock::now();
    const auto distance = linear_position_encoder.detectDistance();
    const auto end = std ::chrono ::steady_clock::now();

    const auto diff = end - start;
    //std::cout << "distance " << distance << " " << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << std::endl;

    if ((angular_velocity * cosine) > 0)
    {
      control_output = -std::clamp(n_energy * energy_error, 0.0, k_energy * gravity);
      previous_output = control_output;
    }
    else if ((angular_velocity * cosine) < 0)
    {
      control_output = std::clamp(n_energy * energy_error, 0.0, k_energy * gravity);
      previous_output = control_output;
    }
    else
    {
      control_output = 0;
    }

    if (distance <= 5.00 and std::signbit(control_output) == true)
    {
      control_output = 0;
    }
    else if (distance >= 30.00 and std::signbit(control_output) == false)
    {
      control_output = 0;
    }

    return control_output;
  }
};

void output_to_motor(double output)
{
  gpioWrite(dir_pin, output > 0);
  output = abs(output) + 25;
  if (output > 255)
    output = 255;
  // std::cout<<"motor output: " << output<<std::endl;
  gpioPWM(motor_pin, output);
}

void pwm_output_test()
{
  gpioSetMode(motor_pin, PI_OUTPUT);
  gpioSetMode(dir_pin, PI_OUTPUT);
  gpioSetPWMfrequency(motor_pin, 2000);
  while (true)
  {
    for (int i = 1; i < 20; i++)
    {
      std::cout << "PWM Duty Cycle: " << i * 10 << " out of 255" << std::endl;
      gpioPWM(motor_pin, i * 10);
      std::this_thread::sleep_for(5000ms);
    }
  }
}

bool runProgram{true};

void limitSwitchISR(int gpio, int level, uint32_t tick)
{
  std::cout << "ISR ENTERED" << std::endl;
  runProgram = false;
  gpioPWM(motor_pin, 0);
}

double uart_to_radians(int uart_value, int reference_angle)
{
  const auto diff = 4096 - reference_angle;
  return static_cast<double>(uart_value + diff) * 2.0 * 3.14159 / 4096.0;
}

void movetoPoint()
{

  Ultrasonic linear_position_encoder(us_trig, us_echo);
  auto distance = linear_position_encoder.detectDistance();

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

int main(int argc, char **argv)
{
  
  AS_5600 rotary_encoder{};

  gpioTerminate();

  gpioInitialise();
  gpioSetMode(limit_switch, PI_INPUT);
  gpioSetPullUpDown(limit_switch, PI_PUD_DOWN);
  gpioSetISRFunc(limit_switch, RISING_EDGE, 0, limitSwitchISR);

  if (gpioInitialise() < 0)
  {
    std::cerr << "gpio failed initialize" << std::endl;
    return -1;
  }
  if (gpioSetMode(motor_pin, PI_OUTPUT) != 0)
  {
    std::cerr << "gpio failed setmode for motor pin" << std::endl;
    return -1;
  }
  if (gpioSetMode(dir_pin, PI_OUTPUT) != 0)
  {
    std::cerr << "gpio failed setmode for dir pin" << std::endl;
    return -1;
  }
  if (gpioSetPWMfrequency(motor_pin, 2000) == PI_BAD_USER_GPIO)
  {
    std::cerr << "failed to set pwm freq" << std::endl;
    return -1;
  }

  std::cout << "After set ISR" << std::endl;
  auto t_old = std::chrono::steady_clock::now();
  auto k_energy = std::stoi(argv[1]);
  auto n_energy = std::stoi(argv[2]);
  auto angle_set_point = std::stoi(argv[3]);
  EnergyController ec{k_energy, n_energy};

  int reference_angle = angle_set_point;
  std::cout << "Reference angle " << reference_angle << std::endl;
  double prev_angle = uart_to_radians(rotary_encoder.getAngleUART(), reference_angle);
  set_my_sched_fifo_priority(1);
  int count = 0;
  int iterations = std::stoi(argv[4]);
  int index = 0;
  
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
        //std::cout << "output " << output << std::endl;

        output_to_motor(output);

      }

      //std::cout << "raw " << raw << std::endl;
      if ((std::abs(raw - reference_angle) < 50)) {
        output_to_motor(0); 
        std::this_thread::sleep_for(3000ms);
        movetoPoint();
        std::this_thread::sleep_for(1000ms);

        std::cout << "index " << index << "iterations " << iterations << std::endl;
        
        index++;

        if(index == iterations){
          runProgram = false;   
          break;
        }
        
        
      }

      auto end = std::chrono::steady_clock::now();
      using namespace std::chrono_literals;
      const auto min_dt{10ms};
      const std::chrono::duration<double, std::nano> time_spent{end - start};

      // std::cout<<"time spent "<<time_spent.count()<<std::endl;
      if (time_spent > min_dt) {
        std::cerr << "Skipping sleep\n";
      } else {
        std::this_thread::sleep_for(min_dt - time_spent);
      }
    }

  output_to_motor(0);  

  gpioPWM(motor_pin, 0);
  gpioTerminate();
}