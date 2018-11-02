#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // steering and throttle PID controllers
  PID pid_s, pid_t;
  // TODO: Initialize the pid variable.
  //final parameter without throttle (0.15,0.0001,3) ;

  //Parameters with throttle pid control

  //pid_s.Init(0.15, 0.0001, 3);
  //pid_t.Init(0.3, 0.0000, 0.1);

  // twiddle update - error: 388.798
  //pid_s.Init(0.17985 0.0001, 3);
  //pid_t.Init(0.3597, 0.0000, 0.1);

  // twiddle update - error: 194.412 - max throttle 0.65
  // pid_s.Init(0.16677, 0.000122089, 3.53487);
  // pid_t.Init(0.333539, 0.0000, 0.117829);

  // twiddle update - error: 154.358 - max throttle 0.8
  //pid_s.Init(0.181112, 0.0001211, 3.49634);
  //pid_t.Init(0.362221, 0.0000, 0.116545);

  pid_s.Init(0.181112, 0.0001211, 3.49634);
  pid_t.Init(0.362221, 0, 0.116545);

  h.onMessage([&pid_s, &pid_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // update error and calculate steer_value at each step
          pid_s.UpdateError(cte);
          steer_value = - pid_s.Kp * pid_s.p_error 
                        - pid_s.Kd * pid_s.d_error 
                        - pid_s.Ki * pid_s.i_error;

          // update error and calculate throttle_value at each step
          pid_t.UpdateError(fabs(cte));     // |cte|
          //pid_t.UpdateError(pow(cte, 2));   // cte^2
          throttle_value = 0.65 - pid_t.Kp * pid_t.p_error
                           - pid_t.Kd * pid_t.d_error 
                           - pid_t.Ki * pid_t.i_error;

          // DEBUG
          /*
          std::cout << "Steer value breakdown: " << std::endl;
          std::cout << "P: " << - pid_s.Kp * pid_s.p_error << std::endl;
          std::cout << "D: " << - pid_s.Kd * pid_s.d_error << std::endl; 
          std::cout << "I: " << - pid_s.Ki * pid_s.i_error << std::endl;
          */

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << //throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
