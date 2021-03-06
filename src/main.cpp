#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"


// Objectives : Speed, Accuracy

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const int actuator_delay = 100;
double Lf = 2.67;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

					// Convert Map Ref Cords to Car Ref Cords
					for (int i = 0; i < ptsx.size(); i++){
						double shift_x = ptsx[i]-px;
						double shift_y = ptsy[i]-py;
						ptsx[i] = (shift_x * cos(0-psi) - shift_y*sin(0 - psi));
						ptsy[i] = (shift_x * sin(0-psi) + shift_y*cos(0 - psi));
					}


					// Convert vector Doubles to Eigens
					// Usefull for passing to polyfit
					double* ptrx = &ptsx[0];
					double* ptry = &ptsy[0];
					Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
					Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

					// Calculate our poly coeffs
          //auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);




          // Calculate vehicle state at t=t
          double x = 0;
          // double y = polyeval(coeffs, x);
          double y = 0;
          double v_mps = v * 0.44704;
                 psi = psi;
          double cte = polyeval(coeffs, x);
          double epsi = -atan(coeffs[1]);

          // Calcuate Constants
          double delta = j[1]["steering_angle"];
          delta = -delta;
          double throttle_value = j[1]["throttle"];
          double t_d = actuator_delay/1000.0;
          double a = throttle_value * 10 * 0.44704; //V is

          // Calculate vehicle state at t=t+1
          x = (v_mps + .5*a*t_d) * t_d * cos(epsi);
          y = (v_mps + .5*a*t_d) * t_d * sin(epsi);
          psi = v * delta * t_d /Lf;
          cte += v_mps * sin(epsi) * t_d;
          epsi += v * delta * t_d/Lf;
          v = v_mps + a * t_d; // * .44704;

          // Set State Vector to t+latency
					Eigen::VectorXd state(6);
					state << x, y, psi, v, cte, epsi;



          // Optomize next acutator states and predict x/y coords
					auto vars = mpc.Solve(state, coeffs);

					vector<double> next_x_vals;
					vector<double> next_y_vals;



					double poly_inc = 2.5;
					double num_points = 25;
					for (int i = 1; i < num_points; i++){
						next_x_vals.push_back(poly_inc*i);
						next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
					}
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;

					mpc_x_vals.push_back(0);
					mpc_y_vals.push_back(0);
					for (int i = 2; i < vars.size(); i++) {
						if (i % 2 == 0) {
							mpc_x_vals.push_back(vars[i]);
						} else {
							mpc_y_vals.push_back(vars[i]);
						}
					}


					json msgJson;

          // Throttle Measurement Settings
          //msgJson["steering_angle"] = 0;
          //msgJson["throttle"] = 1;

          // Set throttle value
          msgJson["steering_angle"] = -vars[0]/(deg2rad(25)*Lf);
          msgJson["throttle"] = vars[1];


          next_x_vals = {2.5};
          next_y_vals = {0};
          // Set Next Car Position (Green Dots)
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

          mpc_x_vals = {2.5};
          mpc_y_vals = {0};
          // Set Polynomial Values (Yellow Dots)
					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(actuator_delay));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
