#pragma once
// 2nd-order Bessel low-pass, discretised with the bilinear transform
// (frequency pre-warped so that the -3 dB point lands exactly on fc).
//
//   analog prototype  H(s) = 3 / (s'^2 + 3 s' + 3),  s' = s / k
//   -3 dB of the prototype is at w = 1.36165412871613 rad/s  =>  k = w_c / 1.3617
//   DC group delay = 1 / k  [s]   (e.g. fc = 15 Hz -> ~14.4 ms)
//
// Implementation: transposed direct form II biquad.  Header-only, no ROS deps.
#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>

namespace mqtt_teleop {

class Bessel2LowPass {
 public:
  Bessel2LowPass() { design(15.0, 200.0); }

  void design(double fc_hz, double fs_hz) {
    if (fs_hz <= 0.0 || fc_hz <= 0.0) {
      throw std::invalid_argument("Bessel2LowPass: fc and fs must be > 0");
    }
    fc_ = std::min(fc_hz, 0.45 * fs_hz);  // keep well below Nyquist
    fs_ = fs_hz;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kW3dB = 1.36165412871613;  // 3 dB point of s^2+3s+3
    const double T = 1.0 / fs_;
    const double c = 2.0 / T;
    const double wa = c * std::tan(kPi * fc_ * T);  // pre-warped analog w_c
    k_ = wa / kW3dB;
    const double k2 = k_ * k_;
    const double a0 = c * c + 3.0 * k_ * c + 3.0 * k2;
    b0_ = 3.0 * k2 / a0;
    b1_ = 6.0 * k2 / a0;
    b2_ = 3.0 * k2 / a0;
    a1_ = (-2.0 * c * c + 6.0 * k2) / a0;
    a2_ = (c * c - 3.0 * k_ * c + 3.0 * k2) / a0;
    reset(0.0);
  }

  // Put the filter in steady state for a constant input/output y0 (no start-up step).
  void reset(double y0) {
    z1_ = (1.0 - b0_) * y0;
    z2_ = (b2_ - a2_) * y0;
  }

  double process(double x) {
    const double y = b0_ * x + z1_;
    z1_ = b1_ * x - a1_ * y + z2_;
    z2_ = b2_ * x - a2_ * y;
    return y;
  }

  // |H(e^{jwT})| in dB at frequency f_hz
  double gain_db(double f_hz) const {
    constexpr double kPi = 3.14159265358979323846;
    const std::complex<double> zi = std::polar(1.0, -2.0 * kPi * f_hz / fs_);  // z^-1
    const std::complex<double> num = b0_ + b1_ * zi + b2_ * zi * zi;
    const std::complex<double> den = 1.0 + a1_ * zi + a2_ * zi * zi;
    return 20.0 * std::log10(std::abs(num / den));
  }

  double group_delay_dc_s() const { return 1.0 / k_; }
  double cutoff_hz() const { return fc_; }
  double sample_rate_hz() const { return fs_; }

 private:
  double fc_{15.0}, fs_{200.0}, k_{1.0};
  double b0_{1}, b1_{0}, b2_{0}, a1_{0}, a2_{0};
  double z1_{0}, z2_{0};
};

}  // namespace mqtt_teleop
