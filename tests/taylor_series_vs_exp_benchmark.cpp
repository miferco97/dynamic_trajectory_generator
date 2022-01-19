

#include <benchmark/benchmark.h>
#include <cmath>
#include <exception>
#include <iostream>

class gaussian
{
private:
  double mean_;
  double sigma_;

public:
  gaussian(double mean, double sigma)
      : mean_(mean), sigma_(sigma){};

  double operator()(double x) const
  {
    x = x - mean_;
    return std::exp(-std::pow(x, 2) / (2 * std::pow(sigma_, 2)));
  }
};

class taylorSerie5terms
{
private:
  double mean_;
  double sigma_;
  double a, b, c, d, e;

public:
  taylorSerie5terms(double mean, double sigma)
      : mean_(mean), sigma_(sigma) { set_coefficients(mean, sigma); };

  double operator()(double x) const
  {
    x = x - mean_;
    return a + b * std::pow(x, 2) + c * std::pow(x, 4) + d * std::pow(x, 6) + e * std::pow(x, 8); // 5 terms
  }

  // overload add operator
  taylorSerie5terms operator+(const taylorSerie5terms &other) const
  {
    taylorSerie5terms result(mean_, sigma_);
    result.a = a + other.a;
    result.b = b + other.b;
    result.c = c + other.c;
    result.d = d + other.d;
    result.e = e + other.e;

    return result;
  }

private:
  void set_coefficients(double mean, double sigma)
  {
    a = 1.0f;
    b = -1.0f / (2.0f * std::pow(sigma, 2));
    c = 1.0f / (8.0f * std::pow(sigma, 4));
    d = -1.0f / (48.0f * std::pow(sigma, 6));
    e = 1.0f / (384.0f * std::pow(sigma, 8));
  }
};

void generateRandomMeanAndSigma(double &mean, double &sigma)
{
  mean = 10 * (std::rand() % 1000) / 1000.0f;
  sigma = 10 * (std::rand() % 1000) / 1000.0f;
}
void generaterRandomX(double &x)
{
  x = 100 * (std::rand() % 1000) / 1000.0f;
}

double add_multiple_gaussians(const std::vector<gaussian> &gausians, double x)
{
  double result = 0;
  for (auto &g : gausians)
  {
    result += g(x);
  }
  return result;
}

double add_multiple_taylor(taylorSerie5terms &taylor, double x)
{
  return taylor(x);
}

static void
BM_GAUSSIAN_COMPUTE(benchmark::State &state)
{
  double mean, sigma, x;
  generateRandomMeanAndSigma(mean, sigma);
  gaussian(mean, sigma);
  auto g = gaussian(mean, sigma);
  for (auto _ : state)
  {
    g(x);
  }
}
BENCHMARK(BM_GAUSSIAN_COMPUTE)->Threads(1)->Repetitions(10);

static void
BM_TAYLOR_COMPUTE(benchmark::State &state)
{
  double mean, sigma, x;
  generateRandomMeanAndSigma(mean, sigma);
  taylorSerie5terms(mean, sigma);
  auto g = taylorSerie5terms(mean, sigma);

  for (auto _ : state)
  {
    g(x);
  }
}
BENCHMARK(BM_TAYLOR_COMPUTE)->Threads(1)->Repetitions(10);

static void
BM_GAUSSIAN_ADDING(benchmark::State &state)
{
  for (auto _ : state)
  {
    state.PauseTiming();
    std::vector<gaussian> gaussians(state.range(0), gaussian(1, 1));
    double mean, sigma, x;
    generaterRandomX(x);
    for (int i = 0; i < gaussians.size(); i++)
    {
      generateRandomMeanAndSigma(mean, sigma);
      gaussians[i] = gaussian(mean, sigma);
    }

    state.ResumeTiming();
    add_multiple_gaussians(gaussians, x);
  }
}

BENCHMARK(BM_GAUSSIAN_ADDING)->Range(1, 1 << 10);

static void
BM_TAYLOR_ADDING(benchmark::State &state)
{
  double mean, sigma, x;
  generateRandomMeanAndSigma(mean, sigma);
  generaterRandomX(x);
  taylorSerie5terms taylor(mean, sigma);

  for (auto _ : state)
  {
    state.PauseTiming();
    for (int i = 0; i < state.range(0); i++)
    {
      double mean, sigma, x;
      generateRandomMeanAndSigma(mean, sigma);
      generaterRandomX(x);
      taylorSerie5terms taylor2(mean, sigma);
      taylor = taylor + taylor2;
    }
    state.ResumeTiming();
    add_multiple_taylor(taylor, x);
  }
}

BENCHMARK(BM_TAYLOR_ADDING)->Range(1, 1 << 10);

int main(int argc, char **argv)
{
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}
