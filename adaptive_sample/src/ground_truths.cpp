#include <vector>
#include <random>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

struct Gaussian {
    /*
    Multivariate normal, to use in assembling an environment model
    Precomputres the covariance inverse and normalization constant
    to make sampling easier
    */
    double weight;
    Vec2 mean;
    Mat2 cov;
    Mat2 cov_inv;
    double norm_denominator;

    void precompute() {
        double det = cov.determinant();
        if (det <= 0.0) {
            throw std::runtime_error("Gaussian has negative determinant");
        }
        cov_inv = cov.inverse();

        const double PI = 3.14159265358979323846;
        norm_denominator = 1.0 / (2.0 * PI * std::sqrt(det));
    }

    double evaluate(const Vec2& x) const {
        Vec2 diff = x - mean;
        double quad = diff.transpose() * cov_inv * diff;
        return norm_denominator * std::exp(-0.5 * quad);
    }

    Gaussian(double w, Vec2 m, Mat2 c) : weight(w), mean(m), cov(c) {
        precompute();
    }
};

class Environment {
    // Abstract class for the different ground truth functions
    public:
        virtual ~Environment() = default;
        virtual double draw_sample(double x, double y) = 0;
};

class SumGaussians : Environment {
    // Generates a scalar field composed of a sum of different 2D gaussians
    public:
        SumGaussians(int num_gaussians) : 
        rand_gen(std::chrono::system_clock::now().time_since_epoch().count()),
        x_mean_rand(2.0,8.0),
        y_mean_rand(2.0,8.0),
        weight_rand(0.0,1.0),
        var_rand(0.0,8.0),
        correlation_rand(-1.0,1.0)
        {
            gaussians.reserve(num_gaussians);
            for (int i = 0; i < num_gaussians; ++i) {
                double w = weight_rand(rand_gen);
                double mx = x_mean_rand(rand_gen);
                double my = y_mean_rand(rand_gen);
                double var_x = var_rand(rand_gen);
                double var_y = var_rand(rand_gen);
                double corr = correlation_rand(rand_gen);

                // Variance should be positive
                if (var_x < 0) var_x *= -1;
                if (var_y < 0) var_y *= -1;

                // I want both variances to be fairly close --- avoids very thin gaussians
                double proportion = 0.6;
                var_y = proportion * var_x + (1 - proportion) * var_y;

                Vec2 m(mx,my);
                Mat2 cov;
                cov << var_x * var_x, var_x * var_y * corr, var_x * var_y * corr, var_y * var_y;

                gaussians.emplace_back(Gaussian(w,m,cov));
            }
        }
        double draw_sample(double x, double y) override {
            double val = 0;
            Vec2 v(x,y);
            for (Gaussian& component : gaussians) {
                val += component.weight * component.evaluate(v);
            }
            return val;
        }
    private:
        std::mt19937 rand_gen;
        std::uniform_real_distribution<double> x_mean_rand;
        std::uniform_real_distribution<double> y_mean_rand;
        std::normal_distribution<double> weight_rand;
        std::normal_distribution<double> var_rand;
        std::uniform_real_distribution<double> correlation_rand;
        std::vector<Gaussian> gaussians;
};

class CenteredGaussian : Environment {
    // Original function used for testing, centered at 5,5
    public:
        double draw_sample(double x, double y) {
            return exp(-0.25*(pow(x - 5.0,2)+3.0*pow(y - 5.0,2)));
        }
};